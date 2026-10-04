/*
 * bebot_esp32.ino
 *
 * ESP32 Dev Module port of bebot_arduino.ino. Runs the same skid-steer
 * drive: 4 DC motors (front-left, front-right, rear-left, rear-right)
 * driven by two BTS7960 (IBT-2) dual half-bridge modules, one per side,
 * with the two motors on a side wired in parallel to the same module so
 * they always turn together.
 *
 * Unlike the Uno version, encoders are on the FRONT wheels (this robot's
 * actual hardware) -- rear wheels are not instrumented.
 *
 * What's different from the Uno version, and why:
 *   - R_EN and L_EN per side are tied to ONE shared GPIO each (not four
 *     separate pins) -- they never need to differ; see the wiring
 *     diagram's "why two enables" discussion. Saves 2 GPIOs.
 *   - PWM uses the ESP32's hardware LEDC peripheral (ledcAttach/ledcWrite)
 *     at 20kHz/10-bit instead of the Uno's ~490Hz/8-bit analogWrite --
 *     quieter (above audible range) and finer speed steps. The incoming
 *     [-255,255] command range is unchanged (scaled up internally), so
 *     the serial protocol and serial_odometry_node.py need no changes.
 *   - Both encoder channels (not just A) are interrupt-driven for full
 *     4x quadrature decoding, since the ESP32 (unlike the Uno) supports
 *     interrupts on every GPIO, not just two. Roughly 2x the counts per
 *     revolution versus the Uno version's single-interrupt-plus-level-read
 *     technique.
 *
 * Serial protocol (unchanged -- matches bebot_firmware/serial_odometry_node.py):
 *   ESP32 -> Jetson : "E,<left_ticks>,<right_ticks>\n"   every ENC_PUBLISH_INTERVAL_MS
 *   Jetson -> ESP32 : "M,<left_pwm>,<right_pwm>\n"       pwm in [-255, 255]
 *
 * Safety: if no "M," command is received for CMD_TIMEOUT_MS, motors stop.
 *
 * IMPORTANT hardware note: GPIO34/35/36/39 (used for encoders below) are
 * ESP32 input-only pins with NO internal pull-up/pull-down circuitry --
 * pinMode(..., INPUT_PULLUP) is a silent no-op on them. This is fine for
 * push-pull Hall-effect encoder outputs (the common case), but if your
 * encoder is open-collector/open-drain you MUST add external ~10k pull-up
 * resistors to 3.3V on these 4 pins yourself.
 */

#include <Arduino.h>

// ----------------------------- Pin map -------------------------------
// Motor driver pins (BTS7960 / IBT-2 dual half-bridge, one module per side).
// RPWM drives the module's "forward" half-bridge, LPWM the "reverse" one.
// If a side spins backwards for a positive command, swap that side's
// M1/M2 leads at the motor (or swap the RPWM/LPWM constants below) rather
// than touching the ROS-side sign convention.
const uint8_t LEFT_RPWM  = 25;
const uint8_t LEFT_LPWM  = 26;
const uint8_t LEFT_EN    = 27;  // drives both R_EN and L_EN, tied together
const uint8_t RIGHT_RPWM = 14;
const uint8_t RIGHT_LPWM = 13;
const uint8_t RIGHT_EN   = 33;  // drives both R_EN and L_EN, tied together

// Quadrature encoder pins -- front wheels only, on the ESP32's input-only
// ADC1 pins (GPIO34/35/36/39). Any GPIO works as an interrupt source on
// ESP32, so both channels of both encoders get their own interrupt for
// full 4x decoding.
const uint8_t LEFT_ENC_CHA  = 34;
const uint8_t LEFT_ENC_CHB  = 35;
const uint8_t RIGHT_ENC_CHA = 36;  // silkscreen label "VP" on most boards
const uint8_t RIGHT_ENC_CHB = 39;  // silkscreen label "VN" on most boards

// ----------------------------- PWM config -----------------------------
// Arduino-ESP32 core 2.x's LEDC API is channel-based (ledcSetup/
// ledcAttachPin/ledcWrite(channel,...)), unlike core 3.x's simpler
// pin-based ledcAttach()/ledcWrite(pin,...). This targets 2.x, since
// that's what a fresh `arduino-cli core install esp32:esp32` currently
// gives you; if you later upgrade to core 3.x, ledcAttach(pin, freq, res)
// + ledcWrite(pin, duty) replaces the channel plumbing below.
const int LEFT_RPWM_CH  = 0;
const int LEFT_LPWM_CH  = 1;
const int RIGHT_RPWM_CH = 2;
const int RIGHT_LPWM_CH = 3;

const int PWM_FREQ_HZ   = 20000;  // above audible range
const int PWM_RES_BITS  = 10;     // 0-1023
const int PWM_RES_MAX   = 1023;
const int CMD_RANGE_MAX = 255;    // unchanged wire-protocol range from the Jetson

// ----------------------------- Timing ---------------------------------
const unsigned long ENC_PUBLISH_INTERVAL_MS = 50;   // 20 Hz encoder reports
const unsigned long CMD_TIMEOUT_MS          = 500;  // stop motors if Jetson goes quiet

// ----------------------------- State -----------------------------------
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// 2-bit (chA<<1 | chB) quadrature state per encoder, for the decode table.
volatile uint8_t leftEncState  = 0;
volatile uint8_t rightEncState = 0;

int targetLeftPWM  = 0;
int targetRightPWM = 0;

unsigned long lastCmdMillis = 0;
unsigned long lastEncPublishMillis = 0;

String rxLine;

// ----------------------------- Quadrature decode -----------------------------
// Standard 4x quadrature decode lookup: index = (prevState<<2)|newState,
// each state = (A<<1)|B. Returns +1, -1, or 0 (no valid transition, e.g.
// a bounce or a missed edge).
static const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

inline void quadDecode(volatile long &ticks, volatile uint8_t &state, uint8_t chA, uint8_t chB) {
  uint8_t newState = (uint8_t)((digitalRead(chA) << 1) | digitalRead(chB));
  ticks += QUAD_TABLE[(state << 2) | newState];
  state = newState;
}

void IRAM_ATTR leftEncoderISR() {
  quadDecode(leftTicks, leftEncState, LEFT_ENC_CHA, LEFT_ENC_CHB);
}

void IRAM_ATTR rightEncoderISR() {
  quadDecode(rightTicks, rightEncState, RIGHT_ENC_CHA, RIGHT_ENC_CHB);
}

// ----------------------------- Motor control -----------------------------
// BTS7960: drive RPWM for forward, LPWM for reverse, the other held at 0.
// Never drive both PWM channels non-zero at once (shoot-through risk).
void setMotorSide(int lpwmChannel, int rpwmChannel, int pwmVal) {
  pwmVal = constrain(pwmVal, -CMD_RANGE_MAX, CMD_RANGE_MAX);
  // Scale the unchanged [-255,255] wire value up to the 10-bit LEDC range.
  int duty = (abs(pwmVal) * PWM_RES_MAX) / CMD_RANGE_MAX;

  if (pwmVal > 0) {
    ledcWrite(rpwmChannel, duty);
    ledcWrite(lpwmChannel, 0);
  } else if (pwmVal < 0) {
    ledcWrite(rpwmChannel, 0);
    ledcWrite(lpwmChannel, duty);
  } else {
    ledcWrite(rpwmChannel, 0);
    ledcWrite(lpwmChannel, 0);
  }
}

void applyMotorTargets() {
  setMotorSide(LEFT_LPWM_CH, LEFT_RPWM_CH, targetLeftPWM);
  setMotorSide(RIGHT_LPWM_CH, RIGHT_RPWM_CH, targetRightPWM);
}

void stopMotors() {
  targetLeftPWM = 0;
  targetRightPWM = 0;
  applyMotorTargets();
}

// ----------------------------- Serial handling -----------------------------
void handleLine(const String &line) {
  // Expected: "M,<left>,<right>"
  if (line.length() < 3 || line.charAt(0) != 'M' || line.charAt(1) != ',') {
    return;
  }

  int firstComma = 1;
  int secondComma = line.indexOf(',', firstComma + 1);
  if (secondComma < 0) {
    return;
  }

  String leftStr = line.substring(firstComma + 1, secondComma);
  String rightStr = line.substring(secondComma + 1);

  targetLeftPWM = constrain(leftStr.toInt(), -CMD_RANGE_MAX, CMD_RANGE_MAX);
  targetRightPWM = constrain(rightStr.toInt(), -CMD_RANGE_MAX, CMD_RANGE_MAX);
  lastCmdMillis = millis();
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      rxLine.trim();
      if (rxLine.length() > 0) {
        handleLine(rxLine);
      }
      rxLine = "";
    } else if (c != '\r') {
      rxLine += c;
      if (rxLine.length() > 32) {
        rxLine = "";  // guard against garbage/overflow
      }
    }
  }
}

void publishEncoders() {
  long l, r;
  noInterrupts();
  l = leftTicks;
  r = rightTicks;
  interrupts();

  Serial.print("E,");
  Serial.print(l);
  Serial.print(",");
  Serial.println(r);
}

// ----------------------------- Setup / loop -----------------------------
void setup() {
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);

  ledcSetup(LEFT_RPWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(LEFT_RPWM, LEFT_RPWM_CH);
  ledcSetup(LEFT_LPWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(LEFT_LPWM, LEFT_LPWM_CH);
  ledcSetup(RIGHT_RPWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(RIGHT_RPWM, RIGHT_RPWM_CH);
  ledcSetup(RIGHT_LPWM_CH, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(RIGHT_LPWM, RIGHT_LPWM_CH);

  // Enable both half-bridges on both driver modules; speed/direction is
  // controlled purely via LPWM/RPWM from here on.
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);

  // INPUT_PULLUP is a no-op on GPIO34/35/36/39 (see file header) -- kept
  // here so the intent is documented and this still behaves correctly on
  // any encoder pins you move to a normal GPIO.
  pinMode(LEFT_ENC_CHA, INPUT_PULLUP);
  pinMode(LEFT_ENC_CHB, INPUT_PULLUP);
  pinMode(RIGHT_ENC_CHA, INPUT_PULLUP);
  pinMode(RIGHT_ENC_CHB, INPUT_PULLUP);

  leftEncState  = (uint8_t)((digitalRead(LEFT_ENC_CHA) << 1) | digitalRead(LEFT_ENC_CHB));
  rightEncState = (uint8_t)((digitalRead(RIGHT_ENC_CHA) << 1) | digitalRead(RIGHT_ENC_CHB));

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_CHA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_CHB), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_CHA), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_CHB), rightEncoderISR, CHANGE);

  stopMotors();

  Serial.begin(57600);
  rxLine.reserve(32);

  lastCmdMillis = millis();
  lastEncPublishMillis = millis();
}

void loop() {
  pollSerial();

  unsigned long now = millis();

  if (now - lastCmdMillis > CMD_TIMEOUT_MS) {
    targetLeftPWM = 0;
    targetRightPWM = 0;
  }
  applyMotorTargets();

  if (now - lastEncPublishMillis >= ENC_PUBLISH_INTERVAL_MS) {
    lastEncPublishMillis = now;
    publishEncoders();
  }
}
