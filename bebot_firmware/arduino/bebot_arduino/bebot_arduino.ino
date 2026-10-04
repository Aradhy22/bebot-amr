/*
 * bebot_arduino.ino
 *
 * Runs on an Arduino Uno wired to:
 *   - 4 DC motors in skid-steer formation (front-left, front-right,
 *     rear-left, rear-right), driven by two BTS7960 (IBT-2 style)
 *     dual half-bridge modules -- one per side. Each module exposes
 *     L_EN / R_EN (half-bridge enables, held HIGH) and LPWM / RPWM
 *     (PWM in the reverse / forward direction respectively). The two
 *     motors on a side are wired in parallel to the same module, so
 *     they always turn together.
 *   - 2 quadrature encoders, rear wheels only (rear-left, rear-right).
 *     Front wheels are not instrumented.
 *
 * Serial protocol (matches bebot_firmware/serial_odometry_node.py):
 *   Arduino -> Jetson : "E,<left_ticks>,<right_ticks>\n"   every ENC_PUBLISH_INTERVAL_MS
 *   Jetson  -> Arduino: "M,<left_pwm>,<right_pwm>\n"       pwm in [-255, 255]
 *
 * Safety: if no "M," command is received for CMD_TIMEOUT_MS, motors stop.
 */

#include <Arduino.h>

// ----------------------------- Pin map -------------------------------
// Motor driver pins (BTS7960 / IBT-2 dual half-bridge, one module per side).
// RPWM drives the module's "forward" half-bridge, LPWM the "reverse" one.
// If a side spins backwards for a positive command, swap that side's
// LPWM/RPWM constants below (or swap the motor leads) rather than
// touching the ROS-side sign convention.
const uint8_t LEFT_L_EN = 4;
const uint8_t LEFT_R_EN = 7;
const uint8_t LEFT_LPWM = 6;
const uint8_t LEFT_RPWM = 5;

const uint8_t RIGHT_L_EN = 9;
const uint8_t RIGHT_R_EN = 8;
const uint8_t RIGHT_LPWM = 11;
const uint8_t RIGHT_RPWM = 10;

// Quadrature encoder pins. A channels MUST be on the Uno's external
// interrupt pins (2 and 3). B channels are plain digital inputs read
// inside the ISR to determine direction.
const uint8_t LEFT_ENC_A  = 3;  // INT1
const uint8_t LEFT_ENC_B  = 13;
const uint8_t RIGHT_ENC_A = 2;  // INT0
const uint8_t RIGHT_ENC_B = 12;

// ----------------------------- Timing ---------------------------------
const unsigned long ENC_PUBLISH_INTERVAL_MS = 50;   // 20 Hz encoder reports
const unsigned long CMD_TIMEOUT_MS          = 500;  // stop motors if Jetson goes quiet

// ----------------------------- State -----------------------------------
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

int targetLeftPWM  = 0;
int targetRightPWM = 0;

unsigned long lastCmdMillis = 0;
unsigned long lastEncPublishMillis = 0;

String rxLine;

// ----------------------------- Encoder ISRs -----------------------------
void leftEncoderISR() {
  // Rising edge on A: use B to determine direction.
  if (digitalRead(LEFT_ENC_B) == LOW) {
    leftTicks++;
  } else {
    leftTicks--;
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == LOW) {
    rightTicks++;
  } else {
    rightTicks--;
  }
}

// ----------------------------- Motor control -----------------------------
// BTS7960: drive RPWM for forward, LPWM for reverse, the other held at 0.
// Never drive both PWM channels non-zero at once (shoot-through risk).
void setMotorSide(uint8_t lpwmPin, uint8_t rpwmPin, int pwmVal) {
  pwmVal = constrain(pwmVal, -255, 255);

  if (pwmVal > 0) {
    analogWrite(rpwmPin, pwmVal);
    analogWrite(lpwmPin, 0);
  } else if (pwmVal < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -pwmVal);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

void applyMotorTargets() {
  setMotorSide(LEFT_LPWM, LEFT_RPWM, targetLeftPWM);
  setMotorSide(RIGHT_LPWM, RIGHT_RPWM, targetRightPWM);
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

  targetLeftPWM = constrain(leftStr.toInt(), -255, 255);
  targetRightPWM = constrain(rightStr.toInt(), -255, 255);
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
  pinMode(LEFT_L_EN, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(RIGHT_L_EN, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);

  // Enable both half-bridges on both driver modules; speed/direction is
  // controlled purely via LPWM/RPWM from here on.
  digitalWrite(LEFT_L_EN, HIGH);
  digitalWrite(LEFT_R_EN, HIGH);
  digitalWrite(RIGHT_L_EN, HIGH);
  digitalWrite(RIGHT_R_EN, HIGH);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

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
