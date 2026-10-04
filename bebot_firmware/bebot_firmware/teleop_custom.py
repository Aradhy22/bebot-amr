#!/usr/bin/env python3
"""Keyboard teleop pre-tuned to this robot's comfortable driving speed.

Run with `ros2 run bebot_firmware teleop_custom` -- NOT `ros2 launch`.
teleop_twist_keyboard needs a real tty for termios-based raw key reading,
which `ros2 launch` doesn't provide to its child processes (it fails with
"termios.error: (25, 'Inappropriate ioctl for device')"). `ros2 run` execs
directly and inherits the calling terminal, so it works.

Starts at speed=0.11 m/s, turn=0.60 rad/s (found by hand to be the right
pace for this chassis -- default teleop speeds of 0.5/1.0 were too fast).
The usual q/z, w/x, e/c hotkeys still work at runtime to nudge speed up or
down from this starting point.
"""

import os


def main():
    os.execvp('ros2', [
        'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
        '--ros-args', '-p', 'speed:=0.11', '-p', 'turn:=0.60',
    ])


if __name__ == '__main__':
    main()
