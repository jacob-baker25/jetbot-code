#!/usr/bin/env python3

import sys
import time
import select
import termios
import tty

from jetbot import Robot


# --------------------------------------------------
# Motor configuration
# Matches the existing JetBot project calibration.
# --------------------------------------------------

MOTOR_DIR_L = -1.0
MOTOR_DIR_R = -1.0

BIAS_L = 1.0
BIAS_R = 1.0

DRIVE_SPEED = 0.25
TURN_SPEED = 0.18

# If no movement command is received for this long,
# automatically stop the robot.
WATCHDOG_SECONDS = 0.15


def clamp(value, lo=-1.0, hi=1.0):
    return max(lo, min(hi, value))


class KeyboardTeleop:

    def __init__(self):
        self.robot = Robot()

        self.speed = DRIVE_SPEED
        self.turn_speed = TURN_SPEED

        self.last_command_time = 0.0
        self.moving = False

    def drive(self, left, right):
        left_cmd = clamp(left * MOTOR_DIR_L * BIAS_L)
        right_cmd = clamp(right * MOTOR_DIR_R * BIAS_R)

        self.robot.set_motors(left_cmd, right_cmd)

    def stop(self):
        self.drive(0.0, 0.0)
        self.moving = False

    def forward(self):
        self.drive(self.speed, self.speed)
        self.moving = True

    def backward(self):
        self.drive(-self.speed, -self.speed)
        self.moving = True

    def left(self):
        self.drive(self.turn_speed, -self.turn_speed)
        self.moving = True

    def right(self):
        self.drive(-self.turn_speed, self.turn_speed)
        self.moving = True

    def refresh_watchdog(self):
        self.last_command_time = time.monotonic()

    def watchdog_expired(self):
        return (
            self.moving
            and
            time.monotonic() - self.last_command_time > WATCHDOG_SECONDS
        )


def print_controls(speed):
    print()
    print("JetBot Keyboard Teleop")
    print("----------------------")
    print("Hold:")
    print("  W = forward")
    print("  S = backward")
    print("  A = turn left")
    print("  D = turn right")
    print()
    print("  SPACE = immediate stop")
    print("  +     = increase speed")
    print("  -     = decrease speed")
    print("  Q     = stop and quit")
    print()
    print("Watchdog automatically stops the motors if commands stop.")
    print("Current speed: {:.2f}".format(speed))
    print()


def main():

    teleop = KeyboardTeleop()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    print_controls(teleop.speed)

    try:
        tty.setcbreak(fd)

        running = True

        while running:

            readable, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                0.05
            )

            if readable:
                key = sys.stdin.read(1).lower()

                if key == 'w':
                    teleop.forward()
                    teleop.refresh_watchdog()

                elif key == 's':
                    teleop.backward()
                    teleop.refresh_watchdog()

                elif key == 'a':
                    teleop.left()
                    teleop.refresh_watchdog()

                elif key == 'd':
                    teleop.right()
                    teleop.refresh_watchdog()

                elif key == ' ':
                    teleop.stop()
                    print("\rSTOP                    ", end='', flush=True)

                elif key in ['+', '=']:
                    teleop.speed = min(0.60, teleop.speed + 0.05)
                    print(
                        "\rSpeed: {:.2f}              ".format(teleop.speed),
                        end='',
                        flush=True
                    )

                elif key == '-':
                    teleop.speed = max(0.15, teleop.speed - 0.05)
                    print(
                        "\rSpeed: {:.2f}              ".format(teleop.speed),
                        end='',
                        flush=True
                    )

                elif key == 'q':
                    running = False

            if teleop.watchdog_expired():
                teleop.stop()

    except KeyboardInterrupt:
        pass

    finally:
        teleop.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nJetBot stopped.")


if __name__ == '__main__':
    main()
