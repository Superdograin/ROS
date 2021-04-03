# Simple four DC motor car class.  Exposes a simple LOGO turtle-like API for
# moving a car forward, backward, turning, pan, speed up,and speed down.
# See CarTest.py for an
# example of using this class.
# Author: zi vu
# License: MIT License https://opensource.org/licenses/MIT
import time
import atexit

from Adafruit_MotorHAT import Adafruit_MotorHAT


class Car(object):
    def __init__(self, address=0x60, motor_id=None, trim=None, speed=None, stop_at_exit=True):
        """Create an instance of the car.  Can specify the following optional
        parameters:
         - address: The I2C address of the motor HAT, default is 0x60.
         - motor_id: The ID of the motors at the left-front,left-rear,right-front,
                      right-rear, default is 1,2,3,4.
         - trim: Amount to offset the speed of the motors, can be positive
                      or negative and use useful for matching the speed of both
                      motors.  Default is 0.
         - speed: The initial speed of the motors, default is 0.
         - stop_at_exit: Boolean to indicate if the motors should stop on program
                         exit.  Default is True (highly recommended to keep this
                         value to prevent damage to the bot on program crash!).
        """
        # Initialize motor HAT and motors.
        if motor_id is None:
            motor_id = [1, 2, 3, 4]
        if trim is None:
            trim = [0, 0, 0, 0]
        if speed is None:
            speed = [0, 0, 0, 0]
        self._mh = Adafruit_MotorHAT(address)
        self._motor_id = motor_id
        self._trim = trim
        self._speed = speed
        self._left_front = self._mh.getMotor(self._motor_id[0])
        self._left_rear = self._mh.getMotor(self._motor_id[1])
        self._right_front = self._mh.getMotor(self._motor_id[2])
        self._right_rear = self._mh.getMotor(self._motor_id[3])
        # Start with motors turned off.
        self.stop()
        # Configure all motors to stop at program exit if desired.
        if stop_at_exit:
            atexit.register(self.stop)

    def set_speed(self, speed=None):
        """Set the speed of the motors, taking into account its trim offset."""
        if speed is None:
            speed = [0, 0, 0, 0]
        for i in range(4):
            if speed[i] is None:
                continue
            assert 0 <= speed[i] <= 255, 'Speed must be a value between 0 to 255 inclusive!'
            speed[i] += self._trim[i]
            speed[i] = max(0, min(255, speed[i]))  # Constrain speed to 0-255 after trimming.
            self._speed[i] = speed[i]
        self._left_front.setSpeed(self._speed[0])
        self._left_rear.setSpeed(self._speed[1])
        self._right_front.setSpeed(self._speed[2])
        self._right_rear.setSpeed(self._speed[3])

    def stop(self):
        """Stop all movement."""
        self._left_front.run(Adafruit_MotorHAT.RELEASE)
        self._left_rear.run(Adafruit_MotorHAT.RELEASE)
        self._right_front.run(Adafruit_MotorHAT.RELEASE)
        self._right_rear.run(Adafruit_MotorHAT.RELEASE)

    @staticmethod
    def set_orientation(orientation_code):
        """Set four motor orientation 0:FORWARD 1:BACKWARD 2:RELEASE."""
        orientation = []
        for code in orientation_code:
            if code == 0:
                orientation.append(Adafruit_MotorHAT.FORWARD)
            elif code == 1:
                orientation.append(Adafruit_MotorHAT.BACKWARD)
            else:
                orientation.append(Adafruit_MotorHAT.RELEASE)
        return orientation

    def run(self, orientation):
        """Run the motor."""
        self._left_front.run(orientation[0])
        self._left_rear.run(orientation[1])
        self._right_front.run(orientation[2])
        self._right_rear.run(orientation[3])

    def duration(self, seconds=None):
        """Set the duration."""
        # If an amount of time is specified, move for that time and then stop.
        if seconds is not None:
            time.sleep(seconds)
            self.stop()

    def set_orientation_and_run(self, orientation_code, seconds=None):
        """Set motor speed and run."""
        orientation = self.set_orientation(orientation_code)
        self.run(orientation)
        self.duration(seconds)

    def forward(self, speed, seconds=None):
        """Move forward at the specified speed (0-255).  Will start moving
    forward and return unless a seconds value is specified, in which
    case the robot will move forward for that amount of time and then stop.
    """
        self.set_speed([speed, speed, speed, speed])
        self.set_orientation_and_run([0, 0, 0, 0], seconds)

    def backward(self, speed, seconds=None):
        self.set_speed([speed, speed, speed, speed])
        self.set_orientation_and_run([1, 1, 1, 1], seconds)

    def right_mode_1(self, speed, seconds=None):
        """Spin to the right at the specified speed.  Will start spinning and
    return unless a seconds value is specified, in which case the robot will
    spin for that amount of time and then stop.
    """
        self.set_speed([speed, speed, 0, 0])
        self.set_orientation_and_run([0, 0, 2, 2], seconds)

    def right_mode_2(self, speed, seconds=None):
        self.set_speed([speed, 0, speed, 0])
        self.set_orientation_and_run([0, 2, 1, 2], seconds)

    def left_mode_1(self, speed, seconds=None):
        """Spin to the left at the specified speed.  Will start spinning and
    return unless a seconds value is specified, in which case the robot will
    spin for that amount of time and then stop.
    """
        self.set_speed([0, 0, speed, speed])
        self.set_orientation_and_run([2, 2, 0, 0], seconds)

    def left_mode_2(self, speed, seconds=None):
        self.set_speed([speed, 0, speed, 0])
        self.set_orientation_and_run([1, 2, 0, 2], seconds)

    def pan_left(self, speed, seconds=None):
        """向左平移"""
        self.set_speed([speed, speed, speed, speed])
        self.set_orientation_and_run([1, 0, 0, 1], seconds)

    def pan_right(self, speed, seconds=None):
        """向右平移"""
        self.set_speed([speed, speed, speed, speed])
        self.set_orientation_and_run([0, 1, 1, 0], seconds)

    def pinwheel_clockwise(self, speed, seconds=None):
        """绕中心旋转, 顺时针方向"""
        self.set_speed([speed, speed, speed, speed])
        self.set_orientation_and_run([0, 0, 1, 1], seconds)

    def pinwheel_counterclockwise(self, speed, seconds=None):
        """绕中心旋转， 逆时针方向"""
        self.set_speed([speed, speed, speed, speed])
        self.set_orientation_and_run([1, 1, 0, 0], seconds)

    def left_forward(self, speed, seconds=None):
        """左前方行进"""
        self.set_speed([0, speed, speed, 0])
        self.set_orientation_and_run([2, 0, 0, 2], seconds)

    def right_forward(self, speed, seconds=None):
        """右前方行进"""
        self.set_speed([speed, 0, 0, speed])
        self.set_orientation_and_run([0, 2, 2, 0], seconds)

    def left_backward(self, speed, seconds=None):
        """左后方行进"""
        self.set_speed([0, speed, speed, 0])
        self.set_orientation_and_run([1, 2, 2, 1], seconds)

    def right_backward(self, speed, seconds=None):
        """右后方行进"""
        self.set_speed([speed, 0, 0, speed])
        self.set_orientation_and_run([2, 1, 1, 2], seconds)
