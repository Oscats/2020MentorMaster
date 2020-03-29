#!/usr/bin/env python3
import math

import wpilib
import wpilib.drive
import hal.simulation




class MyRobot(wpilib.TimedRobot):
    """Main robot class"""
    GEAR_RATIO = 10.71
    WHEEL_DIAMETER = 0.5 * math.pi
    ENCODER_COUNTS_PER_REV = 360

    def robotInit(self):
        """Robot-wide initialization code should go here"""

        self.lstick = wpilib.Joystick(0)
        self.rstick = wpilib.Joystick(1)

        self.l_motor = wpilib.Jaguar(1)
        self.r_motor = wpilib.Jaguar(2)

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)

        self.drive = wpilib.drive.DifferentialDrive(self.l_motor, self.r_motor)

        self.motor = wpilib.Jaguar(4)

        self.limit1 = wpilib.DigitalInput(1)
        self.limit2 = wpilib.DigitalInput(2)

        self.position = wpilib.AnalogInput(2)

        self.leftEncoder = wpilib.Encoder(0, 1)
        self.leftEncoder.setDistancePerPulse(
            ( self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV
        )
        self.leftEncoder.setSimDevice(0)

        self.rightEncoder = wpilib.Encoder(3, 4)
        self.rightEncoder.setDistancePerPulse(
            (self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV
        )

    def autonomousInit(self):
        """Called when autonomous mode is enabled"""

        self.timer = wpilib.Timer()
        self.timer.start()
        self.oldTime=0.0
        self.oldDistance=0.0

    def autonomousPeriodic(self):
        newTime = self.timer.get()
        newDistance = self.leftEncoder.getDistance()
        speed = ((newDistance-self.oldDistance)/(newTime-self.oldTime))
        print(speed)
        print("Rate: " +str(self.leftEncoder.getRate()))
        print("distance: " + str(self.leftEncoder.getDistance()))
        print("time: " +str(self.timer.get()))
        print("RightRate: " +str((self.rightEncoder.getRate()/100000)))
        if self.timer.get() < 2.0:
            self.drive.arcadeDrive(1.0, 0.0)
        else:
            self.drive.arcadeDrive(0, 0)
        self.oldDistance = newDistance
        self.oldTime=newTime

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""

        self.drive.arcadeDrive(-self.lstick.getY(), self.lstick.getX())

        # Move a motor with a Joystick
        y = self.rstick.getY()

        # stop movement backwards when 1 is on
        if self.limit1.get():
            y = max(0, y)

        # stop movement forwards when 2 is on
        if self.limit2.get():
            y = min(0, y)

        self.motor.set(y)


if __name__ == "__main__":
    wpilib.run(MyRobot)