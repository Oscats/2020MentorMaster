#!/usr/bin/env python3
"""
    This is a demo program showing how to use Mecanum control with the
    Navx.
"""

import wpilib
from wpilib.drive import MecanumDrive
from networktables import NetworkTables

# Import Rev Hardware for Can
import rev
# Import NavX
import navx


def run():
    raise ValueError()


class MyRobot(wpilib.TimedRobot):
    # Channels on the roboRIO that the motor controllers are plugged in to
    frontLeftChannel = 2
    rearLeftChannel = 3
    frontRightChannel = 1
    rearRightChannel = 0

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):

        """Robot initialization function.  The Low level is to use the brushless function on the controllers."""
        self.sd = NetworkTables.getTable("SmartDashboard")

        self.timer = wpilib.Timer()

        #
        # Communicate w/navX MXP via the MXP SPI Bus.
        # - Alternatively, use the i2c bus.
        # See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details
        #

        self.navx = navx.AHRS.create_spi()
        # self.navx = navx.AHRS.create_i2c()

        if wpilib.RobotBase.isSimulation():
            self.frontLeftMotor = wpilib.Jaguar(self.frontLeftChannel)
            self.rearLeftMotor = wpilib.Jaguar(self.rearLeftChannel)
            self.frontRightMotor = wpilib.Jaguar(self.frontRightChannel)
            self.rearRightMotor = wpilib.Jaguar(self.rearRightChannel)

        else:
            self.frontLeftMotor = rev.CANSparkMax(
                self.frontLeftChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless
            )
            self.rearLeftMotor = rev.CANSparkMax(
                self.rearLeftChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless
            )
            self.frontRightMotor = rev.CANSparkMax(
                self.frontRightChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless
            )
            self.rearRightMotor = rev.CANSparkMax(
                self.rearRightChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless
            )

        # invert the left side motors
        self.rearRightMotor.setInverted(False)

        # you may need to change or remove this to match your robot
        self.rearLeftMotor.setInverted(False)

        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.frontRightMotor,
            self.rearLeftMotor,
            self.rearRightMotor,
        )

        self.drive.setExpiration(0.1)

        self.stick = wpilib.XboxController(self.joystickChannel)

    def robotPeriodic(self):

        self.logger.info("Entered disabled mode")

        self.timer.reset()
        self.timer.start()

        if self.timer.hasPeriodPassed(0.5):
            self.sd.putNumber("Displacement X", self.navx.getDisplacementX())
            self.sd.putNumber("Displacement Y", self.navx.getDisplacementY())
            self.sd.putBoolean("IsCalibrating", self.navx.isCalibrating())
            self.sd.putBoolean("IsConnected", self.navx.isConnected())
            self.sd.putNumber("Angle", self.navx.getAngle())
            self.sd.putNumber("Pitch", self.navx.getPitch())
            self.sd.putNumber("Yaw", self.navx.getYaw())
            self.sd.putNumber("Roll", self.navx.getRoll())

    def autonomousInit(self):
        """Runs Once during auto"""
        # self.counter = 0

    def autonomousPeriodic(self):
        """Runs Periodically during auto"""

        self.sd.putNumber("Timestamp", self.navx.getLastSensorTimestamp())
        self.drive.driveCartesian(0, 0.5, 0, 0)


if __name__ == "__main__":
    wpilib.run(MyRobot)
