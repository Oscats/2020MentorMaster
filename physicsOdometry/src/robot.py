#!/usr/bin/env python3
import math
import wpilib

from wpilib.drive import MecanumDrive
from networktables import NetworkTables
from wpilib.controller import SimpleMotorFeedforwardMeters
from wpilib.geometry import Pose2d, Rotation2d, Translation2d
from wpilib.kinematics import (
    ChassisSpeeds,
    MecanumDriveKinematics,
    MecanumDriveOdometry,
    MecanumDriveWheelSpeeds,
)

GEAR_RATIO = 10.75
# measurements in metres
TRACK_WIDTH = 0.579  # measured by characterisation
WHEEL_CIRCUMFERENCE = 0.0254 * 6 * math.pi
XOffset = 0.288
YOffset = 0.257


class MyRobot(wpilib.TimedRobot):
    """Main robot class"""

    # Channels on the roboRIO that the motor controllers are plugged in to
    frontLeftChannel = 1
    rearLeftChannel = 2
    frontRightChannel = 3
    rearRightChannel = 4

    # The channel on the driver station that the joystick is connected to
    lStickChannel = 0
    rStickChannel = 1
    WHEEL_DIAMETER = 0.5  # 6 inches
    ENCODER_COUNTS_PER_REV = 360

    def robotInit(self):
        """Robot initialization function"""
        self.frontLeftMotor = wpilib.Talon(self.frontLeftChannel)
        self.rearLeftMotor = wpilib.Talon(self.rearLeftChannel)
        self.frontRightMotor = wpilib.Talon(self.frontRightChannel)
        self.rearRightMotor = wpilib.Talon(self.rearRightChannel)

        self.lstick = wpilib.Joystick(self.lStickChannel)
        self.rstick = wpilib.Joystick(self.rStickChannel)

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)

        self.sd = NetworkTables.getTable("SmartDashboard")
        # Setting up Kinematics (an algorithm to determine chassi speed from wheel speed)
        # The 2d Translation tells the algorithm how far off center (in Meter) our wheels are
        # Ours are about 11.3 (x) by 10.11(y) inches off, so this equates to roughly .288 X .257 Meters
        # We use the X and Y Offsets above.

        m_frontLeftLocation = Translation2d(XOffset, YOffset)
        m_frontRightLocation = Translation2d(XOffset, (-1 * YOffset))
        m_backLeftLocation = Translation2d((-1 * XOffset), (YOffset))
        m_backRightLocation = Translation2d((-1 * XOffset), (-1 * YOffset))

        # Creat our kinematics object using the wheel locations.
        self.m_kinematics = MecanumDriveKinematics(
            m_frontLeftLocation,
            m_frontRightLocation,
            m_backLeftLocation,
            m_backRightLocation,
        )
        # Create the Odometry Object
        self.MecanumDriveOdometry = MecanumDriveOdometry(
            self.m_kinematics,
            Rotation2d.fromDegrees(-self.gyro.getAngle()),
            Pose2d(0, 0, Rotation2d(0)),
        )

        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )
        self.f_l_encoder = wpilib.Encoder(0, 1)
        self.f_l_encoder.setDistancePerPulse(
            (math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV
        )

        self.r_l_encoder = wpilib.Encoder(3, 4)
        self.r_l_encoder.setDistancePerPulse(
            (math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV
        )

        self.f_r_encoder = wpilib.Encoder(1, 2)
        self.f_r_encoder.setDistancePerPulse(
            (math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV
        )

        self.r_r_encoder = wpilib.Encoder(3, 4)
        self.r_r_encoder.setDistancePerPulse(
            (math.pi * self.WHEEL_DIAMETER) / self.ENCODER_COUNTS_PER_REV
        )

    def robotPeriodic(self):
        # Odometry Update
        # First, get the wheel speeds...
        self.wheelSpeeds = MecanumDriveWheelSpeeds(
            self.f_l_encoder.getRate(),
            self.r_l_encoder.getRate(),
            self.f_r_encoder.getRate(),
            self.r_r_encoder.getRate(),
        )
        # Next, we need to grab the Gyro angle and send it into the odometry.  It must be inverted because gyros v Wpilib are backwards
        gyroAngle = Rotation2d.fromDegrees(-self.gyro.getAngle())
        # Finally, we can update the pose...
        self.m_pose = self.MecanumDriveOdometry.update(gyroAngle, self.wheelSpeeds)

    def disabled(self):
        """Called when the robot is disabled"""
        while self.isDisabled():
            wpilib.Timer.delay(0.01)

    def autonomousInit(self):
        """Called when autonomous mode is enabled"""
        self.timer = wpilib.Timer()
        self.timer.start()

    def autonomousPeriodic(self):
        if self.timer.get() < 2.0:
            self.drive.driveCartesian(0, 1, 0, 0)
        else:
            self.drive.driveCartesian(0, 0, 1, 0)

        print(float(self.f_l_encoder.getRate()))

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""

        # self.drive.driveCartesian(
        #     self.lstick.getX(), -self.lstick.getY(), self.rstick.getX(), 0
        # )

        self.drive.driveCartesian(
            self.lstick.getX(), -self.lstick.getY(), self.lstick.getRawAxis(2), 0
        )


if __name__ == "__main__":
    wpilib.run(MyRobot)
