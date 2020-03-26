#!/usr/bin/env python3
import math
import wpilib


from wpilib.drive import MecanumDrive
from networktables import NetworkTables
from wpilib.controller import PIDController
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
TRACK_WIDTH = 0.579  # measured by characterization
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
    ENCODER_COUNTS_PER_REV = 3

    def robotInit(self):
        """Robot initialization function"""
        self.frontLeftMotor = wpilib.Talon(self.frontLeftChannel)
        self.rearLeftMotor = wpilib.Talon(self.rearLeftChannel)
        self.frontRightMotor = wpilib.Talon(self.frontRightChannel)
        self.rearRightMotor = wpilib.Talon(self.rearRightChannel)

        self.lstick = wpilib.Joystick(self.lStickChannel)
        self.rstick = wpilib.Joystick(self.rStickChannel)
        self.sd = NetworkTables.getTable("SmartDashboard")

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)

        #Create the DriveTrain
        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )
        #Create The Encoders
        self.f_l_encoder = wpilib.Encoder(0, 1)
        self.f_l_encoder.setDistancePerPulse(
            (.0254* 6 * math.pi) / 1024
        )
        #self.f_l_encoder.setSimDevice(0)

        self.r_l_encoder = wpilib.Encoder(3, 4)
        self.r_l_encoder.setDistancePerPulse(
            (.0254* 6 * math.pi) / 1024
        )
        #self.r_l_encoder.setSimDevice(1)

        self.f_r_encoder = wpilib.Encoder(1, 2)
        self.f_r_encoder.setDistancePerPulse(
            (.0254* 6 * math.pi) / 1024
        )
        #self.f_r_encoder.setSimDevice(2)

        self.r_r_encoder = wpilib.Encoder(3, 4)
        self.r_r_encoder.setDistancePerPulse(
            (.0254* 6 * math.pi) / 1024
        )
        #self.r_r_encoder.setSimDevice(3)

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

        # Now that we have created the ability to see wheel speeds, chassis speeds and our position
        # Let us start to use feedforward to try to lock our robot into a specific speed.
        self.feedForward = SimpleMotorFeedforwardMeters(kS=0.194, kV=.5, kA=0.457)

    #def robotPeriodic(self):
        # Odometry Update
        # First, get the wheel speeds...
        
        # Next, we need to grab the Gyro angle and send it into the odometry.  It must be inverted because gyros v Wpilib are backwards
        



    def disabled(self):
        """Called when the robot is disabled"""
        while self.isDisabled():
            wpilib.Timer.delay(0.01)

    def autonomousInit(self):
        """Called when autonomous mode is enabled"""
        self.timer = wpilib.Timer()
        self.timer.start()
        self.f_l_encoder.reset()
        self.r_l_encoder.reset()
        self.f_r_encoder.reset()
        self.r_r_encoder.reset()
        self.lastCount = 0


    def autonomousPeriodic(self):
        preChassis = ChassisSpeeds()
        preChassis.vx = 5.0
        preChassis.vy = 0.0
        preChassis.omega = 0.0
        # Convert to wheel speeds
        speeds = self.m_kinematics.toWheelSpeeds(preChassis)
        self.wheelSpeeds = MecanumDriveWheelSpeeds(
            self.f_l_encoder.getRate(),
            self.r_l_encoder.getRate(),
            self.f_r_encoder.getRate(),
            self.r_r_encoder.getRate(),
        )
        gyroAngle = Rotation2d.fromDegrees(-self.gyro.getAngle())
        # Finally, we can update the pose...
        self.m_pose = self.MecanumDriveOdometry.update(gyroAngle, self.wheelSpeeds)
        #For Kinematics, we need to update the wheelspeeds
        CurrentChassis = self.m_kinematics.toChassisSpeeds(self.wheelSpeeds)
        print(CurrentChassis)
        print(self.f_l_encoder.getDistancePerPulse())
        print('difference')
        print(self.f_l_encoder.get()-self.lastCount)
        print('rate')
        print(self.r_r_encoder.getRate())
        print('lastCount')
        self.lastCount = self.f_l_encoder.get()
        print(self.lastCount)
        

        
        

        '''
        
        left_front = self.feedForward.calculate(speeds.frontLeft)s
        right_front = self.feedForward.calculate(speeds.frontRight)
        left_rear = self.feedForward.calculate(speeds.rearLeft)
        right_rear = self.feedForward.calculate(speeds.rearRight)'''

        #print(left_front, right_front, left_rear,right_rear)
    


        if self.timer.get() < 2.0:
            # self.drive.driveCartesian(1, 1, 1, 1) #<---- This is using the drive method built into the mecanum dirve.
            # Maybe we want to use something with more control, like feedforward...
            '''self.frontLeftMotor.set(-left_front)
            self.rearLeftMotor.set(right_front)
            self.frontRightMotor.set(-left_rear)
            self.rearRightMotor.set(right_rear)'''
            self.drive.driveCartesian(1, 0, 0, 0)
        elif self.timer.get()>2 and self.timer.get()<4:
            self.drive.driveCartesian(1, 0, 0, 0)
        else:
            self.drive.driveCartesian(0, 0, 0, 0)

       

        

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
