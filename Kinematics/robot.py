# This is not yet working...
import wpilib
import math
from wpilib import kinematics
from wpilib.geometry import Rotation2d
from wpilib.geometry import Translation2d
from wpilib.kinematics import ChassisSpeeds
from wpilib.kinematics import MecanumDriveKinematics
from wpilib.kinematics import MecanumDriveOdometry
from wpilib.kinematics import MecanumDriveWheelSpeeds
from wpilib.drive import MecanumDrive

from networktables import NetworkTables
import rev

class MyRobot(wpilib.TimedRobot):
    # Channels on the roboRIO that the motor controllers are plugged in to
    frontLeftChannel = 2
    rearLeftChannel = 3
    frontRightChannel = 1
    rearRightChannel = 0
    def robotInit(self):
        # Pull in smart dashboard info...
        self.sd = NetworkTables.getTable("SmartDashboard")

        # Start a timer....
        self.timer = wpilib.Timer()

        """Robot initialization function.  The Low level is to use the brushless function on the controllers."""
        if wpilib.RobotBase.isSimulation():
            self.frontLeftMotor = wpilib.Spark(self.frontLeftChannel)
            self.rearLeftMotor = wpilib.Spark(self.rearLeftChannel)
            self.frontRightMotor = wpilib.Spark(self.frontRightChannel)
            self.rearRightMotor = wpilib.Spark(self.rearRightChannel)

        else:    
            self.frontLeftMotor = rev.CANSparkMax(self.frontLeftChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
            self.rearLeftMotor = rev.CANSparkMax(self.rearLeftChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
            self.frontRightMotor = rev.CANSparkMax(self.frontRightChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
            self.rearRightMotor = rev.CANSparkMax(self.rearRightChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        # The channel on the driver station that the joystick is connected to
        joystickChannel = 0

        m_frontLeftLocation = Translation2d(0.381, 0.381)
        m_frontRightLocation = Translation2d(0.381, -0.381)
        m_backLeftLocation = Translation2d(-0.381, 0.381)
        m_backRightLocation = Translation2d(-0.381, -0.381)

        # Creating my kinematics object using the wheel locations.
        self.m_kinematics = MecanumDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation)

        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )


        # Example chassis speeds: 1 meter per second forward, 3 meters
        # per second to the left, and rotation at 1.5 radians per second
        # counterclockwise.
        #speeds = ChassisSpeeds(1.0, 3.0, 1.5)

        # Convert to wheel speeds
        #wheelSpeeds = MecanumDriveKinematics.toWheelSpeeds(speeds)

        # Get the individual wheel speeds
        #frontLeft = wheelSpeeds.frontLeftMetersPerSecond
        #frontRight = wheelSpeeds.frontRightMetersPerSecond
        #backLeft = wheelSpeeds.rearLeftMetersPerSecond
        #backRight = wheelSpeeds.rearRightMetersPerSecond

        # Field Oriented Drive
        # The desired field relative speed here is 2 meters per second
        # toward the opponent's alliance station wall, and 2 meters per
        # second toward the left field boundary. The desired rotation
        # is a quarter of a rotation per second counterclockwise. The current
        # robot angle is 45 degrees.
        #FieldOrientedspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        #2.0, 2.0, (math.pi # 2.0), Rotation2d.fromDegrees(45.0))

        # Now use this in our kinematics
        #wheelSpeeds = MecanumDriveKinematics.toWheelSpeeds(FieldOrientedspeeds)

        # Example wheel speeds
        # wheelSpeeds = MecanumDriveWheelSpeeds(-17.67, 20.51, -13.44, 16.26)

        # Convert to chassis speeds
        # chassisSpeeds = MecanumDriveKinematics.toChassisSpeeds(wheelSpeeds)

        # Getting individual speeds
    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
        # Example chassis speeds: 1 meter per second forward, 3 meters
        ## per second to the left, and rotation at 1.5 radians per second
        ## counterclockwise.
        # speeds = ChassisSpeeds(self.forward,self.sideways,self.angular)
        #wheelSpeeds = self.m_kinematics.toWheelSpeeds(speeds)
        self.drive.driveCartesian(
                .5,0,0,0
            )
        chassisspeeds = self.m_kinematics.toChassisSpeeds(.5,.5,0,0)
        print(chassisspeeds)
        
    
    
    

    

    





if __name__ == "__main__":
    wpilib.run(MyRobot)