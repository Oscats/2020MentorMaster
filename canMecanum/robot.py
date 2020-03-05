#!/usr/bin/env python3
"""
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
"""

import wpilib
from wpilib.drive import MecanumDrive
# Import network tables
from networktables import NetworkTables
# Import Rev Hardware for Can
import rev
import ctre



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
        '''if wpilib.RobotBase.isSimulation():
            self.frontLeftMotor = ctre.WPI_VictorSPX(self.frontLeftChannel)
            self.rearLeftMotor = ctre.WPI_VictorSPX(self.rearLeftChannel)
            self.frontRightMotor = ctre.WPI_VictorSPX(self.frontRightChannel)
            self.rearRightMotor = ctre.WPI_VictorSPX(self.rearRightChannel)

        else: '''   
        self.frontLeftMotor = rev.CANSparkMax(self.frontLeftChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.rearLeftMotor = rev.CANSparkMax(self.rearLeftChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.frontRightMotor = rev.CANSparkMax(self.frontRightChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.rearRightMotor = rev.CANSparkMax(self.rearRightChannel, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

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
        self.lift = ctre.WPI_VictorSPX(6)
    
    def autonomousInit(self):
        """Runs Once during auto"""
        
    def autonomousPeriodic(self):
        """Runs Periodically during auto"""
        self.drive.driveCartesian(
            0.5, 0, 0, 0
        )

    def teleopInit(self):
        """Runs Once during teleop"""
        self.drive.setSafetyEnabled(True)
        

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        # This sample does not use field-oriented drive, so the gyro input is set to zero.
        self.drive.driveCartesian(
            self.stick.getRawAxis(2), self.stick.getRawAxis(3), self.stick.getRawAxis(0), 0
        )
        self.lift.set(self.stick.getRawButton(1))





if __name__ == "__main__":
    wpilib.run(MyRobot)
