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
#import gyro
import navx




class MyRobot(wpilib.TimedRobot):
    # Channels on the roboRIO that the motor controllers are plugged in to
    frontLeftChannel = 2
    rearLeftChannel = 3
    frontRightChannel = 1
    rearRightChannel = 0
    

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

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

        # invert the left side motors
        self.frontLeftMotor.setInverted(True)

        # you may need to change or remove this to match your robot
        self.rearLeftMotor.setInverted(True)

        self.drive = MecanumDrive(
            self.frontLeftMotor,
            self.rearLeftMotor,
            self.frontRightMotor,
            self.rearRightMotor,
        )

        self.drive.setExpiration(0.1)

        self.stick = wpilib.XboxController(self.joystickChannel)

        #
        # Communicate w/navX MXP via the MXP SPI Bus.
        # - Alternatively, use the i2c bus.
        # See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details
        #

        self.navx = navx.AHRS.create_spi()
        # self.navx = navx.AHRS.create_i2c()

        # Analog input
        # self.analog = wpilib.AnalogInput(navx.pins.getNavxAnalogInChannel(0)) <--It seems as though the analog channel is not currently supported.


    def robotPeriodic(self):
        self.timer.reset()
        self.timer.start()

        while self.isDisabled():

            if self.timer.hasPeriodPassed(0.5):
                self.sd.putNumber("Displacement X", self.navx.getDisplacementX())
                self.sd.putNumber("Displacement Y", self.navx.getDisplacementY())
                self.sd.putBoolean("IsCalibrating", self.navx.isCalibrating())
                self.sd.putBoolean("IsConnected", self.navx.isConnected())
                self.sd.putNumber("Angle", self.navx.getAngle())
                self.sd.putNumber("Pitch", self.navx.getPitch())
                self.sd.putNumber("Yaw", self.navx.getYaw())
                self.sd.putNumber("Roll", self.navx.getRoll())
                # self.sd.putNumber("Analog", self.analog.getVoltage())
                self.sd.putNumber("Timestamp", self.navx.getLastSensorTimestamp())
    
    def autonomousInit(self):
        """Runs Once during auto"""
        
    def autonomousPeriodic(self):
        """Runs Periodically during auto"""
        self.drive.driveCartesian(
            .5, 0, .5, 0
        )

    def teleopInit(self):
        """Runs Once during teleop"""
        self.drive.setSafetyEnabled(True)
        

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        # This sample does not use field-oriented drive, so the gyro input is set to zero.
        self.drive.driveCartesian(
            self.stick.getRawAxis(1), self.stick.getRawAxis(3), self.stick.getRawAxis(2), self.navx.getAngle()
        )
        self.sd.putNumber("Field Angle",self.navx.getAngle())





if __name__ == "__main__":
    wpilib.run(MyRobot)
