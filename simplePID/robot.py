import wpilib
from wpilib import drive
from networktables import NetworkTables
from wpilib.controller import PIDController
import ctre

import navx


class MyRobot(wpilib.TimedRobot):
    if wpilib.RobotBase.isSimulation():
            # These PID parameters are used in simulation
            kP = .06
            kI = 0.00
            kD = 0.00
            kF = 0.00
    else:
        # These PID parameters are used on a real robot
        kP = 0.03
        kI = 0.00
        kD = 0.00
        kF = 0.00

    kToleranceDegrees = 2.0

    def robotInit(self):
        self.sd = NetworkTables.getTable("SmartDashboard")
        self.left_motor = ctre.WPI_TalonSRX(0)
        self.right_motor = ctre.WPI_TalonSRX(1)
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)
        self.stick = wpilib.Joystick(0)
        self.navx = navx.AHRS.create_spi()
        # self.ahrs = AHRS.create_i2c()

        turnController = PIDController(
            self.kP, self.kI, self.kD, period = 1.0
        )

        self.turnController = turnController
        self.rotateToAngleRate = 5
    def autonomousInit(self):
         """Executed at the start of teleop mode"""
         self.drive.setSafetyEnabled(True)
         
    def autonomousPeriodic(self):


        
        self.turnController.setSetpoint(0)
        pidOutput = self.turnController.calculate(self.navx.getAngle())

        

        self.drive.arcadeDrive(pidOutput,0)
        self.sd.putNumber("Field Angle",self.navx.getAngle())
        

if __name__ == "__main__":
    wpilib.run(MyRobot)
