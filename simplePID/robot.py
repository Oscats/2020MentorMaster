import wpilib
from wpilib import drive
from wpilib.controller import PIDController
import ctre

from navx import AHRS


class MyRobot(wpilib.TimedRobot):
    if wpilib.RobotBase.isSimulation():
            # These PID parameters are used in simulation
            kP = 0.06
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
        self.left_motor = ctre.WPI_TalonSRX(0)
        self.right_motor = ctre.WPI_TalonSRX(1)
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)
        self.stick = wpilib.Joystick(0)
        self.ahrs = AHRS.create_spi()
        # self.ahrs = AHRS.create_i2c()

        turnController = PIDController(
            self.kP, self.kI, self.kD, period = 1.0
        )

        self.turnController = turnController
        self.rotateToAngleRate = 5
        self.drive.setExpiration(0.1)
        self.tm = wpilib.Timer()
        self.tm.start()
    def autonomousInit(self):
         """Executed at the start of teleop mode"""
         self.drive.setSafetyEnabled(True)
         
    def autonomousPeriodic(self):


        #self.MyRobot.setSafetyEnabled(True)
        time = self.tm.get()

        if (time >= (1)) and (time<=(25)):
                print("NavX Gyro", self.ahrs.getYaw(), self.ahrs.getAngle())
                self.drive.tankDrive(0, self.turnController.calculate(self.ahrs.getYaw()), True)
                print (self.turnController.calculate(self.ahrs.getYaw()))
        else:
            self.drive.tankDrive(
                0, 0, True
            )
        

if __name__ == "__main__":
    wpilib.run(MyRobot)
