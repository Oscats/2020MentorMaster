import wpilib
from wpilib import drive
import ctre


class MyRobot(wpilib.TimedRobot):
    
    def robotInit(self):
        self.left_motor = wpilib.Spark(0)
        self.right_motor = wpilib.Spark(1)
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)
        self.stick = wpilib.Joystick(0)

        self.drive.setExpiration(0.1)
        self.tm = wpilib.Timer()
        self.tm.start()
    def autonomousInit(self):
         """Executed at the start of teleop mode"""
         self.drive.setSafetyEnabled(True)
         
    def autonomousPeriodic(self):


        #self.MyRobot.setSafetyEnabled(True)
        time = self.tm.get()

        if (time<15):
            self.drive.arcadeDrive(
                .8, 0, True
            )
        else:
            self.drive.arcadeDrive(
                0, 0, True
            )
        

if __name__ == "__main__":
    wpilib.run(MyRobot)
