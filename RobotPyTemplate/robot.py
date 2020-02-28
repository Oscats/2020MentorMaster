import wpilib

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """"Runs Once on Robot Code Start."""

    def autonomousInit(self):
        """Runs Once during auto"""
        
    def autonomousPeriodic(self):
        """Runs Periodically during auto"""
    
    def teleopInit(self):
        """Runs Once during teleop"""
        self.drive.setSafetyEnabled(True)
        

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""





if __name__ == "__main__":
    """Run the code for your robot"""
    wpilib.run(MyRobot)
