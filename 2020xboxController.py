import wpilib
from wpilib.interfaces import GenericHID

Hand = GenericHID.Hand

....

# In robotinit
joystickChannel= 0
self.stick = wpilib.XboxController(joystickChannel)
......

#In teleopPeriodic
self.drive.driveCartesian(
    self.stick.getRawAxis(1), self.stick.getRawAxis(3), self.stick.getRawAxis(0), 0 # <----This works
    self.stick.getX(self.stick.Hand.kLeftHand), self.stick.getY(self.stick.Hand.kRightHand), self.stick.getX(self.stick.Hand.kLeftHand), 0
            #This does not work----- > self.stick.getX(9), self.stick.getY(9), self.stick.getX(10), 0
            #Possibly----- > self.stick.getX(0), self.stick.getY(0), self.stick.getX(1), 0
            
            #self.stick.getX(Hand.kRightHand), self.stick.getY(Hand.kRightHand), self.stick.getX(Hand.kLeftHand), 0
        )
.........
