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
            self.stick.getX(Hand.kRightHand), self.stick.getY(Hand.kRightHand), self.stick.getX(Hand.kLeftHand), 0
        )
.........
