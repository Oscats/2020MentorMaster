#!/usr/bin/env python3
"""
    This is a demo program showing how to use Mecanum control with the
    MecanumDrive class.
"""

import wpilib
from wpilib.drive import MecanumDrive
# Import the SendableChooser

from wpilib import SendableChooser
from wpilib import SmartDashboard
# Import Network Tables
from networktables import NetworkTables



class MyRobot(wpilib.TimedRobot):
    # This shows the user two options on the ShuffleBoard


    def robotInit(self):
        ##sd = NetworkTables.getTable('SmartDashboard')
        self.left_motor = wpilib.Spark(0)
        self.right_motor = wpilib.Spark(1)
        self.drive = wpilib.drive.DifferentialDrive(self.left_motor, self.right_motor)
    
        #wpilib.SmartDashboard.getString("Choice", "null")
        choice = SendableChooser()
        choice.setDefaultOption('option0', 0)
        choice.addOption('option1', 1)
        choice.addOption('option2', 2)

        SmartDashboard.putData("Test", choice)
        
        self.drive.setExpiration(0.1)
        self.tm = wpilib.Timer()
        self.tm.start()
    def autonomousInit(self):
         """Executed at the start of teleop mode"""
         self.drive.setSafetyEnabled(True)
         #wpilib.SmartDashboard.getString("Choice", "null")
         
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
