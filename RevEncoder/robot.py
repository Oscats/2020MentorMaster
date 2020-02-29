#!/usr/bin/env python3

# Copyright (c) 2017-2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

import wpilib
import rev


class MyRobot(wpilib.TimedRobot):
    """
        This is a simple example to show the values that can be read from the REV
        Color Sensor V3
    """

    def robotInit(self):
        self.leftFront = rev.CANSparkMax(1)
        self.leftRear = rev.CANSparkMax(2)
        self.rightFront = rev.CANSparkMax(3)
        self.leftFront = rev.CANSparkMax(4)

        # Let's add our encoders...
        self.leftFront_encoder = rev.CANEncoder(self.leftFront)
        self.leftRear_encoder =  rev.CANEncoder(self.leftRear)
        self.rightFront_encoder =rev.CANEncoder(self.rightFront)
        self.leftFront_encoder  =rev.CANEncoder(self.leftFront)

        # Maybe we should specify our units.
        for enc in (self.leftFront_encoder, self.leftRear_encoder, self.rightFront_encoder,
                self.leftFront_encoder ):
            enc.setPositionConversionFactor(rev_to_m)
            enc.setVelocityConversionFactor(rev_to_m / 60)

    def robotPeriodic(self):

        slef.leftFrontValue = self.left_encoder.getVelocity()


        self.left_encoder.getVelocity()

        wpilib.SmartDashboard.putNumber("Proximity", proximity)


if __name__ == "__main__":
    wpilib.run(MyRobot)