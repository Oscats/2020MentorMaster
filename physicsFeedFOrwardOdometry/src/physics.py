#
# See the notes for the other physics sample
#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import math
import hal.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains


class PhysicsEngine:
    """
       Simulates a 4-wheel mecanum robot using Tank Drive joystick control 
    """

    def __init__(self, physics_controller):
        """
            :param physics_controller: `pyfrc.physics.core.Physics` object
                                       to communicate simulation effects to
        """

        self.physics_controller = physics_controller

        # Motors
        self.lf_motor = hal.simulation.PWMSim(1)
        self.lr_motor = hal.simulation.PWMSim(2)
        self.rf_motor = hal.simulation.PWMSim(3)
        self.rr_motor = hal.simulation.PWMSim(4)

        # Gyro
        self.gyro = hal.simulation.AnalogGyroSim(1)

        self.drivetrain = drivetrains.MecanumDrivetrain()
        # Precompute the encoder constant
        self.frontLeftEncoder = hal.simulation.EncoderSim(0)
        self.rearLeftEncoder = hal.simulation.EncoderSim(1)
        self.frontRightEncoder = hal.simulation.EncoderSim(2)
        self.rearRightEncoder = hal.simulation.EncoderSim(3)
        
        # -> encoder counts per revolution / wheel circumference
        self.encoderDistanceCalculation = (360 / (0.5 * math.pi))
        self.frontLeftEncoderDistance = 0
        self.rearLeftEncoderDistance = 0
        self.frontRightEncoderDistance = 0
        self.rearRightEncoderDistance = 0


    def update_sim(self, now, tm_diff):
        """
            Called when the simulation parameters for the program need to be
            updated.
            
            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        """

        # Simulate the drivetrain
        lf_motor = self.lf_motor.getSpeed()
        lr_motor = self.lr_motor.getSpeed()
        rf_motor = self.rf_motor.getSpeed()
        rr_motor = self.rr_motor.getSpeed()

        speeds = self.drivetrain.calculate(lf_motor, lr_motor, rf_motor, rr_motor)
        pose = self.physics_controller.drive(speeds, tm_diff)
        # Update encoders

        self.frontLeftEncoderDistance += self.drivetrain.wheelSpeeds.frontLeft
        self.rearLeftEncoderDistance += self.drivetrain.wheelSpeeds.rearLeft
        self.frontRightEncoderDistance += self.drivetrain.wheelSpeeds.frontRight
        self.rearRightEncoderDistance += self.drivetrain.wheelSpeeds.rearRight
        

        self.frontLeftEncoder.setCount (int(self.encoderDistanceCalculation * self.frontLeftEncoderDistance ))
        self.rearLeftEncoder.setCount (int(self.encoderDistanceCalculation * self.rearLeftEncoderDistance ))
        self.frontRightEncoder.setCount (int(self.encoderDistanceCalculation * self.frontRightEncoderDistance ))
        self.rearRightEncoder.setCount (int(self.encoderDistanceCalculation * self.rearRightEncoderDistance ))

        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        self.gyro.setAngle(-pose.rotation().degrees())
