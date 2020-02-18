#!/usr/bin/env python3
import math
import wpilib
from wpilib.kinematics import DifferentialDriveKinematics
from wpilib.kinematics import DifferentialDriveWheelSpeeds
from wpilib.kinematics import ChassisSpeeds
import wpilib.drive

if wpilib.RobotBase.isSimulation():
    is_sim = True
    import physics
    import time
else:
    is_sim = False

GEAR_RATIO = 10.71

# measurements in metres
TRACK_WIDTH = 0.581  # theoretical as measured
WHEEL_CIRCUMFERENCE = 0.0254 * 6 * math.pi


class MyRobot(wpilib.TimedRobot):
    """Main robot class"""

    def robotInit(self):
        """Robot-wide initialization code should go here"""

        self.lstick = wpilib.Joystick(0)
        self.rstick = wpilib.Joystick(1)

        self.l_motor = wpilib.Jaguar(1)
        self.r_motor = wpilib.Jaguar(2)

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)

        self.drive = wpilib.drive.DifferentialDrive(self.l_motor, self.r_motor)

        self.motor = wpilib.Jaguar(4)

        self.limit1 = wpilib.DigitalInput(1)
        self.limit2 = wpilib.DigitalInput(2)

        self.position = wpilib.AnalogInput(2)
        self.left_encoder = wpilib.Encoder(1, 2)
        self.right_encoder = wpilib.Encoder(3, 4)

        self.kinematics = DifferentialDriveKinematics(TRACK_WIDTH)
        self.chassis_speeds = ChassisSpeeds()
        self.chassis_speeds.vx = 0.0
        self.chassis_speeds.omega = 0.0

        if is_sim:
            self.physics = physics.PhysicsEngine()
            self.last_tm = time.time()

    if is_sim:
        # TODO: this needs to be builtin
        def robotPeriodic(self):
            now = time.time()
            tm_diff = now - self.last_tm
            self.last_tm = now
            self.physics.update_sim(now, tm_diff)

    def autonomousInit(self):
        """Called when autonomous mode is enabled"""

        self.timer = wpilib.Timer()
        self.timer.start()

    def autonomousPeriodic(self):
        # Get WheelSPeeds out of Inverse Kinematics...
        # XXX: https://github.com/robotpy/robotpy-wpilib/issues/635
        
        #speeds = self.kinematics.toWheelSpeeds(self.chassis_speeds)
        # Uncomment to see wheel speed
        # print(speeds.left,  speeds.right)

        # Get Chassis Speeds using Kinematics
        #sampleWheelSpeeds = DifferentialDriveWheelSpeeds()
        #sampleWheelSpeeds.left = 2.0
        #sampleWheelSpeeds.right = 2.0

        #speeds2 = self.kinematics.toChassisSpeeds(sampleWheelSpeeds)

        #print(speeds2.vx, speeds2.omega)

        if self.timer.get() < 2.0:
            self.chassis_speeds.vx = -.5
            
        else:
            self.chassis_speeds.vx = 0.0
        
        speeds = self.kinematics.toWheelSpeeds(self.chassis_speeds)
        self.drive.tankDrive(speeds.left, speeds.right)

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""

        self.drive.arcadeDrive(self.lstick.getY(), self.lstick.getX())
        # self.drive.arcadeDrive(self.lstick.getRawAxis(1), self.lstick.getRawAxis(3))

        # Move a motor with a Joystick
        y = self.rstick.getY()

        # stop movement backwards when 1 is on
        if self.limit1.get():
            y = max(0, y)

        # stop movement forwards when 2 is on
        if self.limit2.get():
            y = min(0, y)

        self.motor.set(y)


if __name__ == "__main__":

    wpilib.run(MyRobot)
