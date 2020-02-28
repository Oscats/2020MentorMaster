import wpilib
from wpilib import AddressableLED

k_numberLEDs = 60


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """"Runs Once on Robot Code Start."""
        # Instantiate an LED Object on PWM pin 0.
        self.led = AddressableLED(0)
        # set the number of leds
        self.led.setLength(k_numberLEDs)
        # Create an LED Data Object for the right and left sides (this can be re-created for animations)
        self.left = [wpilib.AddressableLED.LEDData(255, 0, 0) for _ in range(30)]
        self.right = [wpilib.AddressableLED.LEDData(255, 0, 0) for _ in range(30)]

        # Now the LEDs are setup, we just need to Fill the LED Buffer with data.
        # For instance, let's fill with purple (red + blue) while we wait for our alliance color.
        # Create the n variable
        i = 0
        # Loop through the strip setting the color for both the left and right strips.
        for d in self.left:
            self.left[i].setRGB(100, 0, 100)
            self.right[i].setRGB(100, 0, 100)
            # Increment the number
            i += 1
        # Now, lets fill the data object with the colors.
        self.led.setData(self.left + self.right)
        # Finally, write the data to the LED strip (if this stays open, it will update automatically).
        self.led.start()

    def autonomousInit(self):
        """Runs Once during auto"""
        # By auto, we should have our alliance color, let's grab it.
        m_alliance = wpilib.DriverStation.Alliance(0)

        # reset n
        i = 0
        # set the leds to the alliance color.
        for d in self.left:
            if m_alliance == m_alliance.kRed:
                self.left[i].setRGB(100, 0, 0)
                self.right[i].setRGB(100, 0, 0)
            elif m_alliance == m_alliance.kBlue:
                self.left[i].setRGB(0, 0, 100)
                self.right[i].setRGB(0, 0, 100)
            else:
                self.left[i].setRGB(100, 0, 100)
                self.right[i].setRGB(100, 0, 100)
            i += 1
        # fill the buffer with color.
        self.led.setData(self.left + self.right)

    def autonomousPeriodic(self):
        """Runs Periodically during auto"""

    def teleopInit(self):
        """Runs Once during teleop"""
        self.i = 0
        self.rainbow = 0

    def teleopPeriodic(self):
        """Runs the motors with Mecanum drive."""
        # Maybe, we want to create a rainbow.  HSV colorspace works well for this.

        hue = (self.rainbow + (self.i * 180 / 30)) % 180
        self.left[self.i].setHSV(int(hue), 255, 128)
        self.right[self.i].setHSV(int(hue), 255, 128)
        if self.i < 29:
            self.i += 1
        else:
            self.i = 0

        self.rainbow += 3
        self.rainbow %= 180

        # Write color to the buffer
        self.led.setData(self.left + self.right)


if __name__ == "__main__":
    """Run the code for your robot"""
    wpilib.run(MyRobot)
