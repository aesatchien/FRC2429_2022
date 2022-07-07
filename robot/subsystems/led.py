from commands2 import SubsystemBase
from wpilib import SmartDashboard, AddressableLED, SendableChooser
import constants


class Led(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Led')

        self.counter = 0
        self.led_counter = 0

        self.led_count = constants.k_led_count
        self.led_strip = AddressableLED(constants.k_led_strip_port)
        self.led_data = [AddressableLED.LEDData(0, 0, 0) for _ in range(self.led_count)]
        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()

        self.mode = 'static'

        self.modes = SendableChooser()
        self.modes.setDefaultOption('Static', 'static')
        self.modes.addOption('Flashing', 'flash')
        self.modes.addOption('Loading', 'loading')

        SmartDashboard.putBoolean('Led/dashboard_override', False)
        SmartDashboard.putData('Led/led_modes', self.modes)

    def periodic(self) -> None:
        if self.counter % 10 == 0:
            if SmartDashboard.getBoolean('Led/dashboard_override', False):
                self.mode = self.modes.getSelected()

            if self.mode == 'static':
                [led.setRGB(255, 0, 0) for led in self.led_data]
            elif self.mode == 'flash':
                if self.led_counter % 2 == 0: # every other update cycle
                    [led.setRGB(0, 0, 255) for led in self.led_data]
                else:
                    [led.setRGB(0, 0, 0) for led in self.led_data]
            elif self.mode == 'loading':
                offset = self.led_counter % self.led_count
                bar_len = 5

                # clear strip
                [led.setRGB(0, 0, 0) for led in self.led_data]

                for i in range(0, bar_len):
                    self.led_data[(offset + i) % self.led_count].setRGB(0, 255, 0)

            self.led_strip.setData(self.led_data)
            self.led_counter += 1

        self.counter += 1








