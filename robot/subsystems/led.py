from math import floor
from commands2 import SubsystemBase
from wpilib import SmartDashboard, AddressableLED, Color8Bit, SendableChooser
import constants


class Led(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Led')
        self.counter = 0

        self.led_count = constants.k_led_count
        self.led_strip = AddressableLED(constants.k_led_strip_port)
        self.led_data = [AddressableLED.LEDData(0, 0, 0) for _ in range(self.led_count)]
        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()

        self.needs_update = True
        self.paused = False

        self.mode = 'static'

        self.modes = SendableChooser()
        self.modes.setDefaultOption('Static', 'static')
        self.modes.addOption('Flashing', 'flash')
        self.modes.addOption('Loading', 'loading')
        self.modes.addOption('Off', 'off')

        SmartDashboard.putBoolean('Led/dashboard_override', False)
        SmartDashboard.putData('Led/led_modes', self.modes)

    def force_paused(self, paused: bool) -> None:
        self.paused = paused

    def change_mode(self, new_mode: str) -> None:
        if new_mode != self.mode:
            self.mode = new_mode
            self.needs_update = True

    def periodic(self) -> None:
        interval = 10

        # update LED strip 5 times/second
        if self.counter % interval == 0 and not self.paused:
            led_cycle = self.counter / interval

            if SmartDashboard.getBoolean('Led/dashboard_override', False):
                self.change_mode(new_mode=self.modes.getSelected())

            if self.needs_update:
                if self.mode == 'off':
                    [led.setRGB(0, 0, 0) for led in self.led_data]
                    self.needs_update = False
                elif self.mode == 'static':
                    [led.setRGB(255, 0, 0) for led in self.led_data]
                    self.needs_update = False
                elif self.mode == 'flash':
                    if led_cycle % 2 == 0: # every other update cycle
                        [led.setRGB(0, 0, 255) for led in self.led_data]
                    else:
                        [led.setRGB(0, 0, 0) for led in self.led_data]
                elif self.mode == 'loading':
                    bar_len = 5
                    bar_count = 5

                    offset = led_cycle % self.led_count
                    stride = self.led_count / bar_count

                    # clear strip
                    [led.setRGB(0, 0, 0) for led in self.led_data]

                    for i in range(0, bar_count):
                        for j in range(0, bar_len):
                            index = floor(offset + i * stride + j) % self.led_count

                            self.led_data[index].setRGB(0, 255, 0)

            self.led_strip.setData(self.led_data)

        self.counter += 1








