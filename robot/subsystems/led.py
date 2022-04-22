import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, AddressableLED, Spark
import constants


class Led(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Led')
        self.counter = 0
        self.led_counter = 0

        self.spark = Spark(8)
        self.spark.set(1)

        self.led_count = 15
        self.led_strip = AddressableLED(7)
        self.led_data = [AddressableLED.LEDData() for _ in range(self.led_count)]

        [led.setRGB(255, 0, 0) for led in self.led_data]

        self.led_strip.setLength(self.led_count)
        self.led_strip.setData(self.led_data)
        self.led_strip.start()
        self.mode = 'static'

    def periodic(self) -> None:
        if self.counter % 10 == 0:
            if self.mode == 'static':
                [led.setRGB(255, 0, 0) for led in self.led_data]
            elif self.mode == 'flash':
                if self.led_counter % 2 == 0: # every other cycle
                    [led.setRGB(0, 0, 255) for led in self.led_data]
                else:
                    [led.setRGB(0, 0, 0) for led in self.led_data]
            elif self.mode == 'loading':
                offset = self.led_counter % self.led_count
                for led, i in enumerate(self.led_data):
                    if offset <= i < offset + 5:
                        led.setRGB(0, 255, 0)
                    else:
                        led.setRGB(0, 0, 0)

            self.led_strip.setData(self.led_data)

            self.led_counter += 1

        self.counter += 1








