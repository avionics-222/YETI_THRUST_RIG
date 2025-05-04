# hx711_module.py
import time
import statistics
from gpiozero import DigitalInputDevice, DigitalOutputDevice

class HX711:
    def __init__(self, dout_pin, sck_pin, calibration_factor=145):
        self.dout = DigitalInputDevice(dout_pin)
        self.sck = DigitalOutputDevice(sck_pin, initial_value=False)
        self.calibration_factor = calibration_factor
        self.offset = 0
        self.last_valid_reading = 0
        time.sleep(0.1)

    def read(self):
        timeout_start = time.time()
        while self.dout.value == 1:
            if time.time() - timeout_start > 0.1:
                return None

        data = 0
        for _ in range(24):
            self.sck.on()
            self.sck.off()
            data = (data << 1) | self.dout.value

        self.sck.on()
        self.sck.off()

        if data & 0x800000:
            data -= 0x1000000

        return None if data == 0 or data == -1 else data

    def read_average(self, num_readings=3, max_retries=5):
        readings = []
        retries = 0
        while len(readings) < num_readings and retries < max_retries:
            reading = self.read()
            if reading is not None:
                readings.append(reading)
            else:
                retries += 1
            time.sleep(0.005)
        return statistics.median(readings) if readings else self.last_valid_reading or 0

    def zero(self, num_readings=10):
        self.offset = self.read_average(num_readings)
        return self.offset

    def get_weight(self, num_readings=3):
        reading = self.read_average(num_readings)
        raw_value = reading - self.offset
        return raw_value / self.calibration_factor, raw_value
