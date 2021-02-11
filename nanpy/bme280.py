from __future__ import division

import logging
from nanpy.i2c import I2C_Master
import time
import ctypes


log = logging.getLogger(__name__)


class BME280(object):
    """Control of BME280 Digital pressure sensor (I2C)

    calculation is based on Bosch datasheet and Adafruit library."""

    def __init__(self, wire, address=0x77):
        self.i2c = I2C_Master(wire)
        self.address = address
        self.constants = {}
        self.act_temp = False

        # soft reset to safely reset config
        self.write_byte(0xe0, 0xb6)

        time.sleep(0.01)

        # wait for wake-up
        while (self.is_calibrating()):
            time.sleep(0.01)

        self.read_constants()
        self.configure()

        time.sleep(0.1)


    def read_bytes(self, address, count):
        self.i2c.send(self.address, [address])
        x = self.i2c.request(self.address, count)
        return x

    def write_byte(self, address, data):
        self.i2c.send(self.address, [address, data])

    def is_calibrating(self):
        status = self.read_bytes(0xf3, 1)
        return (status & 0x01) != 0

    def read_constants(self):
        calib_1 = self.read_bytes(0x88, 26)
        self.constants["T1"] = (calib_1[1] << 8) | calib_1[0]
        self.constants["T2"] = ctypes.c_short((calib_1[3] << 8) | calib_1[2]).value
        self.constants["T3"] = ctypes.c_short((calib_1[5] << 8) | calib_1[4]).value
        self.constants["P1"] = (calib_1[7] << 8) | calib_1[6]
        self.constants["P2"] = ctypes.c_short((calib_1[9] << 8) | calib_1[8]).value
        self.constants["P3"] = ctypes.c_short((calib_1[11] << 8) | calib_1[10]).value
        self.constants["P4"] = ctypes.c_short((calib_1[13] << 8) | calib_1[12]).value
        self.constants["P5"] = ctypes.c_short((calib_1[15] << 8) | calib_1[14]).value
        self.constants["P6"] = ctypes.c_short((calib_1[17] << 8) | calib_1[16]).value
        self.constants["P7"] = ctypes.c_short((calib_1[19] << 8) | calib_1[18]).value
        self.constants["P8"] = ctypes.c_short((calib_1[21] << 8) | calib_1[20]).value
        self.constants["P9"] = ctypes.c_short((calib_1[23] << 8) | calib_1[22]).value
        self.constants["H1"] = calib_1[25]

        calib_2 = self.read_bytes(0xe1, 7)
        self.constants["H2"] = ctypes.c_short((calib_2[1] << 8) | calib_2[0]).value
        self.constants["H3"] = calib_2[2]
        self.constants["H4"] = ctypes.c_short((calib_2[3] << 4) | (calib_2[4] & 0x0f)).value
        self.constants["H5"] = ctypes.c_short((calib_2[5] << 4) | (calib_2[4] >> 4)).value
        self.constants["H6"] = ctypes.c_char(calib_2[6]).value

    def configure(self):
        # ensure SLEEP MODE
        self.write_byte(0xf4, 0x00)

        # write config
        self.write_byte(0xf2, 0b101)  # humidity oversampling 16x
        self.write_byte(0xf5, 0b10100000) # measure once per 1000 ms, filter off, spi off
        self.write_byte(0xf4, 0b10110111) # oversample temp 16x, press 16x, normal mode

    def read_temperature(self):
        msb, lsb, xlsb = self.i2c.read_bytes(0xfa, 3)
        if msb == 0x80:
            return float("nan")
        
        temp = ((msb << 16) | (lsb << 8) | xlsb) >> 4

        var1 = (((temp >> 3) - (self.constants["T1"] << 1)) * self.constants["T2"]) >> 11
        var2 = (((((temp >> 4) - self.constants["T1"]) ** 2) >> 12) * self.constants["T3"]) >> 14
        self.temp_fine = var1 + var2

        t = (temp_fine * 5 + 128) >> 8
        return t / 100.0
  
    def read_humidity(self):
        if not self.act_temp:
            self.read_temperature() # needed for temp_fine 

        msb, lsb = self.read_bytes(0xfd, 2)
        if msb == 0x80:
            return float("nan")

        hum = (msb << 8) | lsb
        v_x1_u32r = self.temp_fine - 76800

        v_x1_u32r = (((((hum << 14) - (self.constants["H4"] << 20) - (self.constants["H5"] * v_x1_u32r)) + 16384) >> 15) *
                    (((((((v_x1_u32r * self.constants["H6"]) >> 10) * (((v_x1_u32r * self.constants["H3"]) >> 11) + 32768)) >> 10) + 2097152) *
                     self.constants["H2"] + 8192) >> 14))
        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) ** 2) >> 7) * self.constants["H1"]) >> 4))
        v_x1_u32r = min(max(0, v_x1_u32r), 419430400)
        
        h = v_x1_u32r >> 12
        return h / 1024.0

    def read_pressure(self):
        if not self.act_temp:
            self.read_temperature() # to get temp_fine

        msb, lsb, xlsb = self.read_bytes(0xf7, 3)
        if msb == 0x80:
            return float("nan")
        
        press = ((msb << 16) | (lsb << 8) | xlsb) >> 4

        var1 = temp_fine - 128000
        var2 = (var1 ** 2) * self.constants["P6"]
        var2 += (var1 * self.constants["P5"]) << 17
        var2 += self.constants["P4"] << 35
        var1 = (((var1 ** 2) * self.constants["P3"]) >> 8) + ((var1 * self.constants["P2"]) << 12)
        var1 = ((1 << 47) + var1) * (self.constants["P1"] >> 33)

        if var1 == 0:
            return 0

        p = 1048576 - press
        p = (((p << 31) - var2) * 3125) / var1
        var1 = (self.constants["P9"] * ((p >> 13) ** 2)) >> 25
        var2 = (self.constants["P8"] * p) >> 19

        p = ((p + var1 + var2) >> 8) + (self.constants["P7"] << 4)
        return p / 256.0

    def read(self):
        '''
        return: temperature (*C), humidity (%), pressure (Pascal)
        '''
        temperature = self.read_temperature()
        self.act_temp = True
        
        humidity = self.read_humidity()
        pressure = self.read_pressure()
        self.act_temp = False

        return [temperature, humidity, pressure]
