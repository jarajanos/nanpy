"""Pressure sensor demo.

Connect the BME280 to the I2C bus.

"""
from __future__ import division

from nanpy.arduinotree import ArduinoTree
from nanpy.serialmanager import SerialManager

from nanpy.bme280 import BME280


def main():
    connection = SerialManager(sleep_after_connect=2)
    connection.open()
    a = ArduinoTree(connection=connection)
    bme = BME280(a.wire)
    t, h, p = bme.read()
    print ('pressure: %d kPa' % (p / 1000))
    print ('temperature: %d C' % t)
    print ('humidity: %d %' % h)

if __name__ == '__main__':
    main()
