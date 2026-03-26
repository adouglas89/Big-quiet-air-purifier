from micropython import const
from machine import Pin, I2C

# ADXL-345 Registers
_DATA_FORMAT = const(0x31)
_POWER_CTL = const(0x2D)
_DATAX0 = const(0x32)

class ADXL345:
    def __init__(self, i2c, address=0x53):
        self.i2c = i2c
        self.address = address
        #self.i2c.start()
        self.i2c.writeto(self.address, bytearray([_POWER_CTL, 0x08])) # power on
        self.i2c.writeto(self.address, bytearray([_DATA_FORMAT, 0x08])) # full resolution mode
        #self.i2c.stop()

    def read(self):
        #self.i2c.start()
        self.i2c.writeto(self.address, bytearray([_DATAX0]))
        data = self.i2c.readfrom(self.address, 6)
        #self.i2c.stop()
        x = ((data[1] << 8) | data[0])
        y = ((data[3] << 8) | data[2])
        z = ((data[5] << 8) | data[4])
        return (x, y, z)