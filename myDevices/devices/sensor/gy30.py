from myDevices.utils.types import toint
from myDevices.devices.i2c import I2C
from myDevices.devices.sensor import Luminosity

import time
from myDevices.utils.logger import debug, info, error,setInfo

class GY30(I2C, Luminosity):

    I2C_ADDRESS_L = 0x23 # Device I2C address for ADDR low
    I2C_ADDRESS_H = 0x5C # Device I2C address for ADDR high

    POWER_DOWN = 0x00 # No active state
    POWER_ON   = 0x01 # Waiting for measurment command
    RESET      = 0x07 # Reset data register value

    #Modes
    CONTINUOUSLY_H_RES_MODE = 0x10 # Start measurement at 1 lx resolution.
    # Measurement time is typically 120ms

    CONTINUOUSLY_H_RES_MODE_2 = 0x11 # Start measurement at 0.5 lx resolution.
    # Measurement time is typically 120ms

    CONTINUOUSLY_L_RES_MODE = 0x13 # Start measurement at 4 lx resolution.
    # Measurement time is typically 16ms.

    ONE_TIME_H_RES_MODE = 0x20 # Start measurement at 1 lx resolution.
    # Measurement time is typically 120ms
    # Device is automatically set to Power Down
    # after measurement.

    ONE_TIME_H_RES_MODE_2 = 0x21 # Start measurement at 0.5 lx resolution.
    # Measurement time is typically 120ms
    # Device is automatically set to Power Down
    # after measurement.

    ONE_TIME_L_RES_MODE = 0x23 # Start measurement at 4 lx resolution.
    # Measurement time is typically 16ms
    # Device is automatically set to Power Down
    # after measurement.

    def __init__(self, time=0.18, slave= 0x23,mode=4,  name="gy_30"):
        I2C.__init__(self, toint(slave))
        self.name = name
        # self.wake() # devices are powered down after power reset, wake them
        # self.setTime(toint(time))

        if slave == 0:
            self.addr = self.I2C_ADDRESS_L
        else:
            self.addr = self.I2C_ADDRESS_H
        if mode == 0:
            self.mode = self.CONTINUOUSLY_H_RES_MODE
            self.wait = 0.18 # Wait 180ms for result
            self.resolution_div = 1
        elif mode == 1:
            self.mode = self.CONTINUOUSLY_H_RES_MODE_2
            self.wait = 0.18 # Wait 180ms for result
            self.resolution_div = 2
        elif mode == 2:
            self.mode = self.CONTINUOUSLY_L_RES_MODE
            self.wait = 0.024 # Wait 24ms for result
            self.resolution_div = 1
        elif mode == 3:
            self.mode = self.ONE_TIME_H_RES_MODE
            self.wait = 0.18 # Wait 180ms for result
            self.resolution_div = 1
        elif mode == 4:
            self.mode = self.ONE_TIME_H_RES_MODE_2
            self.wait = 0.18 # Wait 180ms for result
            self.resolution_div = 2
        else:
            self.mode = self.ONE_TIME_L_RES_MODE
            self.wait = 0.024 # Wait 24ms for result
            self.resolution_div = 1

    def __str__(self):
        return "%s(slave=0x%02X)" % (self.name, self.slave)

    def wake(self):
        self.__wake__()

    def __wake__(self):
        self.writeRegister(self.REG_CONTROL, self.VAL_PWON)

    def sleep(self):
        self.__sleep__()

    def __sleep__(self):
        self.writeRegister(self.REG_CONTROL, self.VAL_PWOFF)

    def setTime(self, time):
        self.__setTime__(time)

    def getTime(self):
        return self.__getTime__()

    def set_mode(self):
        # Set mode on I2C interface
        # print("set_mode",self.addr, self.mode)
        # self.i2cbus.write_byte(self.addr, self.mode)
        self.writeByte(0x21)
        return

    def get_light(self):
        # Read light level from I2C interface
        # Must only be called after an appropriate wait for the next measurement
        # to be ready. Calling this directly in one time modes will result in
        # every measurement being out of date
        # self.data = self.i2cbus.read_i2c_block_data(self.addr,
        #                                             self.mode)
        self.data = self.readBytes(2)
        # Should not need to specify
        # command, but smbus lacks a
        # multi-byte read only command
        lux=(self.data[1] + 256*self.data[0]) / 1.2 / self.resolution_div
        return lux

    def get_light_mode(self):
        # Set mode and read light level from I2C interface
        self.set_mode()
        time.sleep(self.wait) # Wait for result
        return round(self.get_light(),2)


    def __getLux__(self):
        lux = self.get_light_mode()
        return lux

if __name__ == "__main__":
    setInfo()
    gy30 = GY30()
    while True:

        # print(gy30)
        # gy30.set_mode()
        # print(gy30.get_light())
        result = gy30.getLux()
        info(result)
        time.sleep(1)