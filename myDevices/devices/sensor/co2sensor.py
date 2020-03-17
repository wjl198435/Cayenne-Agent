from myDevices.devices.analog.helper import AnalogSensor


import time
import datetime

from myDevices.devices import instance
from myDevices.utils.logger import debug, info,setInfo,error
from myDevices.devices.analog import DAC



class CO2Sensor():

    def __init__(self, adc, channel):
        self.adcname = adc
        self.adc = None
        self.setADCInstance()
        self.co2_channel = channel

    def setADCInstance(self):
        if not self.adc:
            self.adc = instance.deviceInstance(self.adcname)

    def __family__(self):
        return "CO2Sensor"

    def __str__(self):
        return "CO2Sensor mhz19b Vo Sensor"

    # @response("%d")
    def readCO2(self):

        return self.adc.analogReadVolt(self.co2_channel)