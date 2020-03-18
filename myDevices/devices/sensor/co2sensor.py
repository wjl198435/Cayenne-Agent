from myDevices.devices.analog.helper import AnalogSensor


import time
import datetime

from myDevices.devices import instance
from myDevices.utils.logger import debug, info,setInfo,error
from myDevices.devices.analog import DAC



class CO2Sensor():
    RANGE = 2000
    VH = 2.0
    VL = 0.4

    def __init__(self, adc, channel):
        self.adcname = adc
        self.adc = None
        self.setADCInstance()
        self.co2_channel = channel

    def setADCInstance(self):
        if not self.adc:
            self.adc = instance.deviceInstance(self.adcname)

            info(" setADCInstance  self.adcname={} self.adc={}".format(self.adcname,self.adc))

    def __family__(self):
        return "CO2Sensor"

    def __str__(self):
        return "CO2Sensor mhz19b Vo Sensor"

    # @response("%d")
    def readCO2(self):

        # 模拟电压输出与浓度之间的换算关系，以 0.4V~2.0V 输出范围为例:
        # Vo(V)=0.4V+(2.0V-0.4V)*C(浓度 ppm) /量程(ppm)
        vo = 0.0
        if self.adc:
            vo = self.adc.analogReadVolt(self.co2_channel)
        else:
            error("adc is None")

        return int((vo-0.4)*self.RANGE/(self.VH-self.VL))