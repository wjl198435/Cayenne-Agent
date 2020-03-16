
import pigpio
import time
import datetime
from myDevices.devices.sensor import Gas
from myDevices.utils.logger import debug, info,setInfo,error

class MHZ19B():

    def __init__(self,pin=21):
        self.tick0 = None
        self.tick1 = None
        self.measure_time = 0
        self.Th = 0
        self.Tl = 0
        self.pi=pigpio.pi()
        self.cb = self.pi.callback(pin, pigpio.EITHER_EDGE, self.mycallback)

        self.CO2_ppm = 0

    def __del__(self):
        self.cb.cancel()

    def __family__(self):
        return "Gas"

    def __str__(self):
        return "CO2-MHZ19B"

    def __getGas__(self):
        return self.CO2_ppm
        # pass

    def mycallback(self,gpio, level, tick):
        if level == 0:
            self.tick0 = tick
            if self.tick1 is not None:
                diff = pigpio.tickDiff(self.tick1, tick)
                self.Th = diff/1000
                debug ("high for " + str(diff) + " microseconds " + str(self.Th))
        else:
            self.tick1 = tick
            if self.tick0 is not None:
                diff = pigpio.tickDiff(self.tick0, tick)
                self.Tl = diff/1000
                debug ("low for " + str(diff) + " microseconds "+ str(self.Tl))
                if self.Th >0 :
                    self.CO2_ppm = 2000*(self.Th-2)/(self.Th+self.Tl-4)

                    self.measure_time = datetime.datetime.now().timestamp()
                    debug("ppm:{}".format(self.CO2_ppm))

    # def read(self):
    #     try:
    #         if self.Tl>0 and self.Th>0:
    #             return CO2Result(CO2Result.ERR_NO_ERROR,self.CO2_ppm,self.measure_time)
    #         else :
    #             return CO2Result(CO2Result.ERR_MISSING_DATA,-1,self.measure_time)
    #     except Exception as e:
    #         error(e)
    #         return CO2Result(CO2Result.ERR_MISSING_DATA,-1,self.measure_time)



    # def getCO2ppm(self):
    #     return round(self.CO2_ppm,2)