#   Copyright 2012-2013 Eric Ptak - trouch.com
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.

from myDevices.decorators.rest import request, response
from myDevices.utils.types import M_JSON

class GPIOPort():
    IN  = 0
    OUT = 1
    
    LOW  = False
    HIGH = True
    def __init__(self, channelCount):
        self.digitalChannelCount = channelCount
        
    def checkDigitalChannel(self, channel):
        if not 0 <= channel < self.digitalChannelCount:
            raise ValueError("Channel %d out of range [%d..%d]" % (channel, 0, self.digitalChannelCount-1))

    def checkDigitalValue(self, value):
        if not (value == 0 or value == 1):
            raise ValueError("Value %d not in {0, 1}")
    
    @response("%d")
    def digitalCount(self):
        return self.digitalChannelCount

    def __family__(self):
        return "GPIOPort"
    
    def __getFunction__(self, channel):
        raise NotImplementedError
    
    def __setFunction__(self, channel, func):
        raise NotImplementedError
    
    def __digitalRead__(self, channel):
        raise NotImplementedError
        
    def __portRead__(self):
        raise NotImplementedError
    
    def __digitalWrite__(self, channel, value):
        raise NotImplementedError
        
    def __portWrite__(self, value):
        raise NotImplementedError
  
    def getFunction(self, channel):
        self.checkDigitalChannel(channel)
        return self.__getFunction__(channel)  

    def getFunctionString(self, channel):
        func = self.getFunction(channel)
        if func == self.IN:
            return "IN"
        elif func == self.OUT:
            return "OUT"
#        elif func == GPIO.PWM:
#            return "PWM"
        else:
            return "UNKNOWN"

    def setFunction(self, channel, value):
        self.checkDigitalChannel(channel)
        self.__setFunction__(channel, value)
        return self.__getFunction__(channel)

    def setFunctionString(self, channel, value):
        value = value.lower()
        if value == "in":
            self.setFunction(channel, self.IN)
        elif value == "out":
            self.setFunction(channel, self.OUT)
#        elif value == "pwm":
#            self.setFunction(channel, GPIO.PWM)
        else:
            raise ValueError("Bad Function")
        return self.getFunctionString(channel)

    @response("%d")
    def digitalRead(self, channel):
        self.checkDigitalChannel(channel)
        return self.__digitalRead__(channel)

    @response(contentType=M_JSON)
    def wildcard(self, compact=False):
        if compact:
            f = "f"
            v = "v"
        else:
            f = "function"
            v = "value"
            
        values = {}
        for i in range(self.digitalChannelCount):
            if compact:
                func = self.__getFunction__(i)
            else:
                func = self.getFunctionString(i)
            values[i] = {f: func, v: int(self.digitalRead(i))}
        return values

    @response("%d")
    def portRead(self):
        return self.__portRead__()
    
    @response("%d")
    def digitalWrite(self, channel, value):
        self.checkDigitalChannel(channel)
        self.checkDigitalValue(value)
        self.__digitalWrite__(channel, value)
        return self.digitalRead(channel)  

    @response("%d")
    def portWrite(self, value):
        self.__portWrite__(value)
        return self.portRead()


DRIVERS = {}
DRIVERS["helper"] = ["DigitalSensor", "DigitalActuator", "LightSwitch", "MotorSwitch", "RelaySwitch", "ValveSwitch", "MotionSensor"]
DRIVERS["pcf8574" ] = ["PCF8574", "PCF8574A"]
DRIVERS["ds2408" ] = ["DS2408"]
