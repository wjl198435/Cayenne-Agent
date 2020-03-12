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
from myDevices.utils.types import toint, M_JSON
from myDevices.devices import instance
from myDevices.utils.logger import info


class ADC():
    def __init__(self, channelCount, resolution, vref):
        self._analogCount = channelCount
        self._analogResolution = resolution
        self._analogMax = 2**resolution - 1
        self._analogRef = vref
    
    def __family__(self):
        return "ADC"

    def checkAnalogChannel(self, channel):
        if not 0 <= channel < self._analogCount:
            raise ValueError("Channel %d out of range [%d..%d]" % (channel, 0, self._analogCount-1))

    def checkAnalogValue(self, value):
        if not 0 <= value <= self._analogMax:
            raise ValueError("Value %d out of range [%d..%d]" % (value, 0, self._analogMax))
    
    @response("%d")
    def analogCount(self):
        return self._analogCount

    @response("%d")
    def analogResolution(self):
        return self._analogResolution
    
    @response("%d")
    def analogMaximum(self):
        return int(self._analogMax)
    
    @response("%.2f")
    def analogReference(self):
        return self._analogRef
    
    def __analogRead__(self, channel, diff):
        raise NotImplementedError
    
    @response("%d")
    def analogRead(self, channel, diff=False):
        self.checkAnalogChannel(channel)
        return self.__analogRead__(channel, diff)
    
    @response("%.2f")
    def analogReadFloat(self, channel, diff=False):
        return self.analogRead(channel, diff) / float(self._analogMax)
    
    @response("%.2f")
    def analogReadVolt(self, channel, diff=False):
        if self._analogRef == 0:
            raise NotImplementedError
        return self.analogReadFloat(channel, diff) * self._analogRef
    
    @response(contentType=M_JSON)
    def analogReadAll(self):
        values = {}
        for i in range(self._analogCount):
            values[i] = self.analogRead(i)
        return values
            
    @response(contentType=M_JSON)
    def analogReadAllFloat(self):
        values = {}
        for i in range(self._analogCount):
            values[i] = float("%.2f" % self.analogReadFloat(i))
        return values
    
    @response(contentType=M_JSON)
    def analogReadAllVolt(self):
        values = {}
        for i in range(self._analogCount):
            values[i] = float("%.2f" % self.analogReadVolt(i))
        return values

    def read(self, channel, value_type=None, diff=False):
        read_functions = {'float': self.analogReadFloat, 'volt': self.analogReadVolt}
        read_function = read_functions.get(value_type, self.analogRead)
        return read_function(channel, diff)
    
    def readFloat(self, channel, diff=False):
        return self.analogReadFloat(channel, diff)

    def readVolt(self, channel, diff=False):
        return self.analogReadVolt(channel, diff)

         
class DAC(ADC):
    def __init__(self, channelCount, resolution, vref):
        ADC.__init__(self, channelCount, resolution, vref)
    
    def __family__(self):
        return "DAC"
    
    def __analogWrite__(self, channel, value):
        raise NotImplementedError
    
    @response("%d")    
    def analogWrite(self, channel, value):
        self.checkAnalogChannel(channel)
        self.checkAnalogValue(value)
        self.__analogWrite__(channel, value)
        return self.analogRead(channel)
    
    @response("%.2f")    
    def analogWriteFloat(self, channel, value):
        self.analogWrite(channel, int(value * self._analogMax))
        return self.analogReadFloat(channel)
    
    @response("%.2f")    
    def analogWriteVolt(self, channel, value):
        self.analogWriteFloat(channel, value /self._analogRef)
        return self.analogReadVolt(channel)
    
    def write(self, channel, value, value_type=None):
        write_functions = {'float': self.analogWriteFloat, 'volt': self.analogWriteVolt}
        write_function = write_functions.get(value_type, self.analogWrite)        
        return write_function(channel, value)

    def writeFloat(self, channel, value):
        return self.analogWriteFloat(channel, value)

    def writeVolt(self, channel, value):
        return self.analogWriteVolt(channel, value)


class PWM():
    def __init__(self, channelCount, resolution, frequency):
        self._pwmCount = channelCount
        self._pwmResolution = resolution
        self._pwmMax = 2**resolution - 1
        self.frequency = frequency
        self.period = 1.0/frequency
        
        # Futaba servos standard
        self.servo_neutral = 0.00152
        self.servo_travel_time = 0.0004
        self.servo_travel_angle = 45.0
        
        self.reverse = [False for i in range(channelCount)]
         
    def __family__(self):
        return "PWM"

    def checkPWMChannel(self, channel):
        if not 0 <= channel < self._pwmCount:
            raise ValueError("Channel %d out of range [%d..%d]" % (channel, 0, self._pwmCount-1))

    def checkPWMValue(self, value):
        if not 0 <= value <= self._pwmMax:
            raise ValueError("Value %d out of range [%d..%d]" % (value, 0, self._pwmMax))
    
    def __pwmRead__(self, channel):
        raise NotImplementedError
    
    def __pwmWrite__(self, channel, value):
        raise NotImplementedError
    
    @response("%d")
    def pwmCount(self):
        return self._pwmCount

    @response("%d")
    def pwmResolution(self):
        return self._pwmResolution
    
    @response("%d")
    def pwmMaximum(self):
        return int(self._pwmMax)
    
    @response("%d")
    def pwmRead(self, channel):
        self.checkPWMChannel(channel)
        return self.__pwmRead__(channel)
    
    @response("%.2f")
    def pwmReadFloat(self, channel):
        return self.pwmRead(channel) / float(self._pwmMax)
    
    @response("%d")    
    def pwmWrite(self, channel, value):
        self.checkPWMChannel(channel)
        self.checkPWMValue(value)
        self.__pwmWrite__(channel, value)
        return self.pwmRead(channel)
    
    @response("%.2f")    
    def pwmWriteFloat(self, channel, value):
        self.pwmWrite(channel, int(value * self._pwmMax))
        return self.pwmReadFloat(channel)

    def read(self, channel, value_type=None):
        read_functions = {'float': self.pwmReadFloat, 'angle': self.pwmReadAngle}
        read_function = read_functions.get(value_type, self.pwmRead)
        return read_function(channel)        

    def readFloat(self, channel):
        return self.pwmReadFloat(channel)

    def write(self, channel, value, value_type=None):
        write_functions = {'float': self.pwmWriteFloat, 'angle': self.pwmWriteAngle}
        write_function = write_functions.get(value_type, self.pwmWrite)
        return write_function(channel, value)

    def writeFloat(self, channel, value):
        return self.pwmWriteFloat(channel, value)

    def getReverse(self, channel):
        self.checkChannel(channel)
        return self.reverse[channel]
    
    def setReverse(self, channel, value):
        self.checkChannel(channel)
        self.reverse[channel] = value
        return value
    
    def RatioToAngle(self, value):
        f = value
        f *= self.period
        f -= self.servo_neutral
        f *= self.servo_travel_angle
        f /= self.servo_travel_time
        return f

    def AngleToRatio(self, value):
        f = value
        f *= self.servo_travel_time
        f /= self.servo_travel_angle
        f += self.servo_neutral
        f /= self.period
        return f
    
    @response("%.2f")
    def pwmReadAngle(self, channel):
        f = self.pwmReadFloat(channel)
        f = self.RatioToAngle(f)
        if self.reverse[channel]:
            f = -f
        else:
            f = f
        return f
        
    @response("%.2f")
    def pwmWriteAngle(self, channel, value):
        if self.reverse[channel]:
            f = -value
        else:
            f = value
        f = self.AngleToRatio(f)
        self.pwmWriteFloat(channel, f)
        return self.pwmReadAngle(channel)

    @response(contentType=M_JSON)
    def pwmWildcard(self):
        values = {}
        for i in range(self._pwmCount):
            val = self.pwmReadFloat(i)
            values[i] = {}
            values[i]["float"] = float("%.2f" % val)
            values[i]["angle"] = float("%.2f" % self.RatioToAngle(val))
        return values

    def readAngle(self, channel):
        return self.pwmReadAngle(channel)

    def writeAngle(self, channel, value):
        return self.pwmWriteAngle(channel, value)
    
      
    
DRIVERS = {}
DRIVERS["helper"]  = ["AnalogSensor", "AnalogActuator", "ServoMotor", "Thermistor", "Photoresistor", "LoadSensor", "DistanceSensor", "LightDimmer"]
DRIVERS["ads1x1x"] = ["ADS1014", "ADS1015", "ADS1114", "ADS1115"]
DRIVERS["mcp4725"] = ["MCP4725"]
DRIVERS["mcp48XX"] = ["MCP4802", "MCP4812", "MCP4822"]
DRIVERS["mcp492X"] = ["MCP4921", "MCP4922"]
DRIVERS["pcf8591"] = ["PCF8591"]
