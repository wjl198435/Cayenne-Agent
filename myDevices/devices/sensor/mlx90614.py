import time
from myDevices.utils.types import signInteger
import smbus

from myDevices.devices.sensor import Temperature
from time import  sleep
import datetime
from myDevices.utils.logger import debug, info, error,setInfo
class TemperatureResult:
    """DHT11 sensor result returned by DHT11.read() method"""

    ERR_NO_ERROR = 0
    ERR_MISSING_DATA = 1
    ERR_CRC = 2

    error_code = ERR_NO_ERROR
    amb_temp = -1
    obj_temp = -1
    uptime = 0
    unit = "{}℃ {}℃"

    def __init__(self, error_code, amb_temp, obj_temp,time):
        self.error_code = error_code
        self.amb_temp = amb_temp
        self.obj_temp = obj_temp
        self.uptime = time


    def is_valid(self):
        return self.error_code == TemperatureResult.ERR_NO_ERROR

class MLX90614(Temperature):
    MLX90614_RAWIR1=0x04
    MLX90614_RAWIR2=0x05
    MLX90614_TA=0x06
    MLX90614_TOBJ1=0x07
    MLX90614_TOBJ2=0x08

    MLX90614_TOMAX=0x20
    MLX90614_TOMIN=0x21
    MLX90614_PWMCTRL=0x22
    MLX90614_TARANGE=0x23
    MLX90614_EMISS=0x24
    MLX90614_CONFIG=0x25
    MLX90614_ADDR=0x0E
    MLX90614_ID1=0x3C
    MLX90614_ID2=0x3D
    MLX90614_ID3=0x3E
    MLX90614_ID4=0x3F

    comm_retries = 1
    comm_sleep_amount = 0.1
    address = 0x5a

    def __init__(self,temperature=True,obj_temperature=True):
        self.temperature = temperature
        self.obj_temperature = obj_temperature
        # I2C.__init__(self, 0x5a, True)
        # Temperature.__init__(self)

        self.bus = smbus.SMBus(bus=1)

    def __str__(self):
            return "MLX90614"

    def __family__(self):
        family = []
        if self.temperature:
            family.append(Temperature.__family__(self))

        return family

    def read_reg(self, reg_addr):
        for i in range(self.comm_retries):
            try:
                return self.bus.read_word_data(self.address, reg_addr)
                # print(reg_addr)
                # return  self.readRegisters(reg_addr,2)
            except IOError as e:
                # "Rate limiting" - sleeping to prevent problems with sensor
                # when requesting data too quickly
                sleep(self.comm_sleep_amount)
                # By this time, we made a couple requests and the sensor didn't respond
                # (judging by the fact we haven't returned from this function yet)
                # So let's just re-raise the last IOError we got
                raise e

    def __getCelsius__(self):
        return self.get_obj_temp()

    def data_to_temp(self, data):
        temp = (data*0.02) - 273.15
        return temp

    def get_amb_temp(self):
        data = self.read_reg(self.MLX90614_TA)
        info("data={}".format(data))
        return round(self.data_to_temp(data),2)

    def get_obj_temp(self):
        data = self.read_reg(self.MLX90614_TOBJ1)
        return round(self.data_to_temp(data),2)
    def read(self):
        measure_time = datetime.datetime.now().timestamp()
        try:

            amb_temp = self.get_amb_temp()
            obj_temp = self.get_obj_temp()
            return TemperatureResult(TemperatureResult.ERR_NO_ERROR,amb_temp,obj_temp,measure_time)
        except Exception as e:
            error(e)
            return TemperatureResult(TemperatureResult.ERR_MISSING_DATA,-1,-1,measure_time)


if __name__ == "__main__":
    setInfo()
    mlx = mlx90614()
    result = mlx.read()
    info(result.unit.format(result.amb_temp,result.obj_temp))