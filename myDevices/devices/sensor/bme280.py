"""BME280 Driver."""
from i2cdevice import Device, Register, BitField, _int_to_bytes
from i2cdevice.adapter import LookupAdapter, Adapter
import struct
import time
from myDevices.utils.logger import debug, info, error,setInfo
try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

from myDevices.devices.sensor import Temperature, Pressure,Humidity

__version__ = '0.0.2'

CHIP_ID = 0x60
I2C_ADDRESS_GND = 0x76
I2C_ADDRESS_VCC = 0x77


class S8Adapter(Adapter):
    """Convert unsigned 8bit integer to signed."""

    def _decode(self, value):
        if value & (1 << 7):
            value -= 1 << 8
        return value


class S16Adapter(Adapter):
    """Convert unsigned 16bit integer to signed."""

    def _decode(self, value):
        return struct.unpack('<h', _int_to_bytes(value, 2))[0]


class U16Adapter(Adapter):
    """Convert from bytes to an unsigned 16bit integer."""

    def _decode(self, value):
        return struct.unpack('<H', _int_to_bytes(value, 2))[0]


class H5Adapter(S16Adapter):
    def _decode(self, value):
        b = _int_to_bytes(value, 2)
        r = ((b[0] >> 4) & 0x0F) | (b[1] << 4)
        if r & (1 << 11):
            r = r - 1 << 12
        return r


class H4Adapter(S16Adapter):
    def _decode(self, value):
        b = _int_to_bytes(value, 2)
        r = (b[0] << 4) | (b[1] & 0x0F)
        if r & (1 << 11):
            r = r - 1 << 12
        return r


class BME280Calibration():
    def __init__(self):
        self.dig_t1 = 0
        self.dig_t2 = 0
        self.dig_t3 = 0

        self.dig_p1 = 0
        self.dig_p2 = 0
        self.dig_p3 = 0
        self.dig_p4 = 0
        self.dig_p5 = 0
        self.dig_p6 = 0
        self.dig_p7 = 0
        self.dig_p8 = 0
        self.dig_p9 = 0

        self.dig_h1 = 0
        self.dig_h2 = 0
        self.dig_h3 = 0
        self.dig_h4 = 0
        self.dig_h5 = 0
        self.dig_h6 = 0

        self.temperature_fine = 0

    def set_from_namedtuple(self, value):
        # Iterate through a tuple supplied by i2cdevice
        # and copy its values into the class attributes
        for key in self.__dict__.keys():
            try:
                setattr(self, key, getattr(value, key))
            except AttributeError:
                pass

    def compensate_temperature(self, raw_temperature):
        var1 = (raw_temperature / 16384.0 - self.dig_t1 / 1024.0) * self.dig_t2
        var2 = raw_temperature / 131072.0 - self.dig_t1 / 8192.0
        var2 = var2 * var2 * self.dig_t3
        self.temperature_fine = (var1 + var2)
        return self.temperature_fine / 5120.0

    def compensate_pressure(self, raw_pressure):
        var1 = self.temperature_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_p6 / 32768.0
        var2 = var2 + var1 * self.dig_p5 * 2
        var2 = var2 / 4.0 + self.dig_p4 * 65536.0
        var1 = (self.dig_p3 * var1 * var1 / 524288.0 + self.dig_p2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_p1
        pressure = 1048576.0 - raw_pressure
        pressure = (pressure - var2 / 4096.0) * 6250.0 / var1
        var1 = self.dig_p9 * pressure * pressure / 2147483648.0
        var2 = pressure * self.dig_p8 / 32768.0
        return pressure + (var1 + var2 + self.dig_p7) / 16.0

    def compensate_humidity(self, raw_humidity):
        humidity = self.temperature_fine - 76800.0
        humidity = (raw_humidity - (self.dig_h4 * 64.0 + self.dig_h5 / 16384.0 * humidity)) * (self.dig_h2 / 65536.0 * (1.0 + self.dig_h6 / 67108864.0 * humidity * (1.0 + self.dig_h3 / 67108864.0 * humidity)))
        humidity = humidity * (1.0 - self.dig_h1 * humidity / 524288.0)
        return max(0.0, min(100.0, humidity))


class BME280(Temperature, Pressure,Humidity):


    def __init__(self, i2c_addr=I2C_ADDRESS_GND, i2c_dev=None,altitude=0, external=None, humidity=False, temperature=True, pressure=False):

        self.temp = temperature
        self.hum = humidity
        self.press = pressure


        if self.press:
            Pressure.__init__(self, altitude, external)

        self.calibration = BME280Calibration()
        self._is_setup = False
        self._i2c_addr = i2c_addr
        self._i2c_dev = i2c_dev
        self._bme280 = Device([I2C_ADDRESS_GND, I2C_ADDRESS_VCC], i2c_dev=self._i2c_dev, bit_width=8, registers=(
            Register('CHIP_ID', 0xD0, fields=(
                BitField('id', 0xFF),
            )),
            Register('RESET', 0xE0, fields=(
                BitField('reset', 0xFF),
            )),
            Register('STATUS', 0xF3, fields=(
                BitField('measuring', 0b00001000),  # 1 when conversion is running
                BitField('im_update', 0b00000001),  # 1 when NVM data is being copied
            )),
            Register('CTRL_MEAS', 0xF4, fields=(
                BitField('osrs_t', 0b11100000,   # Temperature oversampling
                         adapter=LookupAdapter({
                             1: 0b001,
                             2: 0b010,
                             4: 0b011,
                             8: 0b100,
                             16: 0b101
                         })),
                BitField('osrs_p', 0b00011100,   # Pressure oversampling
                         adapter=LookupAdapter({
                             1: 0b001,
                             2: 0b010,
                             4: 0b011,
                             8: 0b100,
                             16: 0b101})),
                BitField('mode', 0b00000011,     # Power mode
                         adapter=LookupAdapter({
                             'sleep': 0b00,
                             'forced': 0b10,
                             'normal': 0b11})),
            )),
            Register('CTRL_HUM', 0xF2, fields=(
                BitField('osrs_h', 0b00000111,   # Humidity oversampling
                         adapter=LookupAdapter({
                             1: 0b001,
                             2: 0b010,
                             4: 0b011,
                             8: 0b100,
                             16: 0b101})),
            )),
            Register('CONFIG', 0xF5, fields=(
                BitField('t_sb', 0b11100000,     # Temp standby duration in normal mode
                         adapter=LookupAdapter({
                             0.5: 0b000,
                             62.5: 0b001,
                             125: 0b010,
                             250: 0b011,
                             500: 0b100,
                             1000: 0b101,
                             10: 0b110,
                             20: 0b111})),
                BitField('filter', 0b00011100),                   # Controls the time constant of the IIR filter
                BitField('spi3w_en', 0b0000001, read_only=True),  # Enable 3-wire SPI interface when set to 1. IE: Don't set this bit!
            )),
            Register('DATA', 0xF7, fields=(
                BitField('humidity', 0x000000000000FFFF),
                BitField('temperature', 0x000000FFFFF00000),
                BitField('pressure', 0xFFFFF00000000000)
            ), bit_width=8 * 8),
            Register('CALIBRATION', 0x88, fields=(
                BitField('dig_t1', 0xFFFF << 16 * 12, adapter=U16Adapter()),  # 0x88 0x89
                BitField('dig_t2', 0xFFFF << 16 * 11, adapter=S16Adapter()),  # 0x8A 0x8B
                BitField('dig_t3', 0xFFFF << 16 * 10, adapter=S16Adapter()),  # 0x8C 0x8D
                BitField('dig_p1', 0xFFFF << 16 * 9, adapter=U16Adapter()),   # 0x8E 0x8F
                BitField('dig_p2', 0xFFFF << 16 * 8, adapter=S16Adapter()),   # 0x90 0x91
                BitField('dig_p3', 0xFFFF << 16 * 7, adapter=S16Adapter()),   # 0x92 0x93
                BitField('dig_p4', 0xFFFF << 16 * 6, adapter=S16Adapter()),   # 0x94 0x95
                BitField('dig_p5', 0xFFFF << 16 * 5, adapter=S16Adapter()),   # 0x96 0x97
                BitField('dig_p6', 0xFFFF << 16 * 4, adapter=S16Adapter()),   # 0x98 0x99
                BitField('dig_p7', 0xFFFF << 16 * 3, adapter=S16Adapter()),   # 0x9A 0x9B
                BitField('dig_p8', 0xFFFF << 16 * 2, adapter=S16Adapter()),   # 0x9C 0x9D
                BitField('dig_p9', 0xFFFF << 16 * 1, adapter=S16Adapter()),   # 0x9E 0x9F
                BitField('dig_h1', 0x00FF),                                   # 0xA1 uint8
            ), bit_width=26 * 8),
            Register('CALIBRATION2', 0xE1, fields=(
                BitField('dig_h2', 0xFFFF0000000000, adapter=S16Adapter()),   # 0xE1 0xE2
                BitField('dig_h3', 0x0000FF00000000),                         # 0xE3 uint8
                BitField('dig_h4', 0x000000FFFF0000, adapter=H4Adapter()),    # 0xE4 0xE5[3:0]
                BitField('dig_h5', 0x00000000FFFF00, adapter=H5Adapter()),    # 0xE5[7:4] 0xE6
                BitField('dig_h6', 0x000000000000FF, adapter=S8Adapter())     # 0xE7 int8
            ), bit_width=7 * 8)
        ))

    def __str__(self):
            return "BMP280"

    def __family__(self):
        family = []
        if self.temp:
            family.append(Temperature.__family__(self))
        if self.press:
            family.append(Pressure.__family__(self))
        if self.hum:
            family.append(Humidity.__family__(self))
        return family

    def setup(self, mode='normal', temperature_oversampling=16, pressure_oversampling=16, humidity_oversampling=16, temperature_standby=500):
        if self._is_setup:
            return
        self._is_setup = True

        self._bme280.select_address(self._i2c_addr)
        self._mode = mode

        if mode == "forced":
            mode = "sleep"

        try:
            chip = self._bme280.get('CHIP_ID')
            if chip.id != CHIP_ID:
                raise RuntimeError("Unable to find bme280 on 0x{:02x}, CHIP_ID returned {:02x}".format(self._i2c_addr, chip.id))
        except IOError:
            raise RuntimeError("Unable to find bme280 on 0x{:02x}, IOError".format(self._i2c_addr))

        self._bme280.set('RESET', reset=0xB6)
        time.sleep(0.1)

        self._bme280.set('CTRL_HUM', osrs_h=humidity_oversampling)

        self._bme280.set('CTRL_MEAS',
                         mode=mode,
                         osrs_t=temperature_oversampling,
                         osrs_p=pressure_oversampling)

        self._bme280.set('CONFIG',
                         t_sb=temperature_standby,
                         filter=2)

        self.calibration.set_from_namedtuple(self._bme280.get('CALIBRATION'))
        self.calibration.set_from_namedtuple(self._bme280.get('CALIBRATION2'))

    def update_sensor(self):
        self.setup()

        if self._mode == "forced":
            self._bme280.set('CTRL_MEAS', mode="forced")
            while self._bme280.get('STATUS').measuring:
                time.sleep(0.001)

        raw = self._bme280.get('DATA')

        # info("update_sensor raw.temperature={} raw.pressure={} raw.humidity={} ".format(raw.temperature,raw.pressure,raw.humidity))
        if self.temp:
            self._temperature = self.calibration.compensate_temperature(raw.temperature)
        if self.press:
            self._pressure = self.calibration.compensate_pressure(raw.pressure) / 100.0
        if self.hum:
            self._humidity = self.calibration.compensate_humidity(raw.humidity)

    def get_temperature(self):
        self.update_sensor()
        return self._temperature

    def get_pressure(self):
        self.update_sensor()
        return self._pressure

    def get_humidity(self):
        self.update_sensor()
        return self._humidity

    def get_altitude(self, qnh=1013.25):
        self.update_sensor()
        pressure = self.get_pressure()
        altitude = 44330.0 * (1.0 - pow(pressure / qnh, (1.0 / 5.255)))
        return altitude

    def __getCelsius__(self):
        return self.get_temperature()

    def __getKelvin__(self):
            return self.Celsius2Kelvin()

    def __getFahrenheit__(self):
        return self.Celsius2Fahrenheit()

    def __getPascal__(self):
        return self.get_pressure()

    def __getHumidity__(self):
        return self.get_humidity()/100.0

    def __getPascalAtSea__(self):
        pressure = self.__getPascal__()
        if self.external != None:
            k = self.external.getKelvin()
            if k != 0:
                return float(pressure) / (1.0 / (1.0 + 0.0065 / k * self.altitude)**5.255)
        return float(pressure) / (1.0 - self.altitude / 44330.0)**5.255


if __name__ == "__main__":
    bus = SMBus(1)
    bme280 = BME280(i2c_dev=bus,humidity=True, temperature=True, pressure=True)

    while True:
        temperature = bme280.get_temperature()
        pressure = bme280.get_pressure()
        humidity = bme280.get_humidity()
        print('{}*C {}hPa {}'.format(temperature, pressure, humidity))
        # print('{:05.2f}*C {:05.2f}hPa {:05.2f}%'.format(temperature, pressure, humidity))
        time.sleep(1)

# from smbus2 import SMBus
# import numpy as np
#
# ADDR_CTRL_MEAS = 0xF4
# ADDR_CONFIG = 0xF5
# ADDR_STATUS = 0xF3
# ADDR_RESET = 0xE0
# ADDR_ID = 0xD0
#
# class bmp280:
#     def __init__(self, addr = 0x76):
#         self.bus = SMBus(1)
#         self.address = addr
#         self.osrs_t = {0: 0b000, 1: 0b001, 2: 0b010, 4: 0b011, 8: 0b100, 16: 0b101}
#         self.osrs_p = {0: 0b000, 1: 0b001, 2: 0b010, 4: 0b011, 8: 0b100, 16: 0b101}
#         self.IIR_filter = {0: 0b000, 2: 0b001, 4: 0b010, 8: 0b011, 16: 0b100}
#         self.t_sb = {0.5: 0b000, 62.5: 0b001, 125: 0b010, 250: 0b011, 500: 0b100, 1000: 0b101, 2000: 0b110, 4000: 0b111}
#         self.mode = {"normal": 0b11, "forced": 0b01, "sleep": 0b00}
#         self.T = 0
#         self.p = 0
#         self.h = 300
#         self.p0 = 1013.25
#         self.defaultConfiguration()
#         self.readCalibrationData()
#
#     def readMeasureBlock(self):
#         start_addr = 0xF7
#         stop_addr = 0xFC
#         read_len = stop_addr-start_addr+1
#         return self.bus.read_i2c_block_data(self.address, start_addr, read_len)
#
#     def readCalibrationData(self):
#         start_addr = 0x88
#         stop_addr = 0x9F
#         read_len = stop_addr-start_addr+1
#         cal_data = self.bus.read_i2c_block_data(self.address, start_addr, read_len)
#         self.dig_T1 = np.uint16(cal_data[0] | cal_data[1] << 8)
#         self.dig_T2 = np.int16(cal_data[2] | cal_data[3] << 8)
#         self.dig_T3 = np.int16(cal_data[4] | cal_data[5] << 8)
#         self.dig_P1 = np.uint16(cal_data[6] | cal_data[7] << 8)
#         self.dig_P2 = np.int16(cal_data[8] | cal_data[9] << 8)
#         self.dig_P3 = np.int16(cal_data[10] | cal_data[11] << 8)
#         self.dig_P4 = np.int16(cal_data[12] | cal_data[13] << 8)
#         self.dig_P5 = np.int16(cal_data[14] | cal_data[15] << 8)
#         self.dig_P6 = np.int16(cal_data[16] | cal_data[17] << 8)
#         self.dig_P7 = np.int16(cal_data[18] | cal_data[19] << 8)
#         self.dig_P8 = np.int16(cal_data[20] | cal_data[21] << 8)
#         self.dig_P9 = np.int16(cal_data[22] | cal_data[23] << 8)
#         self.dig_Tx = [self.dig_T1, self.dig_T2, self.dig_T3]
#         self.dig_Px = [self.dig_P1, self.dig_P2, self.dig_P3, self.dig_P4, self.dig_P5, self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9]
#
#
#     def writeControlReg(self, oversample_t, oversample_p, measure_mode):
#         val = 0
#         val |= self.osrs_t[oversample_t]<<5
#         val |= self.osrs_p[oversample_p]<<2
#         val |= self.mode[measure_mode]
#         self.writeByte(ADDR_CTRL_MEAS, val)
#
#     def readControlReg(self):
#         return self.readByte(ADDR_CTRL_MEAS)
#
#     def writeConfigReg(self, time_standby, iir_filter_length):
#         val = 0
#         val |= self.t_sb[time_standby]<<5
#         val |= self.IIR_filter[iir_filter_length]<<2
#         self.writeByte(ADDR_CONFIG, val)
#
#     def defaultConfiguration(self):
#         self.writeControlReg(2, 2, "normal")
#         self.writeConfigReg(2000, 2)
#
#     def getMeasurement(self):
#         raw_data = self.readMeasureBlock()
#         up = np.int32(((raw_data[0]<<12) | raw_data[1]<<4) | raw_data[2]>>4)
#         ut = np.int32(((raw_data[3]<<12) | raw_data[4]<<4) | raw_data[5]>>4)
#         T, t_fine = self.tempCompensation(ut)
#         p = self.pressureCompensation(up, t_fine)
#         self.updateValues(T, p/100)
#         return round(T, 2), round(p/100, 1)
#
#     def seaLevelPressure(self):
#         return self.p*(1-(0.0065*self.h/(self.T+0.0065*self.h+273.15)))**(-5.257)
#
#     def altitude(self):
#         return ((self.p0/self.p)**(1/5.257)-1)*(self.T+273.15)/0.0065
#
#     def updateValues(self, T, p):
#         self.T = T
#         self.p = p
#
#     def tempCompensation(self, ut):
#         var1 = (ut/16384 - self.dig_T1/1024)*self.dig_T2
#         var2 = ((ut/131072 - self.dig_T1/8192)*(ut/131072 - self.dig_T1/8192))*self.dig_T3
#         t_fine = var1+var2
#         T = (var1+var2)/5120
#         return T, t_fine
#
#     def pressureCompensation(self, up, t_fine):
#         var1 = t_fine/2 - 64000
#         var2 = var1*var1*self.dig_P6/32768
#         var2 = var2 + var1*self.dig_P5*2
#         var2 = var2/4 + self.dig_P4*65536
#         var1 = (self.dig_P3*var1*var1/524288 + self.dig_P2*var1)/524288
#         var1 = (1 + var1/32768)*self.dig_P1
#         p = 1048576 - up
#         if (var1 == 0): return 0
#         p = (p - var2/4096)*6250/var1
#         var1 = self.dig_P9*p*p/2147483648
#         var2 = p*self.dig_P8/32768
#         p = p + (var1 + var2 + self.dig_P7)/16
#         return p
#
#     def readConfigReg(self):
#         return self.readByte(ADDR_CONFIG)
#
#     def reset(self):
#         self.writeByte(ADDR_RESET, 0xB6)
#
#     def readID(self):
#         return self.readByte(ADDR_ID)
#
#     def readSatus(self):
#         return self.readByte(ADDR_STATUS)
#
#     def writeByte(self, reg_addr, val):
#         self.bus.write_byte_data(self.address, reg_addr, val)
#
#     def readByte(self, reg_addr):
#         return self.bus.read_byte_data(self.address, reg_addr)