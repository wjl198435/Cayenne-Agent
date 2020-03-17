from myDevices.devices.analog.helper import AnalogSensor


import time
import datetime
import math
from myDevices.decorators.rest import request, response
from traceback import extract_stack
from myDevices.devices import instance
from myDevices.utils.logger import debug, info,setInfo,error

class MQ136Result:
    """DHT11 sensor result returned by DHT11.read() method"""

    ERR_NO_ERROR = 0
    ERR_MISSING_DATA = 1
    ERR_CRC = 2

    error_code = ERR_NO_ERROR
    h2s = -1
    co = -1
    ch4 = -1
    uptime = 0
    unit = "{}ppm {}ppm {}ppm"

    def __init__(self, error_code, h2s, co,ch4,time):
        self.error_code = error_code
        self.h2s = h2s
        self.co = co
        self.ch4 = ch4
        self.uptime = time

    def is_valid(self):
        return self.error_code == MQ136Result.ERR_NO_ERROR

class MQSensor():

    ######################### Hardware Related Macros #########################
    MQ_PIN                       = 0        # define which analog input channel you are going to use (MCP3008)
    RL_VALUE                     = 5        # define the load resistance on the board, in kilo ohms
    RO_CLEAN_AIR_FACTOR          = 9.83     # RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
    # which is derived from the chart in datasheet

    ######################### Software Related Macros #########################
    CALIBARAION_SAMPLE_TIMES     = 50       # define how many samples you are going to take in the calibration phase
    CALIBRATION_SAMPLE_INTERVAL  = 500      # define the time interal(in milisecond) between each samples in the
    # cablibration phase
    READ_SAMPLE_INTERVAL         = 50       # define how many samples you are going to take in normal operation
    READ_SAMPLE_TIMES            = 5        # define the time interal(in milisecond) between each samples in
    # normal operation

    ######################### Application Related Macros ######################
    GAS_H2S                      = 0
    GAS_CO                       = 1
    GAS_CH4                      = 2

    def __init__(self, adc, channel):
        self.adcname = adc
        self.adc = None
        self.setADCInstance()
        self.mq_channel = channel
        # self.LPGCurve = [2.3,0.21,-0.47]    # two points are taken from the curve.
        self.H2SCurve = [0.7, 0.78, -0.72]  # ( Log10(0.5) -  log10(6) ) / ( log10(100) - log10(5) )
        # with these two points, a line is formed which is "approximately equivalent"
        # to the original curve.
        # data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
        self.COCurve = [0.7,0.95,-0.37]     # two points are taken from the curve.
        # with these two points, a line is formed which is "approximately equivalent"
        # to the original curve.
        # data format:[ x, y, slope]; point1: (lg200, 0.72), point2: (lg10000,  0.15)
        self.CH4Curve =[0.7,0.90,-0.75]   # two points are taken from the curve.
        # with these two points, a line is formed which is "approximately equivalent"
        # to the original curve.
        # data format:[ x, y, slope]; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
        self.first = True


    def setADCInstance(self):
        if not self.adc:
            self.adc = instance.deviceInstance(self.adcname)


    def __family__(self):
        return "MQSensor"

    def __str__(self):
        return "MQSensor"

    # @response("%d")
    def readMQ(self):

        if self.first:
            info("Calibrating...")
            self.Ro = self.MQCalibration(self.mq_channel)
            info("Calibration is done...\n")
            info("Ro=%f kohm" % self.Ro)
            self.first = False

        try:
            read = self.MQRead(self.mq_channel)
            info("read mq_channel:{}".format(read))
            measure_time = datetime.datetime.now().timestamp()
            h2s = self.MQGetGasPercentage(read / self.Ro, self.GAS_H2S)
            # co = self.MQGetGasPercentage(read/self.Ro, self.GAS_CO)
            # ch4 = self.MQGetGasPercentage(read / self.Ro, self.GAS_CH4)
            # return MQ136Result(MQ136Result.ERR_NO_ERROR, h2s, co, ch4, measure_time)
            info(" h2s ={}".format(h2s))
            return h2s
        except Exception as e:
            error(e)
            # return MQ136Result(MQ136Result.ERR_MISSING_DATA, -1, -1, -1,measure_time)
            return -1

    def MQPercentage(self):
        val = {}
        read = self.MQRead(self.mq_channel)
        val["GAS_H2S"]  = self.MQGetGasPercentage(read / self.Ro, self.GAS_H2S)
        val["CO"]       = self.MQGetGasPercentage(read/self.Ro, self.GAS_CO)
        val["GAS_CH4"]    = self.MQGetGasPercentage(read / self.Ro, self.GAS_CH4)
        return val

    ######################### MQResistanceCalculation #########################
    # Input:   raw_adc - raw value read from adc, which represents the voltage
    # Output:  the calculated sensor resistance
    # Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
    #          across the load resistor and its resistance, the resistance of the sensor
    #          could be derived.
    ############################################################################
    def MQResistanceCalculation(self, raw_adc):
        return float(self.RL_VALUE*(1023.0-raw_adc)/float(raw_adc));


    ######################### MQCalibration ####################################
    # Input:   mq_pin - analog channel
    # Output:  Ro of the sensor
    # Remarks: This function assumes that the sensor is in clean air. It use
    #          MQResistanceCalculation to calculates the sensor resistance in clean air
    #          and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
    #          10, which differs slightly between different sensors.
    ############################################################################
    def MQCalibration(self, mq_pin):
        val = 0.0
        for i in range(self.CALIBARAION_SAMPLE_TIMES):          # take multiple samples
            val += self.MQResistanceCalculation(self.adc.read(mq_pin))
            time.sleep(self.CALIBRATION_SAMPLE_INTERVAL/1000.0)

        val = val/self.CALIBARAION_SAMPLE_TIMES                 # calculate the average value

        val = val/self.RO_CLEAN_AIR_FACTOR                      # divided by RO_CLEAN_AIR_FACTOR yields the Ro
        # according to the chart in the datasheet

        return val


    #########################  MQRead ##########################################
    # Input:   mq_pin - analog channel
    # Output:  Rs of the sensor
    # Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
    #          The Rs changes as the sensor is in the different consentration of the target
    #          gas. The sample times and the time interval between samples could be configured
    #          by changing the definition of the macros.
    ############################################################################
    def MQRead(self, mq_pin):
        rs = 0.0

        for i in range(self.READ_SAMPLE_TIMES):
            rs += self.MQResistanceCalculation(self.adc.read(mq_pin))
            time.sleep(self.READ_SAMPLE_INTERVAL/1000.0)

        rs = rs/self.READ_SAMPLE_TIMES

        return rs

    #########################  MQGetGasPercentage ##############################
    # Input:   rs_ro_ratio - Rs divided by Ro
    #          gas_id      - target gas type
    # Output:  ppm of the target gas
    # Remarks: This function passes different curves to the MQGetPercentage function which
    #          calculates the ppm (parts per million) of the target gas.
    ############################################################################
    def MQGetGasPercentage(self, rs_ro_ratio, gas_id):
        if ( gas_id == self.GAS_H2S):
            return self.MQGetPercentage(rs_ro_ratio, self.H2SCurve)
        elif ( gas_id == self.GAS_CO ):
            return self.MQGetPercentage(rs_ro_ratio, self.COCurve)
        elif ( gas_id == self.GAS_CH4):
            return self.MQGetPercentage(rs_ro_ratio, self.CH4Curve)
        return 0

    #########################  MQGetPercentage #################################
    # Input:   rs_ro_ratio - Rs divided by Ro
    #          pcurve      - pointer to the curve of the target gas
    # Output:  ppm of the target gas
    # Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
    #          of the line could be derived if y(rs_ro_ratio) is provided. As it is a
    #          logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
    #          value.
    ############################################################################
    def MQGetPercentage(self, rs_ro_ratio, pcurve):
        return round((math.pow(10,( ((math.log(rs_ro_ratio)-pcurve[1])/ pcurve[2]) + pcurve[0]))),3)