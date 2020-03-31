import unittest
import os
import grp
from myDevices.sensors import sensors
from myDevices.devices import manager
from myDevices.utils.config import Config
from myDevices.utils import types
from myDevices.utils.logger import exception, setDebug, info, debug, error, logToFile, setInfo
from myDevices.devices.bus import checkAllBus, BUSLIST
from myDevices.devices.digital.gpio import NativeGPIO as GPIO
from myDevices.devices import instance
from time import sleep
from json import loads, dumps
from myDevices.cloud.client import CloudServerClient

from myDevices.utils.config import Config,APP_SETTINGS


class SensorsClientTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        client = CloudServerClient(host='192.168.8.107', port='1883', cayenneApiHost='192.168.8.107')
        cls.client = sensors.SensorsClient(client)

    @classmethod
    def tearDownClass(cls):
        cls.client.StopMonitoring()
        del cls.client

    def OnDataChanged(self, sensor_data):
        # if len(sensor_data) < 5:
        #     info('OnDataChanged: {}'.format(sensor_data))
        # else:
        #     info('OnDataChanged: {}'.format(len(sensor_data)))
        self.previousSystemData = self.currentSystemData
        self.currentSystemData = sensor_data

        if self.previousSystemData:
            self.done = True

    def testMonitor(self):
        debug('testMonitor')
        self.previousSystemData = None
        self.currentSystemData = None
        self.done = False
        SensorsClientTest.client.SetDataChanged(self.OnDataChanged)
        for i in range(35):
            sleep(1)
            if self.done:
                break
        info('Changed items: {}'.format([x for x in self.currentSystemData if x not in self.previousSystemData]))
        self.assertNotEqual(self.previousSystemData, self.currentSystemData)

    def testBusInfo(self):
        debug('testBusInfo')
        bus = {item['channel']:item['value'] for item in SensorsClientTest.client.BusInfo()}
        info('Bus info: {}'.format(bus))
        for pin in GPIO().pins:
            self.assertIn('sys:gpio:{};function'.format(pin), bus)
            self.assertIn('sys:gpio:{};value'.format(pin), bus)

    def testSensorsInfo(self):
        debug('testSensorsInfo')
        sensors = SensorsClientTest.client.SensorsInfo()
        info('Sensors info: {}'.format(sensors))
        for sensor in sensors:
            self.assertEqual('dev:', sensor['channel'][:4])
            self.assertIn('value', sensor)

    def testSetFunction(self):
        debug('testSetFunciton')
        self.setChannelFunction(GPIO().pins[7], 'IN')
        self.setChannelFunction(GPIO().pins[7], 'OUT')

    def testSetValue(self):
        debug('testSetValue')
        self.setChannelFunction(GPIO().pins[7], 'OUT')
        self.setChannelValue(GPIO().pins[7], 1)
        self.setChannelValue(GPIO().pins[7], 0)

    def testSensors(self):
        debug('testSensors')
        #Test adding a sensor
        channel = GPIO().pins[8]
        testSensor = {'description': 'Digital Input', 'device': 'DigitalSensor', 'args': {'gpio': 'GPIO', 'invert': False, 'channel': channel}, 'name': 'testdevice'}
        SensorsClientTest.client.RemoveSensor(testSensor['name']) #Attempt to remove device if it already exists from a previous test
        compareKeys = ('args', 'description', 'device')
        retValue = SensorsClientTest.client.AddSensor(testSensor['name'], testSensor['description'], testSensor['device'], testSensor['args'])
        self.assertTrue(retValue)
        retrievedSensor = next(obj for obj in manager.getDeviceList() if obj['name'] == testSensor['name'])
        for key in compareKeys:
            self.assertEqual(testSensor[key], retrievedSensor[key])
        #Test updating a sensor
        editedSensor = testSensor
        editedSensor['args']['channel'] = GPIO().pins[5]
        retValue = SensorsClientTest.client.EditSensor(editedSensor['name'], editedSensor['description'], editedSensor['device'], editedSensor['args'])
        self.assertTrue(retValue)
        retrievedSensor = next(obj for obj in manager.getDeviceList() if obj['name'] == editedSensor['name'])
        for key in compareKeys:
            self.assertEqual(editedSensor[key], retrievedSensor[key])
        #Test removing a sensor
        retValue = SensorsClientTest.client.RemoveSensor(testSensor['name'])
        self.assertTrue(retValue)
        deviceNames = [device['name'] for device in manager.getDeviceList()]
        self.assertNotIn(testSensor['name'], deviceNames)

    def testSensorInfo(self):
        debug('testSensorInfo')

        self.config = Config(APP_SETTINGS)
        self.location = self.config.get('Agent', 'Location', "house0_room0_")

        actuator_channel = GPIO().pins[10]
        light_switch_channel = GPIO().pins[11]

        adcSensors = {

        }

        for sensor in adcSensors.values():
            info('--------{} {} {}'.format(sensor['name'], sensor['description'], sensor['device']))
            SensorsClientTest.client.AddSensor(sensor['name'],sensor['description'], sensor['device'], sensor['args'])

        sensors = {
                   # 'actuator' : {'description': 'Digital Output', 'device': 'DigitalActuator', 'args': {'gpio': 'GPIO', 'invert': False, 'channel': actuator_channel}, 'name': 'test_actuator'},
                   # 'light_switch' : {'description': 'Light Switch', 'device': 'LightSwitch', 'args': {'gpio': 'GPIO', 'invert': True, 'channel': light_switch_channel}, 'name': 'test_light_switch'},
                   # 'MCP3004' : {'description': 'MCP3004', 'device': 'MCP3004', 'args': {'chip': '0'}, 'name': 'test_MCP3004'},


                    'PCF8591' : {'description': 'PCF8591','index':0, 'device': 'PCF8591','args': {},  'name': 'adc'},
                    'distance' : {'description': 'distance', 'index':1 ,'device': 'VL6180X','args': {},  'name': self.location+'distance'},
                    'object_temperature' : {'description': 'ir_body_temperature', 'index':2, 'device': 'MLX90614','args': {'obj_temp': True},  'name': self.location+'ir_body_temp'},
                    'amb_temperature' : {'description': 'ir_climate_temperature', 'index':3,'device': 'MLX90614','args': {'obj_temp': False},  'name': self.location+'ir_climate_temp'},
                    'luminosity' : {'description': 'luminosity','index':4, 'device': 'GY30','args': {},  'name': self.location+'luminosity'},

                    'co2' : {'description': 'co2', 'index':5,'device': 'CO2Sensor','args': {'adc': 'adc', 'channel': 3},  'name': self.location+'co2'},
                    'h2s' : {'description': 'h2s',  'index':6,'device': 'MQSensor', 'args': {'adc': 'adc', 'channel': 2}, 'name': self.location+'h2s'},
                    'nh3' : {'description': 'nh3',  'index':6,'device': 'MQSensor', 'args': {'adc': 'adc', 'channel': 4}, 'name': self.location+'nh3'},
                    'climate' : {'description': 'climate','index':7, 'device': 'BME280','args': {'temperature':True,'pressure': True,'humidity': True},  'name': self.location+'climate'},
                    # 'climate_pressure' : {'description': 'climate_pressure','index':8, 'device': 'BME280','args': {'pressure': True},  'name': self.location+'climate_pressure'},
                    # 'climate_humidity' : {'description': 'climate_humidity','index':9, 'device': 'BME280','args': {'humidity': True},  'name': self.location+'climate_humidity'}

                  }
        for sensor in sensors.values():
            # info("sensors:{}".format(sensor))
            SensorsClientTest.client.RemoveSensor(sensor['name'])

        for sensor in sensors.values():
            info('--------{} {} {}'.format(sensor['name'], sensor['description'], sensor['device']))
            SensorsClientTest.client.AddSensor(sensor['name'],sensor['description'], sensor['device'], sensor['args'])
        # SensorsClientTest.client.SensorsInfo()
        #Test setting sensor values
        # self.setSensorValue(sensors['actuator'], 1)
        # self.setSensorValue(sensors['actuator'], 0)
        # self.setSensorValue(sensors['light_switch'], 1)
        # self.setSensorValue(sensors['light_switch'], 0)
        #Test getting analog value
        # channel = 'dev:{}'.format(sensors['MQ']['name'])
        # info(" channel -----> {} ".format(channel))
        count = 0
        while count < 1:
            info("new loop for SensorsInfo")
            sleep(5)
            for obj in SensorsClientTest.client.SensorsInfo():
                info(obj)
            count = count + 1


        # for obj in SensorsClientTest.client.SensorsInfo():
        #     info(obj)

        # for obj in SensorsClientTest.client.SensorsInfo():
        #     info(obj)

        # retrievedSensorInfo = next(obj for obj in SensorsClientTest.client.SensorsInfo() if obj['channel'] == channel)
        #
        # info(" retrievedSensorInfo -----> {} value={}".format(retrievedSensorInfo,retrievedSensorInfo['value']))
        #
        # self.assertGreaterEqual(retrievedSensorInfo['value'], 0.0)
        #
        # self.assertLessEqual(retrievedSensorInfo['value'], 1.0)
        # for sensor in sensors.values():
        #     info(sensor['name'])
        #     self.assertTrue(SensorsClientTest.client.RemoveSensor(sensor['name']))

    def testSensorCallback(self):
        debug('testSensorCallback')
        self.previousSystemData = None
        self.currentSystemData = None
        self.done = False
        SensorsClientTest.client.SetDataChanged(self.OnDataChanged)
        actuator_channel = GPIO().pins[10]
        sensor_channel = GPIO().pins[11]
        sensors = {'actuator' : {'description': 'Digital Output', 'device': 'DigitalActuator', 'args': {'gpio': 'GPIO', 'invert': False, 'channel': actuator_channel}, 'name': 'test_actuator'},
            'sensor': {'description': 'Digital Input', 'device': 'DigitalSensor', 'args': {'gpio': 'GPIO', 'invert': False, 'channel': sensor_channel}, 'name': 'testdevice'}}
        for sensor in sensors.values():
            SensorsClientTest.client.AddSensor(sensor['name'], sensor['description'], sensor['device'], sensor['args'])
        for i in range(35):
            sleep(1)
            if self.done:
                break
        info('Changed items: {}'.format([x for x in self.currentSystemData if x not in self.previousSystemData]))
        self.assertNotEqual(self.previousSystemData, self.currentSystemData)
        for sensor in sensors.values():
            self.assertTrue(SensorsClientTest.client.RemoveSensor(sensor['name']))

    def setSensorValue(self, sensor, value):
        SensorsClientTest.client.SensorCommand('integer', sensor['name'], None, value)
        channel = 'dev:{}'.format(sensor['name'])
        sensorInfo = next(obj for obj in SensorsClientTest.client.SensorsInfo() if obj['channel'] == channel)
        self.assertEqual(value, sensorInfo['value'])

    def setChannelFunction(self, channel, function):
        SensorsClientTest.client.GpioCommand('function', channel, function)
        bus = {item['channel']:item['value'] for item in SensorsClientTest.client.BusInfo()}
        self.assertEqual(function, bus['sys:gpio:{};function'.format(channel)])

    def setChannelValue(self, channel, value):
        SensorsClientTest.client.GpioCommand('value', channel, value)
        bus = {item['channel']:item['value'] for item in SensorsClientTest.client.BusInfo()}
        self.assertEqual(value, bus['sys:gpio:{};value'.format(channel)])

if __name__ == '__main__':
    setInfo()
    # setDebug()
    # unittest.main()
    suite = unittest.TestSuite()
    suite.addTest(SensorsClientTest('testSensorInfo')) # 将需要执行的case添加到Test Suite中，没有添加的不会被执行
    # suite.addTest(TestCase_01('testSecond_01'))
    # suite.addTest(TestCase_01('testFirst_01'))

    unittest.TextTestRunner().run(suite) # 将根据case添加的先后顺序执行

