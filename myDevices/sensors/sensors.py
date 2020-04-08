"""
This module provides a class for interfacing with sensors and actuators. It can add, edit and remove
sensors and actuators as well as monitor their states and execute commands.
"""
from datetime import datetime, timedelta
from json import dumps, loads
from os import getpid, path
from threading import Event, RLock

from myDevices.cloud import cayennemqtt
from myDevices.cloud.dbmanager import DbManager
from myDevices.cloud.download_speed import DownloadSpeed
from myDevices.devices import instance, manager
from myDevices.devices.bus import BUSLIST, checkAllBus
from myDevices.devices.digital.gpio import NativeGPIO as GPIO
from myDevices.system import services
from myDevices.system.systeminfo import SystemInfo
from myDevices.plugins.manager import PluginManager
from myDevices.utils.config import Config, APP_SETTINGS
from myDevices.utils.daemon import Daemon
from myDevices.utils.logger import debug, error, exception, info, logJson, warn
from myDevices.utils.threadpool import ThreadPool
from myDevices.utils.types import M_JSON
# from myDevices.system.hardware import Hardware

from myDevices.devices.manager import DYNAMIC_DEVICES

REFRESH_FREQUENCY = 15 #seconds
REAL_TIME_FREQUENCY = 60/55 #Seconds/messages, this is done to keep messages under the rate limit



sensors = {
    'PCF8591' : {'description': 'PCF8591','index':0, 'device': 'PCF8591','args': {},  'name': 'adc'},
    'distance' : {'description': 'distance', 'index':1 ,'device': 'VL6180X','args': {},  'name': 'distance'},
    'object_temperature' : {'description': 'ir_temperature', 'index':2, 'device': 'MLX90614','args': {'obj_temp': True},  'name': 'ir_body'},
    'amb_temperature' : {'description': 'climate_temperature', 'index':3,'device': 'MLX90614','args': {'obj_temp': False},  'name': 'ir_reference'},
    'illuminance' : {'description': 'illuminance','index':4, 'device': 'GY30','args': {},  'name': 'illuminance'},
    'co2' : {'description': 'co2', 'index':5,'device': 'CO2Sensor','args': {'adc': 'adc', 'channel': 3},  'name': 'co2'},
    'h2s' : {'description': 'h2s',  'index':6,'device': 'MQSensor', 'args': {'adc': 'adc', 'channel': 2}, 'name': 'h2s'},
    'nh3' : {'description': 'nh3',  'index':6,'device': 'MQSensor', 'args': {'adc': 'adc', 'channel': 4}, 'name': 'nh3'},
    # 'bme280' : {'description': 'climate','index':7, 'device': 'BME280','args': {'temperature':True,'pressure': True,'humidity': True},  'name': 'temperature'},
    'temperature' : {'description': 'temperature','index':7, 'device': 'BME280','args': {'temperature':True},  'name': 'climate_temperature'},
    'pressure' : {'description': 'pressure','index':7, 'device': 'BME280','args': {'pressure':True},  'name': 'climate_pressure'},
    'humidity' : {'description': 'humidity','index':7, 'device': 'BME280','args': {'humidity':True},  'name': 'climate_humidity'}
}


class SensorsClient():
    """Class for interfacing with sensors and actuators"""

    def __init__(self,client):
        """Initialize the bus and sensor info and start monitoring sensor states"""
        self.cloudClient = client
        self.sensorMutex = RLock()
        self.realTimeMutex = RLock()
        self.exiting = Event()
        self.onDataChanged = None
        self.systemData = []
        self.currentSystemState = []
        self.currentRealTimeData = {}                                
        self.queuedRealTimeData = {}
        self.disabledSensors = {}
        self.disabledSensorTable = "disabled_sensors"
        checkAllBus()
        self.gpio = GPIO()
        # self.downloadSpeed = DownloadSpeed(Config(APP_SETTINGS))
        # self.downloadSpeed.getDownloadSpeed()
        manager.addDeviceInstance("GPIO", "GPIO", "GPIO", self.gpio, [], "system")

        manager.loadJsonDevices("rest")


        if not DYNAMIC_DEVICES  :
            warn("loadJsonDevices is None")
            for sensor in sensors.values():
                # info('--------{} {} {}'.format(sensor['name'], sensor['description'], sensor['device']))
                self.AddSensor(sensor['name'],sensor['description'], sensor['device'], sensor['args'])

        #
        # info(DYNAMIC_DEVICES)
        self.config = Config(APP_SETTINGS)
        self.clientId = self.config.get('Agent', 'ClientID', None)
        self.mqtt_dis_prefix = self.config.get('Agent', 'MQTT_DIS_PREFIX', "homeassistant")
        self.serial = self.cloudClient.hardware.Serial


        for name,device in DYNAMIC_DEVICES.items():

            for type in device['type']  :
                if type in ['DAC','ADC']:
                    continue
                topic,message = self.AddMQTTSensorDevice(name,type,device)

                if self.cloudClient:
                    info("{} {}".format(topic,message))
                    self.cloudClient.EnqueuePacket(message,topic)
                # info(mqttsensor)


        results = DbManager.Select(self.disabledSensorTable)
        if results:
            for row in results:
                self.disabledSensors[row[0]] = 1
        self.realTimeMonitorRunning = False
        self.pluginManager = PluginManager(self.OnPluginChange)
        self.pluginManager.load_plugins()
        self.InitCallbacks()
        self.StartMonitoring()


    def AddMQTTSensorDevice(self,name,type, device):

        sensor_topic = "sensor/{}/{}/config".format(self.serial,name )
        unit = {"Temperature": "℃", "Humidity":"%", "Pressure":"pa", "Distance":"mm", "MQSensor":"ppm", "Luminosity":"lu", "CO2Sensor":"ppm"}
        icon = {"Temperature": "mdi:coolant-temperature", "Humidity":"mdi:water", "Pressure":"mdi:file-powerpoint-box", "Distance":"mdi:ruler", "MQSensor":"mdi:gas-cylinder", "Luminosity":"mdi:white-balance-sunny", "CO2Sensor":"mdi:periodic-table-co2"}
        sensor_config = \
        {
                "device_class": "temperature",
                "name": "{}{}".format(self.cloudClient.location,name) ,
                "state_topic": "{}/sensor/{}/dev:{}/state".format(self.mqtt_dis_prefix, self.serial, name),
                "unit_of_measurement": unit[type],
                "icon": icon[type],
                "value_template": "{{ value_json.value}}"
        }

        return  sensor_topic,sensor_config


    def SetDataChanged(self, onDataChanged=None):
        """Set callback to call when data has changed
        
        Args:
            onDataChanged: Function to call when sensor data changes
        """
        self.onDataChanged = onDataChanged

    def QueueRealTimeData(self, name, data):
        """Add real-time data to queue to be sent on thread

        Args:
            name: The name to use for the data
            data: The data to send
        """
        with self.realTimeMutex:
            if name not in self.currentRealTimeData:
                self.currentRealTimeData[name] = data
            else:
                self.queuedRealTimeData[name] = data

    def OnSensorChange(self, device, value):
        """Callback that is called when digital sensor data has changed

        Args:
            device: The device that has changed data
            value: The new data value
        """
        debug('OnSensorChange: {}, {}'.format(device, value))
        with self.realTimeMutex:
            data = {'name': device['description'], 'value': value, 'type': 'digital_sensor', 'unit': 'd'}
            if 'args' in device:
                data['args'] = device['args']
            self.QueueRealTimeData(device['name'], data)

    def OnPluginChange(self, data):
        """Callback that is called when digital sensor data has changed

        Args:
            data: The new data value
        """
        debug('OnPluginChange: {}'.format(data))
        self.QueueRealTimeData(data['id'], data)
        with self.realTimeMutex:
            if not self.realTimeMonitorRunning:
                ThreadPool.Submit(self.RealTimeMonitor)

    def OnGpioStateChange(self, channel, value):
        """Send updated pin state when it has changed

        Args:
            channel: The pin number
            value: The new value for the pin
        """
        debug('OnGpioStateChange: channel {}, value {}'.format(channel, value))
        data = []
        cayennemqtt.DataChannel.add_unique(data, cayennemqtt.SYS_GPIO, channel, cayennemqtt.VALUE, value)
        if not self.realTimeMonitorRunning:
            self.onDataChanged(data)
        else:
            self.QueueRealTimeData(data[0]['channel'], data[0])

    def InitCallbacks(self):
        """Set callback function for any digital devices that support them"""
        devices = manager.getDeviceList()
        for device in devices:
            sensor = instance.deviceInstance(device['name'])
            if 'DigitalSensor' in device['type'] and hasattr(sensor, 'setCallback'):
                debug('Set callback for {}'.format(sensor))
                sensor.setCallback(self.OnSensorChange, device)
                if not self.realTimeMonitorRunning:
                    ThreadPool.Submit(self.RealTimeMonitor)

    def RemoveCallbacks(self):
        """Remove callback function for all digital devices"""
        devices = manager.getDeviceList()
        for device in devices:
            sensor = instance.deviceInstance(device['name'])
            if 'DigitalSensor' in device['type'] and hasattr(sensor, 'removeCallback'):
                sensor.removeCallback()

    def StartMonitoring(self):
        """Start thread monitoring sensor data"""
        # pass
        ThreadPool.Submit(self.Monitor)

    def StopMonitoring(self):
        """Stop thread monitoring sensor data"""
        self.RemoveCallbacks()
        self.exiting.set()

    def Monitor(self):
        """Monitor bus/sensor states and system info and report changed data via callbacks"""
        debug('Monitoring sensors and os resources started')
        sendAllDataCount = 0
        nextTime = datetime.now()
        while not self.exiting.is_set():
            try:
                difference = nextTime - datetime.now()
                delay = min(REFRESH_FREQUENCY, difference.total_seconds())
                delay = max(0, delay)
                if not self.exiting.wait(delay):
                    nextTime = datetime.now() + timedelta(seconds=REFRESH_FREQUENCY)
                    self.currentSystemState = []
                    self.MonitorSystemInformation()
                    self.MonitorSensors()
                    self.MonitorPlugins()
                    self.MonitorBus()
                    if self.currentSystemState != self.systemData:
                        data = self.currentSystemState
                        if self.systemData and not sendAllDataCount == 0:
                            data = [x for x in self.currentSystemState if x not in self.systemData]
                        if self.onDataChanged and data:
                            self.onDataChanged(data)
                    sendAllDataCount += 1
                    if sendAllDataCount >= 4:
                        sendAllDataCount = 0
                    self.systemData = self.currentSystemState
            except:
                exception('Monitoring sensors and os resources failed')
        debug('Monitoring sensors and os resources finished')

    def RealTimeMonitor(self):
        """Monitor real-time state changes and report changed data via callbacks"""
        self.realTimeMonitorRunning = True
        info('Monitoring real-time state changes')
        nextTime = datetime.now()
        while not self.exiting.is_set():
            try:
                if not self.exiting.wait(0.5):
                    if datetime.now() > nextTime:
                        nextTime = datetime.now() + timedelta(seconds=REAL_TIME_FREQUENCY)
                        self.SendRealTimeData()
            except:
                exception('Monitoring real-time changes failed')
        debug('Monitoring real-time changes finished')
        self.realTimeMonitorRunning = False

    def SendRealTimeData(self):
        """Send real-time data via callback"""
        data = []
        with self.realTimeMutex:
            if self.currentRealTimeData:
                for name, item in self.currentRealTimeData.items():
                    if cayennemqtt.SYS_GPIO in name:
                        data.append(item)
                    else: 
                        cayennemqtt.DataChannel.add_unique(data, cayennemqtt.DEV_SENSOR, name, value=item['value'], name=item['name'], type=item['type'], unit=item['unit'])
                        try:
                            cayennemqtt.DataChannel.add_unique(data, cayennemqtt.SYS_GPIO, item['args']['channel'], cayennemqtt.VALUE, item['value'])
                        except:
                            pass
                        if name in self.queuedRealTimeData and self.queuedRealTimeData[name]['value'] == item['value']:
                            del self.queuedRealTimeData[name]
                self.currentRealTimeData = self.queuedRealTimeData
                self.queuedRealTimeData = {}
        if data:
            self.onDataChanged(data)

    def MonitorSensors(self):
        """Check sensor states for changes"""
        if self.exiting.is_set():
            return
        self.currentSystemState += self.SensorsInfo()




    def MonitorPlugins(self):
        """Check plugin states for changes"""
        if self.exiting.is_set():
            return
        self.currentSystemState += self.pluginManager.get_plugin_readings()

    def MonitorBus(self):
        """Check bus states for changes"""
        if self.exiting.is_set():
            return
        self.currentSystemState += self.BusInfo()

    def MonitorSystemInformation(self):
        """Check system info for changes"""
        if self.exiting.is_set():
            return
        self.currentSystemState += self.SystemInformation()

    def SystemInformation(self):
        """Return dict containing current system info, including CPU, RAM, storage and network info"""
        newSystemInfo = []
        try:
            systemInfo = SystemInfo()
            newSystemInfo = systemInfo.getSystemInformation()
            # download_speed = self.downloadSpeed.getDownloadSpeed()
            # if download_speed:
            #     cayennemqtt.DataChannel.add(newSystemInfo, cayennemqtt.SYS_NET, suffix=cayennemqtt.SPEEDTEST, value=download_speed, type='bw', unit='mbps')
        except Exception:
            exception('SystemInformation failed')
        return newSystemInfo

    def CallDeviceFunction(self, func, *args):
        """Call a function for a sensor/actuator device and format the result value type

        Args:
            func: Function to call
            args: Parameters to pass to the function

        Returns:
            True for success, False otherwise.
        """
        result = func(*args)

        debug("result={}".format(result))

        if result != None:
            if hasattr(func, "contentType"):
                if func.contentType != M_JSON:
                    value_type = type(result)
                    response = value_type(func.format % result)
                else:
                    response = result
            else:
                response = result
        debug("response={}".format(response))
        return response

    def BusInfo(self):
        """Return a dict with current bus info"""
        bus_info = []
        gpio_state = self.gpio.wildcard()
        for key, value in gpio_state.items():
            cayennemqtt.DataChannel.add(bus_info, cayennemqtt.SYS_GPIO, key, cayennemqtt.VALUE, value['value'])
            cayennemqtt.DataChannel.add(bus_info, cayennemqtt.SYS_GPIO, key, cayennemqtt.FUNCTION, value['function'])
        return bus_info

    def SensorsInfo(self):
        """Return a list with current sensor states for all enabled sensors"""
        manager.deviceDetector()
        devices = manager.getDeviceList()
        sensors_info = []
        if devices is None:
            return sensors_info
        for device in devices:
            sensor = instance.deviceInstance(device['name'])
            if 'enabled' not in device or device['enabled'] == 1:
                sensor_types = {'Temperature': {'function': 'getCelsius', 'data_args': {'type': 'temperature', 'unit': 'c'}},
                                'Humidity': {'function': 'getHumidityPercent', 'data_args': {'type': 'humidity', 'unit': 'p'}},
                                'Pressure': {'function': 'getPascal', 'data_args': {'type': 'pressure', 'unit': 'pa'}},
                                'Luminosity': {'function': 'getLux', 'data_args': {'type': 'illuminance', 'unit': 'lux'}},
                                'Distance': {'function': 'getCentimeter', 'data_args': {'type': 'prox', 'unit': 'cm'}},
                                'MQSensor': {'function': 'getMQ', 'data_args': {'type': 'mq136','unit': 'ppm'}},
                                'CO2Sensor': {'function': 'readCO2', 'data_args': {'type': 'co2','unit': 'ppm'}},
                                'ServoMotor': {'function': 'readAngle', 'data_args': {'type': 'analog_actuator'}},
                                'DigitalSensor': {'function': 'read', 'data_args': {'type': 'digital_sensor', 'unit': 'd'}},
                                'DigitalActuator': {'function': 'read', 'data_args': {'type': 'digital_actuator', 'unit': 'd'}},
                                # 'AnalogSensor': {'function': 'readFloat', 'data_args': {'type': 'analog_sensor'}},
                                'AnalogSensor': {'function': 'readVolt', 'data_args': {'type': 'analog_sensor'}},
                                'AnalogActuator': {'function': 'readFloat', 'data_args': {'type': 'analog_actuator'}}}
                # extension_types = {'ADC': {'function': 'analogReadAllFloat'},
                #                     'DAC': {'function': 'analogReadAllFloat'},
                #                     'PWM': {'function': 'pwmWildcard'},
                #                     'GPIOPort': {'function': 'wildcard'}}
                for device_type in device['type']:
                    try:
                        display_name = device['description']

                    except:
                        display_name = None
                    # info("display_name:{}".format(display_name))
                    if device_type in sensor_types:
                        try:
                            sensor_type = sensor_types[device_type]
                            # info("sensor_type:{}".format(sensor_type))
                            func = getattr(sensor, sensor_type['function'])
                            # info("func:{}".format(func))

                            if len(device['type']) > 1:
                                channel = '{}:{}'.format(device['name'], device_type.lower())
                            else:
                                channel = device['name']
                            value = self.CallDeviceFunction(func)
                            # info("value={}".format(value))
                            cayennemqtt.DataChannel.add(sensors_info, cayennemqtt.DEV_SENSOR, channel, value=value, name=display_name, **sensor_type['data_args'])
                            if 'DigitalActuator' == device_type and value in (0, 1):
                                manager.updateDeviceState(device['name'], value)
                        except:
                            exception('Failed to get sensor data: {} {}'.format(device_type, device['name']))
                    # else:
                    #     try:
                    #         extension_type = extension_types[device_type]
                    #         func = getattr(sensor, extension_type['function'])
                    #         values = self.CallDeviceFunction(func)
                    #         for pin, value in values.items():
                    #             cayennemqtt.DataChannel.add(sensors_info, cayennemqtt.DEV_SENSOR, device['name'] + ':' + str(pin), cayennemqtt.VALUE, value, name=display_name)
                    #     except:
                    #         exception('Failed to get extension data: {} {}'.format(device_type, device['name']))
        # info('Sensors info: {}'.format(sensors_info))
        return sensors_info

    def AddSensor(self, name, description, device, args):
        """Add a new sensor/actuator
   
        Args:
            name: Name of sensor to add
            description: Sensor description
            device: Sensor device class
            args: Sensor specific args

        Returns:
            True for success, False otherwise.
            :param index:
        """
        info('AddSensor: {}, {}, {}, {}'.format(name, description, device, args))
        bVal = False
        try:
            sensorAdd = {}

            if name:
                sensorAdd['name'] = name
            if device:
                sensorAdd['device'] = device
            if args:
                sensorAdd['args'] = args
            if description:
                sensorAdd['description'] = description
            with self.sensorMutex:
                retValue = manager.addDeviceJSON(sensorAdd)
                self.InitCallbacks()
            info('Add device returned: {}'.format(retValue))
            if retValue[0] == 200:
                bVal = True
        except Exception:
            exception('Error adding sensor')
            bVal = False
        return bVal

    def EditSensor(self, name, description, device, args):
        """Edit an existing sensor/actuator
  
        Args:
            name: Name of sensor to edit
            description: New sensor description
            device: New sensor device class
            args: New sensor specific args

        Returns:
            True for success, False otherwise.
        """
        info('EditSensor: {}, {}, {}, {}'.format(name, description, device, args))
        bVal = False
        try:
            sensorEdit = {}
            name = name
            sensorEdit['name'] = name
            sensorEdit['device'] = device
            sensorEdit['description'] = description
            sensorEdit['args'] = args
            with self.sensorMutex:
                retValue = manager.updateDevice(name, sensorEdit)
                self.InitCallbacks()
            info('Edit device returned: {}'.format(retValue))
            if retValue[0] == 200:
                bVal = True
        except:
            exception("Edit sensor failed")
            bVal = False
        return bVal

    def RemoveSensor(self, name):
        """Remove an existing sensor/actuator

        Args:
            name: Name of sensor to remove

        Returns:
            True for success, False otherwise.
        """
        bVal = False
        try:
            if self.pluginManager.is_plugin(name):
                return self.pluginManager.disable(name)
            sensorRemove = name
            try:
                sensor = instance.deviceInstance(sensorRemove)
                if hasattr(sensor, 'removeCallback'):
                    sensor.removeCallback()
            except: 
                pass
            with self.sensorMutex:
                retValue = manager.removeDevice(sensorRemove)
            info('Remove device returned: {}'.format(retValue))
            if retValue[0] == 200:
                bVal = True
        except:
            exception("Remove sensor failed")
            bVal = False
        return bVal

    def EnableSensor(self, sensor, enable):
        """Enable a sensor/actuator

        Args:
            sensor: Hash composed from name and device class/type
            enable: 1 to enable, 0 to disable

        Returns:
            True for success, False otherwise.
        """
        info('Enable sensor: ' + str(sensor) + ' ' + str(enable))
        try:
            if sensor is None:
                return False
            if enable is None:
                return False
            with self.sensorMutex:
                if enable == 0:
                    #add item to the list
                    if sensor not in self.disabledSensors:
                        DbManager.Insert(self.disabledSensorTable, sensor)
                        self.disabledSensors[sensor] = 1
                else:
                    #remove item from the list
                    if sensor in self.disabledSensors:
                        DbManager.Delete(self.disabledSensorTable, sensor)
                        del self.disabledSensors[sensor]
                    #save list
        except Exception as ex:
            error('EnableSensor Failed with exception: '  + str(ex))
            return False
        return True

    def GpioCommand(self, command, channel, value):
        """Execute onboard GPIO command

        Args:
            command: Type of command to execute
            channel: GPIO pin
            value: Value to use for writing data

        Returns:
            String containing command specific return value on success, or 'failure' on failure
        """
        info('GpioCommand {}, channel {}, value {}'.format(command, channel, value))
        result = 'failure'
        if command == 'function':
            old_state = self.gpio.digitalRead(channel)
            if value.lower() in ('in', 'input'):
                result = str(self.gpio.setFunctionString(channel, 'in'))
            elif value.lower() in ('out', 'output'):
                result = str(self.gpio.setFunctionString(channel, 'out'))
            new_state = self.gpio.digitalRead(channel)
            if new_state != old_state:
                self.OnGpioStateChange(channel, new_state)
        elif command in ('value', ''):
            return self.gpio.digitalWrite(channel, int(value))
        debug('GPIO command failed')
        return result

    def SensorCommand(self, command, sensorId, channel, value):
        """Execute sensor/actuator command

        Args:
            command: Type of command to execute
            sensorId: Sensor id
            channel: Pin/channel on device, None if there is no channel
            value: Value to use for setting the sensor state

        Returns:
            Command specific return value on success, False on failure
        """
        result = False
        info('SensorCommand: {}, sensor {}, channel {}, value {}'.format(command, sensorId, channel, value))
        try:
            if self.pluginManager.is_plugin(sensorId, channel):
                return self.pluginManager.write_value(sensorId, channel, value)
            commands = {'integer': {'function': 'write', 'value_type': int},
                        'value': {'function': 'write', 'value_type': int},
                        'function': {'function': 'setFunctionString', 'value_type': str},
                        'angle': {'function': 'writeAngle', 'value_type': float},
                        'float': {'function': 'writeFloat', 'value_type': float},
                        'volt': {'function': 'writeVolt', 'value_type': float}}
            with self.sensorMutex:
                if sensorId in self.disabledSensors:
                    info('Sensor disabled')
                    return result
                sensor = instance.deviceInstance(sensorId)
                if not sensor:
                    info('Sensor not found')
                    return result
                if command in commands:
                    device = instance.DEVICES[sensorId]
                    info('Sensor found: {}'.format(device))
                    func = getattr(sensor, commands[command]['function'])
                    value = commands[command]['value_type'](value)
                    if channel:
                        result = self.CallDeviceFunction(func, int(channel), value)
                    else:
                        result = self.CallDeviceFunction(func, value)
                    if 'DigitalActuator' in device['type']:
                        manager.updateDeviceState(sensorId, value)
                    return result
                warn('Command not implemented: {}'.format(command))
                return result
        except Exception:
            exception('SensorCommand failed')
        return result
