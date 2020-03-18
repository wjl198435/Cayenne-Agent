json_array = [{"name": "MQ136", "description": "Analog MQ136 Sensor", "device": "MQSensor", "type": ["MQSensor"], "status": 1, "args": {"adc": "test_PCF8591", "channel": 1}, "origin": "rest", "install_date": 1584514581}, {"name": "test_BME280", "description": "Temperature", "device": "BME280", "type": ["Temperature", "Pressure", "Humidity"], "status": 1, "args": {"pressure": True, "humidity": True}, "origin": "rest", "install_date": 1584514581}, {"name": "test_CO2Sensor", "description": "CO2", "device": "CO2Sensor", "type": ["CO2Sensor"], "status": 1, "args": {"adc": "test_PCF8591", "channel": 2}, "origin": "rest", "install_date": 1584514580}, {"name": "test_GY30", "description": "Luminosity", "device": "GY30", "type": ["Luminosity"], "status": 1, "args": {}, "origin": "rest", "install_date": 1584514580}, {"name": "test_MLX90614_amb", "description": "Temperature", "device": "MLX90614", "type": ["Temperature"], "status": 1, "args": {"obj_temp": False}, "origin": "rest", "install_date": 1584514580}, {"name": "test_MLX90614_obj", "description": "Temperature", "device": "MLX90614", "type": ["Temperature"], "status": 1, "args": {"obj_temp": True}, "origin": "rest", "install_date": 1584514580}, {"name": "test_PCF8591", "description": "PCF8591", "device": "PCF8591", "type": ["DAC", "ADC"], "status": 1, "args": {}, "origin": "rest", "install_date": 1584514580}, {"name": "test_vl6180x_distance", "description": "Distance", "device": "VL6180X", "type": ["Distance"], "status": 1, "args": {}, "origin": "rest", "install_date": 1584514580}]
json_array.sort(key = lambda x:x["install_date"])
print(json_array)