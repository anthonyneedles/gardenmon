#!/usr/bin/python3 -u

from abc import ABC, abstractmethod
import csv
import datetime
import glob
import logging
import mysql.connector
import os
import smbus
import sys
import time

INSERT_STATEMENT = (
    "INSERT INTO environmental_data "
    "(cpu_temp_f, ambient_light_lx, soil_moisture_val, soil_moisture_level, "
    "soil_temp_f, ambient_temp_f, ambient_humidity, insert_time) "
    "VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
)

def c_to_f(c: float) -> float:
    """
    Celcius to Farenheit.
    """
    return (c * 1.8) + 32

class sensor(ABC):
    """
    Base class for all sensors.
    """

    @abstractmethod
    def read(self):
        """
        Read sensor data.
        """
        pass

class cpu_temp(sensor):
    """
    Sensor class for the temperature sensor for the RPi CPU.
    """

    def __init__(self):
        self.cpu_temp_file = "/sys/class/thermal/thermal_zone0/temp"

    def read(self) -> float:
        with open(self.cpu_temp_file) as cpu_temp_file:
            val = c_to_f(int(cpu_temp_file.read()) / 1000.0)
        return val

    def get_value(self) -> float:
        try:
            return self.read()
        except:
            logging.exception(f"Failure to read CPU temp sensor")
            return 99999.9


class aths(sensor):
    """
    Ambient Temperature/Humidity Sensor. Underlying sensor is the SHT30
    temperature and humidity sensor. Connected via I2C.
    """

    def __init__(self):
        self.i2cbus = smbus.SMBus(1)

        # addr bit is pulled to ground.
        self.i2caddr = 0x44

        # Set the sensor for high repeatability and 10 measurements per
        # second. Kinda overkill, but we aren't on battery power.
        self.i2cbus.write_byte_data(self.i2caddr, 0x27, 0x37)

        self.temperature_trim = 0.0
        self.humidity_trim = 0.0

    def read(self) -> dict:
        # Sensor readings are 6 bytes:
        #   0 : MSB of temp reading
        #   1 : LSB of temp reading
        #   2 : CRC of temp reading (ignored)
        #   3 : MSB of humidity reading
        #   4 : LSB of humidity reading
        #   5 : CRC of humidity reading (ignored)
        data = self.i2cbus.read_i2c_block_data(self.i2caddr, 0x00, 6)
        temperature_raw = data[0] << 8 | data[1]
        humidity_raw    = data[3] << 8 | data[4]

        # Apply conversion formulas to raw values.
        temperature_f    = ((temperature_raw * 315.0) / 0xFFFF) - 49 + self.temperature_trim
        humidity_percent = ((humidity_raw    * 100.0) / 0xFFFF) + self.humidity_trim

        vals = dict();
        vals["temperature"] = temperature_f
        vals["humidity"] = humidity_percent
        return vals

    def get_value(self) -> dict:
        try:
            return self.read()
        except:
            logging.exception(f"Failure to read Ambient Temperature/Humidity Sensor")
            return { "temperature": 9999.9, "humidity": 9999.9 }

class sts(sensor):
    """
    Soil Temperature Sensor. Underlying sensor is the DS18B20 temperature
    sensor. Connected via 1 wire.
    """

    def __init__(self):
        # Device will appear at "/sys/bus/w1/devices/28-xxxxxxxxxxxx/w1_slave".
        base_dir = '/sys/bus/w1/devices/'
        device_folder = glob.glob(base_dir + '28*')[0]
        self.device_file = device_folder + '/w1_slave'

        self.temperature_trim = -2.2

    def read(self) -> float:
        with open(self.device_file, 'r') as device_file:
            lines = device_file.readlines()

        if "YES" not in lines[0] and "t=" not in lines[1]:
            raise RuntimeError("Invalid reading from Soil Temperature Reading")

        # The end of the second line has "t=X", where X is the temperature
        # reading in Celsius * 1000.
        temperature_string = lines[1][lines[1].find("t=") + 2:]
        temperature_f = c_to_f(float(temperature_string) / 1000.0)
        return temperature_f + self.temperature_trim

    def get_value(self) -> float:
        try:
            # If the sensor is disconnected from the 1wire connection to the
            # board this will read 0, without any way to really detect the
            # error condition.
            return self.read()
        except:
            logging.exception(f"Failure to read Soil Temperature Sensor")
            return 99999.9

class sms(sensor):
    """
    Soil Moisture Sensor. Underlying sensor is a soil moisture probe with
    the output fed into an MCP3221 ADC. Connected via I2C.
    """

    def __init__(self):
        self.i2cbus = smbus.SMBus(1)

        # Address of the A5 variant of the MCP3221.
        self.i2caddr = 0x4d

        self.value_trim = 0

    def read(self) -> int:
        # Read fake "register" 0x00, get back 2 bytes:
        #   0 : MSB of ADC reading
        #   1 : LSB of ADC reading
        data = self.i2cbus.read_i2c_block_data(self.i2caddr, 0x00, 2)
        val = data[0] << 8 | data[1]

        val += self.value_trim

        return val

    def get_value(self) -> int:
        try:
            return self.read()
        except:
            logging.exception(f"Failure to read Soil Moisture Sensor")
            return 99999

class als(sensor):
    """
    Ambient Light Sensor. Underlying sensor is probably a BH1750. Connected
    via I2C.
    """

    def __init__(self):
        self.i2cbus = smbus.SMBus(1)
        self.i2caddr = 0x23

        self.lux_trim = 0.0

    def read(self) -> float:
        # From register 0x10, sensor readings are 2 bytes:
        #   0 : MSB of lux reading
        #   1 : LSB of lux reading
        data = self.i2cbus.read_i2c_block_data(self.i2caddr, 0x10, 2)
        val = data[0] << 8 | data[1]
        lux = float(val)/1.2 + self.lux_trim
        return lux

    def get_value(self) -> float:
        try:
            return self.read()
        except:
            logging.exception(f"Failure to read Ambient Light Sensor")
            return 99999.9

def gardenmon_main():
    logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')

    log_folder = '/var/log/gardenmon'
    if not os.path.exists(log_folder):
        os.makedirs(log_folder)

    cpu_temp_sensor = cpu_temp()
    aths_sensor = aths()
    sts_sensor = sts()
    sms_sensor = sms()
    als_sensor = als()

    time.sleep(1)

    logging.info("gardenmon starting...")

    while True:
        current_time = datetime.datetime.now()

        timestamp = current_time.strftime('%Y-%m-%d %H:%M:%S')
        row = [timestamp]

        cpu_temp_val = cpu_temp_sensor.get_value()
        row.extend(["CPU Temperature", f"{cpu_temp_val:0.1f}", "F"])

        aths_vals = aths_sensor.get_value()
        row.extend(["Ambient Temperature", f"{aths_vals['temperature']:0.1f}", "F"])
        row.extend(["Ambient Humidity",    f"{aths_vals['humidity']:0.1f}",    "%"])

        sts_temperature = sts_sensor.get_value()
        row.extend(["Soil Temperature", f"{sts_temperature:0.1f}", "F"])

        sms_val = sms_sensor.get_value()
        row.extend(["Soil Moisture Value", f"{sms_val}", "decimal_value"])

        als_val = als_sensor.get_value()
        row.extend(["Ambient Light", f"{als_val:0.1f}", "lx"])

        with open(f"{log_folder}/main.csv", "a") as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow(row)

        with open(f"{log_folder}/{current_time.date()}.csv", "a") as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow(row)

        data = (cpu_temp_val, als_val, sms_val, 5,
                sts_temperature, aths_vals['temperature'], aths_vals['humidity'], current_time)
        try:
            connection = mysql.connector.connect(
                            host=host,
                            database=database,
                            user='gardenmon',
                            password=sys.argv[1],
            )
    
            cursor = connection.cursor()
            cursor.execute(insert_stmt, data)
            connection.commit()        
        except mysql.connector.Error as e:
            logging.info(f"An error occurred: {e}")
        finally:
            if connection and connection.is_connected():
                cursor.close()
                connection.close()

        time.sleep(1)

if __name__ == "__main__":
    gardenmon_main()
