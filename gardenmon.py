#!/usr/bin/python3 -u

from abc import ABC, abstractmethod
import csv
import datetime
import glob
import local_options
import logging
import mysql.connector
import os
from smbus2 import SMBus
import sys
import time

def c_to_f(c: float) -> float:
    """
    Celcius to Farenheit.
    """
    return (c * 1.8) + 32

class Sensor(ABC):
    """
    Base class for all sensors.
    """

    @abstractmethod
    def read(self):
        """
        Read sensor data.
        """
        pass

    def get_value_or_none(self):
        """
        Attempt to read and return the value from a sensor, or return None if the
        read fails.
        """
        try:
            return self.read()
        except:
            logging.exception(f"Failure to read '{type(self).__name__}' sensor")
            return None

class CpuTemp(Sensor):
    """
    Sensor class for the temperature sensor for the RPi CPU.
    """

    def __init__(self):
        self.cpu_temp_file = "/sys/class/thermal/thermal_zone0/temp"

    def read(self) -> float:
        with open(self.cpu_temp_file) as cpu_temp_file:
            val = c_to_f(int(cpu_temp_file.read()) / 1000.0)
        return round(val, 2)

class ATHS(Sensor):
    """
    Ambient Temperature/Humidity Sensor. Underlying sensor is the SHT30
    temperature and humidity sensor. Connected via I2C.
    """

    def __init__(self):
        self.i2cbus = SMBus(1)

        # addr bit is pulled to ground.
        self.i2caddr = 0x44

        # Set the sensor for high repeatability and 10 measurements per
        # second. Kinda overkill, but we aren't on battery power.
        self.i2cbus.write_byte_data(self.i2caddr, 0x27, 0x37)

        self.temperature_trim = 1.0
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
        vals["temperature"] = round(temperature_f, 2)
        vals["humidity"] = round(humidity_percent, 1)
        return vals

class STS(Sensor):
    """
    Soil Temperature Sensor. Underlying sensor is the DS18B20 temperature
    sensor. Connected via 1 wire.
    """

    def __init__(self):
        # Device will appear at "/sys/bus/w1/devices/28-xxxxxxxxxxxx/w1_slave".
        base_dir = '/sys/bus/w1/devices/'
        device_folder = glob.glob(base_dir + '28*')[0]
        self.device_file = device_folder + '/w1_slave'
        self.temperature_trim = 0

    def read(self) -> float:
        with open(self.device_file, 'r') as device_file:
            lines = device_file.readlines()

        if "YES" not in lines[0] and "t=" not in lines[1]:
            raise RuntimeError("Invalid reading from Soil Temperature Reading")

        # The end of the second line has "t=X", where X is the temperature
        # reading in Celsius * 1000.
        temperature_string = lines[1][lines[1].find("t=") + 2:]
        temperature_f = c_to_f(float(temperature_string) / 1000.0)
        temperature_f = temperature_f + self.temperature_trim
        return round(temperature_f, 2)

class SMS(Sensor):
    """
    Soil Moisture Sensor. Underlying sensor is a soil moisture probe with
    the output fed into an MCP3221 ADC. Connected via I2C.
    """

    def __init__(self):
        self.i2cbus = SMBus(1)

        # Address of the A5 variant of the MCP3221.
        self.i2caddr = 0x4d

        self.value_trim = 0

        # Measured low (dry) and high (wet) points.
        self.low_value = 200
        self.high_value = 1100
        self.levels = 10

    def read(self) -> int:
        # Read fake "register" 0x00, get back 2 bytes:
        #   0 : MSB of ADC reading
        #   1 : LSB of ADC reading
        data = self.i2cbus.read_i2c_block_data(self.i2caddr, 0x00, 2)
        val = data[0] << 8 | data[1]

        val += self.value_trim

        return val

    def value_to_level(self, value: int) -> int:
        if value < self.low_value:
            return 1
        elif value > self.high_value:
            return self.levels
        else:
            # Rely on int() rounding to floor() the calculated level.
            value_per_step = (self.high_value - self.low_value)/(self.levels - 2)
            return int((value - self.low_value)/value_per_step) + 2

class ALS(Sensor):
    """
    Ambient Light Sensor. Underlying sensor is probably a BH1750. Connected
    via I2C.
    """

    def __init__(self):
        self.i2cbus = SMBus(1)
        self.i2caddr = 0x23

        self.lux_trim = 0.0

    def read(self) -> float:
        # From register 0x10, sensor readings are 2 bytes:
        #   0 : MSB of lux reading
        #   1 : LSB of lux reading
        data = self.i2cbus.read_i2c_block_data(self.i2caddr, 0x10, 2)
        val = data[0] << 8 | data[1]
        lux = float(val)/1.2 + self.lux_trim
        return round(lux, 1)

def gardenmon_main():
    logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')

    log_folder = '/var/log/gardenmon'
    if not os.path.exists(log_folder):
        os.makedirs(log_folder)

    cpu_temp_sensor = CpuTemp()
    aths_sensor = ATHS()
    sts_sensor = STS()
    sms_sensor = SMS()
    als_sensor = ALS()

    time.sleep(1)

    logging.info("gardenmon starting...")

    while True:
        # Ensure we sample every minute, on the minute.
        sleeptime = 60 - datetime.datetime.now().second
        time.sleep(sleeptime)
        current_time = datetime.datetime.now()

        cpu_temp  = cpu_temp_sensor.get_value_or_none()
        aths_vals = aths_sensor.get_value_or_none()
        aths_temp = aths_vals["temperature"]
        aths_hmd  = aths_vals["humidity"]
        sts_temp  = sts_sensor.get_value_or_none()
        sms_val   = sms_sensor.get_value_or_none()
        sms_level = sms_sensor.value_to_level(sms_val)
        als_lux   = als_sensor.get_value_or_none()

        row = [current_time.strftime('%Y-%m-%d %H:%M:%S')]
        row.extend(["CPU Temperature",     cpu_temp,  "F"])
        row.extend(["Ambient Temperature", aths_temp, "F"])
        row.extend(["Ambient Humidity",    aths_hmd,  "%"])
        row.extend(["Soil Temperature",    sts_temp,  "F"])
        row.extend(["Soil Moisture Value", sms_val,   "decimal_value"])
        row.extend(["Soil Moisture Level", sms_level, "decimal_value"])
        row.extend(["Ambient Light",       als_lux,   "lx"])

        with open(f"{log_folder}/main.csv", "a") as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow(row)

        with open(f"{log_folder}/{current_time.date()}.csv", "a") as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow(row)

        # TODO: sms levels
        data = (
            cpu_temp,
            als_lux,
            sms_val,
            sms_level,
            sts_temp,
            aths_temp,
            aths_hmd,
            current_time
        )

        try:
            connection = mysql.connector.connect(
                host=local_options.database_host,
                database=local_options.database_name,
                user=local_options.database_user,
                password=local_options.database_password
            )

            INSERT_STATEMENT = (
                f"INSERT INTO {local_options.database_table} "
                "(cpu_temp_f, ambient_light_lx, soil_moisture_val, soil_moisture_level, "
                "soil_temp_f, ambient_temp_f, ambient_humidity, insert_time) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s, %s)"
            )

            cursor = connection.cursor()
            cursor.execute(INSERT_STATEMENT, data)
            connection.commit()
        except mysql.connector.Error:
            logging.exception(f"Could not insert data to database.")
        finally:
            if connection and connection.is_connected():
                cursor.close()
                connection.close()

if __name__ == "__main__":
    gardenmon_main()
