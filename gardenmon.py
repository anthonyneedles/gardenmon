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
from socket import gethostname
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

        self.temperature_trim = local_options.aths_temperature_trim
        self.humidity_trim = local_options.aths_temperature_trim

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
        self.temperature_trim = local_options.sts_temperature_trim

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
    Soil Moisture Sensor. Underlying sensor is a capacitive moisture probe with
    the output fed into an MCP3221 ADC. Connected via I2C.

    The relation between raw value and moisture level is inversely proportional.
    """

    def __init__(self):
        self.i2cbus = SMBus(1)

        # Address of the A5 variant of the MCP3221.
        self.i2caddr = 0x4d

        self.value_trim = local_options.sms_value_trim

        # Measured driest (in the air) and wettest (in a glass of water) points.
        self.driest_value = local_options.sms_driest_value
        self.wettest_value = local_options.sms_wettest_value
        self.levels = 10

        # Keep a 24 hour averaging window to smooth out fluctuations due to
        # daily temperature cycles.
        self.data_history = []
        self.data_history_maxlen = 60*24

    def read(self) -> int:
        # Read fake "register" 0x00, get back 2 bytes:
        #   0 : MSB of ADC reading
        #   1 : LSB of ADC reading
        data = self.i2cbus.read_i2c_block_data(self.i2caddr, 0x00, 2)
        val = data[0] << 8 | data[1]

        val += self.value_trim

        # Perform a moving average of the last n data values.
        # This ramps up to the max number of n data values to average.
        if len(self.data_history) < self.data_history_maxlen:
            self.data_history.append(val)
        else:
            self.data_history.pop(0)
            self.data_history.append(val)

        window_val = int(sum(self.data_history)/len(self.data_history))

        return val, window_val

    def value_to_level(self, raw_value: int) -> int:
        """
        Maps raw ADC values from the MCP3221 to a discrete moisture level.

        The sensor output is inversely proportional:
        - High ADC values (e.g., 3600) indicate dry soil.
        - Low ADC values (e.g., 500) indicate wet soil.

        Example:
            If driest_value=3600 and wettest_value=500:
            - A raw value of 3600 returns level 1.
            - A raw value of 2050 (midpoint) returns level 5.
            - A raw value of 500 returns level 10.
        """
        if raw_value is None:
            logging.error(f"Can't convert SMS value '{raw_value}' to level.")
            return None

        # Clamp at the wettest and the driest raw values to the highest
        # and lowest levels, respectively.
        if raw_value >= self.driest_value:
            return 1
        if raw_value <= self.wettest_value:
            return self.levels

        # Calculate the percentage of "moisture" (0.0 to 1.0).
        # This gives us 0.0 at driest_value and 1.0 at wettest_value.
        total_range = self.driest_value - self.wettest_value
        moisture_ratio = (self.driest_value - raw_value) / total_range

        # Map to 1-10 scale.
        level = round(moisture_ratio * (self.levels - 1)) + 1

        return level

class ALS(Sensor):
    """
    Ambient Light Sensor. Underlying sensor is probably a BH1750. Connected
    via I2C.
    """

    def __init__(self):
        self.i2cbus = SMBus(1)
        self.i2caddr = 0x23

        self.lux_trim = local_options.als_lux_trim

    def read(self) -> float:
        # From register 0x10, sensor readings are 2 bytes:
        #   0 : MSB of lux reading
        #   1 : LSB of lux reading
        data = self.i2cbus.read_i2c_block_data(self.i2caddr, 0x10, 2)
        val = data[0] << 8 | data[1]
        lux = float(val)/1.2 + self.lux_trim
        return round(lux, 1)

def header_matches(csv_file: str, header: tuple[str]) -> bool:
    # Assumes csv_file exists and has a header.
    with open(csv_file, "r") as f:
        reader = csv.reader(f)
        return header == tuple(next(reader))

def create_log(csv_file: str, header: tuple[str]):
    logging.info(f"Making csv log '{csv_file}'")
    with open(csv_file, "w") as f:
        writer = csv.writer(f)
        writer.writerow(header)

def write_row(csv_file: str, row: tuple):
    with open(csv_file, "a") as f:
        writer = csv.writer(f)
        writer.writerow(row)

def gardenmon_main():
    logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')

    header_row = (
        "Time",
        "CPU Temperature (F)",
        "Ambient Temperature (F)",
        "Ambient Humidity (%)",
        "Soil Temperature (F)",
        "Soil Moisture Raw Value",
        "Soil Moisture Level",
        "Ambient Light (lx)"
    )

    csv_log_dir = "/var/log/gardenmon"
    start_time = datetime.datetime.now()
    hostname = gethostname()
    daily_csv_log = f"{csv_log_dir}/{hostname}_{start_time.date()}.csv"
    main_csv_log = f"{csv_log_dir}/{hostname}_main.csv"

    if not os.path.exists(csv_log_dir):
        logging.info(f"Making csv log dir '{csv_log_dir}'")
        os.makedirs(csv_log_dir)

    # If log does not exist, create new log.
    # If log exists but header doesn't match, make backup and create new log.
    # If log exists and header does match, nothing needs to be done.
    for log in (daily_csv_log, main_csv_log):
        log_exists = os.path.isfile(log)
        if log_exists:
            header_match = header_matches(log, header_row)
            if header_match:
                logging.info(f"'{log}' exists and header matches.")
            else:
                logging.info(f"'{log}' exists but header does not match.")
                backup = f"{log}.{start_time.strftime('%Y%m%d%H%M%S')}.bak"
                logging.info(f"Moving '{log}' to {backup}")
                os.rename(log, backup)
                create_log(log, header_row)
        else:
            logging.info(f"'{log}' does not exist.")
            create_log(log, header_row)

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
        current_time_local = datetime.datetime.now()
        current_time_utc = datetime.datetime.now(datetime.timezone.utc)

        timestamp  = current_time_local.strftime("%Y-%m-%d %H:%M:%S")
        cpu_temp   = cpu_temp_sensor.get_value_or_none()
        aths_vals  = aths_sensor.get_value_or_none()
        aths_temp  = aths_vals["temperature"]
        aths_hmd   = aths_vals["humidity"]
        sts_temp   = sts_sensor.get_value_or_none()
        sms_val, sms_window_val = sms_sensor.get_value_or_none()
        sms_level  = sms_sensor.value_to_level(sms_window_val)
        als_lux    = als_sensor.get_value_or_none()

        row = (
            timestamp,
            cpu_temp,
            aths_temp,
            aths_hmd,
            sts_temp,
            sms_val,
            sms_window_val,
            sms_level,
            als_lux
        )

        # Write rows to logs. If the day has changed, create new daily log.
        daily_csv_log = f"{csv_log_dir}/{hostname}_{current_time_local.date()}.csv"
        if not os.path.isfile(daily_csv_log):
            create_log(daily_csv_log, header_row)
        write_row(daily_csv_log, row)
        write_row(main_csv_log, row)

        data = (
            cpu_temp,
            als_lux,
            sms_val,
            sms_window_val,
            sms_level,
            sts_temp,
            aths_temp,
            aths_hmd,
            current_time_utc,
            hostname
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
                "(cpu_temp_f, ambient_light_lx, soil_moisture_val, soil_moisture_window_val, soil_moisture_level, "
                "soil_temp_f, ambient_temp_f, ambient_humidity, insert_time, device) "
                "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
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
