# Micropython devoce driver for the sensirion sps30 particle matter sensor from sensirion.
# Partially based on a python3 script: https://github.com/feyzikesim/sps30/blob/master/sps30/sps30.py
#
# Only I2C communication is supported.
#
# @Thomas Braun
# MIT License

from machine import Pin, SoftI2C
import time
import struct

class SPS30_I2C():
    """
    MicroPython driver for the Sensirion SPS30 particulate matter sensor over I2C.

    This class provides methods to initialize, configure, and read data from the SPS30 sensor
    using either 16-bit unsigned integer or IEEE754 float output formats.

    Features:
    - Start/stop measurement
    - Sleep/wake control
    - Fan auto-cleaning and manual cleaning
    - CRC-8 data integrity checking
    - Read firmware version, serial number, status register
    - Supports both UI16 and FP32 measurement modes

    Attributes:
        dict_values (dict): Dictionary of measured values with keys like 'pm2p5', 'nc0p5', etc.
        dict_register (dict): Dictionary of status flags from the status register.

    Parameters:
        i2c (I2C): Initialized MicroPython I2C object.
        datatype (str): Data format for measurements ('UI16' or 'FP32').
    """

    _SPS_ADDR = 0x69 
    # I2C cmands (Stolen from https://github.com/dvsu/sps30/blob/main/sps30.py)
    CMD_START_MEASUREMENT = [0x00, 0x10]
    CMD_STOP_MEASUREMENT = [0x01, 0x04]
    CMD_READ_DATA_READY_FLAG = [0x02, 0x02]
    CMD_READ_MEASURED_VALUES = [0x03, 0x00, 0x34] #[0x03, 0x00]
    CMD_SLEEP = [0x10, 0x01]
    CMD_WAKEUP = [0x11, 0x03]
    CMD_START_FAN_CLEANING = [0x56, 0x07]
    CMD_AUTO_CLEANING_INTERVAL = [0x80, 0x04]
    CMD_PRODUCT_TYPE = [0xD0, 0x02]
    CMD_SERIAL_NUMBER = [0xD0, 0x33]
    CMD_FIRMWARE_VERSION = [0xD1, 0x00]
    CMD_READ_STATUS_REGISTER = [0xD2, 0x06]
    CMD_CLEAR_STATUS_REGISTER = [0xD2, 0x10]
    CMD_RESET = [0xD3, 0x04]

    _DATA_FORMAT = {
            "IEEE754_float": 0x03,
            "unsigned_16_bit_integer": 0x05
        }

    dict_values = {"mc_pm1.0"  : None,
                    "mc_pm2.5"  : None,
                    "mc_pm4.0"  : None,
                    "mc_pm10.0" : None,
                    "pc_pm0.5"  : None,
                    "pc_pm1.0"  : None,
                    "pc_pm2.5"  : None,
                    "pc_pm4.0"  : None,
                    "pc_pm10.0" : None,
                    "typical_size": None}
    # Micropaython dict.keys() does not preserve the order of keys. We use an explicit list:
    _values_keys = ["mc_pm1.0",
                    "mc_pm2.5",
                    "mc_pm4.0" ,
                    "mc_pm10.0",
                    "pc_pm0.5" ,
                    "pc_pm1.0" ,
                    "pc_pm2.5" ,
                    "pc_pm4.0" ,
                    "pc_pm10.0",
                    "typical_size"]
    
    dict_register = {"Fan speed error": False, "Laser error": False,
                    "Fan error": False}

    dict_firmware_version = {"major": None, "minor": None}

    # Length of response in bytes
    NBYTES_READ_DATA_READY_FLAG = 3
    NBYTES_MEASURED_VALUES_FLOAT = 60  # IEEE754 float
    NBYTES_MEASURED_VALUES_INTEGER = 30  # unsigned 16 bit integer
    NBYTES_AUTO_CLEANING_INTERVAL = 6
    NBYTES_PRODUCT_TYPE = 12
    NBYTES_SERIAL_NUMBER = 48
    NBYTES_FIRMWARE_VERSION = 3
    NBYTES_READ_STATUS_REGISTER = 6

    # Initialization --------------------------------------------------------------------
    def __init__(self, i2c, datatype="UI16"):
        self.i2c = i2c

        # Handling measurement data-types
        self.datatype_init = datatype
        if datatype == "FP32": # Request IEEE754 floats for measurements
            self.datatype = self._DATA_FORMAT["IEEE754_float"]
            self.nbytes_measured_values = self.NBYTES_MEASURED_VALUES_FLOAT
        else:
            self.datatype = self._DATA_FORMAT["unsigned_16_bit_integer"]
            self.nbytes_measured_values = self.NBYTES_MEASURED_VALUES_INTEGER
            if datatype != "UI16":
                print(f"SPS30 Warning: Unknown datatype ({datatype}). Fallback to UI16")

    # CRC8 functions --------------------------------------------------------------------

    def _crc8(self, data):
        """ Calculate CRC8 using polynomial 0x31 and initial value 0xFF """
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
                crc &= 0x0000FF  # Ensure 8-bit value
        return crc

    def _verify_crc(self, data):
        """ Verify the CRC for data in groups of two bytes + 1 checksum byte """
        # Data length must be a multiple of 3
        if len(data) % 3 != 0:
            print("SPS30 verify CRC error: Data length is not a multiple of 3")
            return False

        # Iterate over each group of two bytes + checksum byte
        for i in range(0, len(data), 3):
            bytes_pair = data[i:i+2]
            checksum = data[i+2]
            calculated_crc = self._crc8(bytes_pair)
            
            if calculated_crc != checksum:
                print(f"SPS 30 CRC Mismatch at index {i}: Expected {checksum}, Got {calculated_crc}")
                return False

        return True

    # SPS30 information ------------------------------------------------------------------------------

    def read_status_register(self):
        """
        Reads and decodes the sensor status register.

        Returns:
            dict: A dictionary indicating the error status of internal components like fan and laser.
        """
        try:
            # Read data command
            self.i2c.writeto(self._SPS_ADDR, bytes(self.CMD_READ_STATUS_REGISTER))
            time.sleep(0.1)  # Allow data to be prepared

            # Reading dat (either UInt or FP32), defined during class initialization
            print("Reading SPS30 register data. Warning: Formally works, but I'm not able to debug.")
            register_raw = self.i2c.readfrom(self._SPS_ADDR, self.NBYTES_READ_STATUS_REGISTER)

        except Exception as e:
            print("SPS30 I2C Read Error:", e)
            return None

        # Checking data integrity
        if not self._verify_crc(register_raw):
            print("SPS30 CRC Check Failed")
            return None

        register_data = register_raw[0:2] + register_raw[3:5]
        # print(register_data)

        register_bit = []
        for byte in register_data:
            for i in range(7, -1, -1):  # MSB to LSB
                register_bit.append((byte >> i) & 1 == 1)
        # print(register_bit)

        # Only three bits are currently defined 2025-05-14
        self.dict_register["Fan speed error"] = register_bit[21]
        self.dict_register["Laser error"] = register_bit[5]
        self.dict_register["Fan error"] = register_bit[4]
        return self.dict_register

    def clear_status_register(self):
        """
        Sends a command to clear the SPS30 sensor's status register.
        """
        try:
            self.i2c.writeto(self._SPS_ADDR,bytes(self.CMD_CLEAR_STATUS_REGISTER))
        except Exception as e:
            print(f"SPS30 clear status register Error: {e}")

    def read_firmware_version(self):
        """
        Retrieves the sensor firmware version.

        Returns:
            int: Firmware major version. None if CRC fails or read error occurs.
        """
        self.i2c.writeto(self._SPS_ADDR, bytes(self.CMD_FIRMWARE_VERSION))
        try:
            print("SPS30 reading firmware version")
            raw_data = self.i2c.readfrom(self._SPS_ADDR, self.NBYTES_FIRMWARE_VERSION)
            # print(raw_data)
            # print(struct.unpack('>B', raw_data)[0])
            if not self._verify_crc(raw_data):
                print("SPS30 CRC Check Failed")
                return None

            self.dict_firmware_version["major"] = raw_data[0]
            self.dict_firmware_version["minor"] = raw_data[1]
            
            return self.dict_firmware_version
        except Exception as e:
            print("SPS30 Firmware Read Error:", e)
            return None

    def read_product_type(self):
        """
        Retrioeves the product type string.
        
        Returns:
            String containing product type.
        """

        self.i2c.writeto(self._SPS_ADDR, bytes(self.CMD_PRODUCT_TYPE))
        try:
            print("SPS30 reading product type")
            raw_data = self.i2c.readfrom(self._SPS_ADDR, self.NBYTES_PRODUCT_TYPE)
            # print(raw_data)
            
            if not self._verify_crc(raw_data):
                print("SPS30 CRC Check Failed")
                return None

            chars = []   
            for i in range(0,len(raw_data),3):
                chars.extend(raw_data[i:i+2])  # skip the third byte (CRC)

            # print(chars)
            string =  bytes(chars).decode('ascii').strip()
            return string

        except Exception as e:
            print("SPS30 Product Type Read Error:", e)
            return None

    def read_serial_number(self):
        """
        Retrioeves the product serial number.
        
        Returns:
            String containing serial number.
        """

        self.i2c.writeto(self._SPS_ADDR, bytes(self.CMD_SERIAL_NUMBER))
        try:
            print("SPS30 reading serial number")
            raw_data = self.i2c.readfrom(self._SPS_ADDR, self.NBYTES_SERIAL_NUMBER)
            print(raw_data)
            
            if not self._verify_crc(raw_data):
                print("SPS30 CRC Check Failed")
                return None

            chars = []   
            for i in range(0,len(raw_data),3):
                chars.extend(raw_data[i:i+2])  # skip the third byte (CRC)

            print(chars)

            string =  bytes(chars).decode('ascii').strip()
            return string.rstrip('0')

        except Exception as e:
            print("SPS30 Product Type Read Error:", e)
            return None

    # Housekeeping commends
    def sleep(self):
        """
        Puts the SPS30 into low-power sleep mode.
        """
        print("Setting SPS30 into sleep mode.")
        try:
            self.i2c.writeto(self._SPS_ADDR,bytes(self.CMD_SLEEP))
        except Exception as e:
            print(f"SPS30 enter sleep mode Error: {e}")  

    def wakeup(self):
        """
        Wakes the SPS30 sensor from sleep mode.
        """
        try:
            print("SPS30 waking up")
            self.i2c.writeto(self._SPS_ADDR,bytes(self.CMD_WAKEUP))
            time.sleep(0.05)
            self.i2c.writeto(self._SPS_ADDR,bytes(self.CMD_WAKEUP))
        except Exception as e:
            print("SPS30 I2C Read Error:", e)
            return None

    def start_fan_cleaning(self):
        """
        Triggers a manual fan cleaning cycle.
        """
        try:
            print("SPS30 sending self cleaning command")
            self.i2c.writeto(self._SPS_ADDR,bytes(self.CMD_START_FAN_CLEANING))
        except Exception as e:
            print("SPS30 I2C Read Error:", e)

    def read_auto_cleaning_interval(self):
        """
        Reads the configured auto-cleaning interval of the sensor.

        Returns:
            int: Cleaning interval in seconds, or None on CRC failure.
        """
        self.i2c.writeto(self._SPS_ADDR,bytes(self.CMD_AUTO_CLEANING_INTERVAL))
        raw_data = self.i2c.readfrom(self._SPS_ADDR,self.NBYTES_AUTO_CLEANING_INTERVAL)
        if not self._verify_crc(raw_data):
                print("SPS30 CRC Check Failed")
                return None

        interval = []
        for i in range(0, self.NBYTES_AUTO_CLEANING_INTERVAL, 3):
            interval.extend(raw_data[i:i+2])
        
        return (interval[0] << 24 | interval[1] << 16 | interval[2] << 8 | interval[3])



    # Masurements ------------------------------------------------------------------------------

    def start_measurement(self):
        """
        Sends a command to start continuous measurement mode.

        The format of the returned data is determined by the `datatype` passed at initialization.
        """
        data = self.CMD_START_MEASUREMENT
        data.extend([self.datatype, 0x00])
        data.append(self._crc8(data[2:4]))
        try:
            #print(f"Start Measurement Command: {bytes(data)}")
            self.i2c.writeto(self._SPS_ADDR, bytes(data))
            print("Measurement started")
            time.sleep(0.02) # Estiamted command execution time < 20 ms
        except Exception as e:
            print(f"Start Measurement Error: {e}")

    def read_data_ready_flag(self):
        '''
        Checks whether new measurement data is available for reading.

        Returns:
            bool: True if data is ready, False otherwise. None on error.
        '''
        self.i2c.writeto(self._SPS_ADDR, bytes(self.CMD_READ_DATA_READY_FLAG))
        try:
            print("SPS30 reading data ready flag")
            raw_data = self.i2c.readfrom(self._SPS_ADDR, self.NBYTES_READ_DATA_READY_FLAG)
            # print(raw_data)
            if not self._verify_crc(raw_data):
                print("SPS30 CRC Check Failed reading data ready flag")
                return None
            if raw_data[1] == 0x01:
                flag = True
            else:
                flag = False
            return flag
        except Exception as e:
            print("SPS30 I2C Read Error:", e)
            return None

    def stop_measurement(self):
        """
        Stops the current measurement cycle and halts sensor data collection.
        """
        try:
            self.i2c.writeto(self._SPS_ADDR, bytes(self.CMD_STOP_MEASUREMENT))
            print("Measurement stopped")
            time.sleep(0.02) # Estiamted command execution time < 20 ms
        except Exception as e:
            print(f"Stop Measurement Error: {e}")   

    def _parse_uint_data(self, raw_data):
        data_values = []
        for i in range(0, len(raw_data), 3):
            data_pair = raw_data[i:i+2]
            checksum = raw_data[i+2]
            # Verify CRC
            calculated_crc = self._crc8(data_pair)
            if calculated_crc != checksum:
                print(f"SPR30 CRC Error at index {i}: Expected {checksum}, Got {calculated_crc}")
                continue
            # Convert to unsigned integer (Big Endian)
            value = struct.unpack(">H", data_pair)[0]
            data_values.append(value)

        return data_values

    def _parse_fp32_data(self, raw_data):
        # Check for data integrity
        self._verify_crc(raw_data)

        # parese data: Take out 6 bit, remove checkssums, convert 4 bit to FP32, Append to data values

        data_values = []

        for i in range(0, len(raw_data), 6):
            chunk = raw_data[i:i+2] + raw_data[i+3:i+5]  # skip CRC bytes (i+2 and i+5)
            # Convert 4 bytes to IEEE754_float (Big Endian)
            value = struct.unpack('>f', chunk)[0]
            data_values.append(value)

        return data_values

    def read_data(self):
        """
        Reads and parses measured data from the SPS30 sensor.

        Returns:
            list: A list of 10 values representing PM and number concentration values.
                  Values are returned as `int` or `float`, depending on selected datatype.
        """
        try:
            # Read data command
            self.i2c.writeto(self._SPS_ADDR, bytes(self.CMD_READ_MEASURED_VALUES))
            time.sleep(0.1)  # Allow data to be prepared

            # Reading dat (either UInt or FP32), defined during class initialization
            raw_data = self.i2c.readfrom(self._SPS_ADDR, self.nbytes_measured_values)
            # print(f"SPS30 Maeured Raw Data: {raw_data}")

            # Parse data as Big Endian Unsigned Integers
            if self.datatype_init == "UI16":
                data_values = self._parse_uint_data(raw_data)
            else:
                data_values = self._parse_fp32_data(raw_data)

            # print("Data Values:", data_values)

            # transfer the data list into doct
            for key, val in zip(self._values_keys, data_values):
                self.dict_values[key] = val

            return self.dict_values

        except Exception as e:
            print(f"SPS30 Data Read Error: {e}")
            return [] 
