import machine
import time
import struct

# class for the Sensirion SPS30
# Connector:
# ---------------
# |       12345 |
# | *  *   *  * |
# ---------------
# Pin layout:
# 1: VDD (5V)
# 2: SDA
# 3: SCL
# 4: GRND (for I2C)
# 5: GND

class SPS30:
    address = 0x69
    start_measurement = [0x00, 0x10, 0x03, 0x00] # float numbers as return values
    cleanup = [0x56, 0x07]
    data_header = ['mass pm 1.0', 'mass pm 2.5', 'mass pm 4.0', 'mass pm 10',
            'number pm 0.5', 'number pm 1.0', 'number pm 2.5', 'number pm 4.0',
            'number pm 10', 'typical size']
    
    def __init__(self, i2c, print_output=False):
        self.i2c = i2c
        self.output = print_output
        self.found = self.__findi2c()
        self.starttime = time.time()
    
    def write_read(self, cmd, nbytes=60):
        if len(cmd) == 2:
            cmd.append(self.calc_crc8(cmd))
        self.i2c.writeto(self.address, cmd)
        return self.i2c.readfrom(self.address, nbytes)
    
    def start_measurement(self):
        cmd = bytearray([0x00, 0x10, 0x03, 0x00, self.calc_crc8([0x03, 0x00])])
        max_tries = 0
        while max_tries < 10:
            try:
                self.i2c.writeto(self.address, cmd) # measurement mode
                time.sleep(0.5) # wait till everthing is ready
                self.cleanup(dt=10)
                max_tries = 11
            except OSError as e:
                max_tries += 1
                print('trying to configure %d/10' %(max_tries))
                print(e)
                time.sleep(0.5)
        if max_tries == 10:
            raise OSError("Not possible to configure sensor. Something is wrong...")

        self.read_data() # first measurement usualy doesn't look goodself.i2c.writeto(self.address, cmd) # measurement mode
    
    def cleanup(self, dt=10):
        self.starttime = time.time()
        self.i2c.writeto(self.address, bytearray([0x56, 0x07])) #cleanup
        time.sleep(dt)
        self.read_data()
        
    def gen_array(self, command):
        command.append(self.calc_crc8(command))
        return bytearray(command)
    
    def print_data(self):
        self.read_data()
        for ii, val in enumerate(self.last_measurement):
            print(self.data_header[ii], val)
    
    def read_data(self):
        cmd = bytearray([0x03, 0x00])
        data = self.write_read(cmd)
        cleandata = self.crc_array(data)
        self.last_measurement = []
        for ii in range(len(cleandata)//2):
            self.last_measurement.append(self.calcFloat(cleandata[2*ii] + cleandata[2*ii+1]))
        if (time.time() - self.starttime) % (259200) == 0: # cleanup every 3 days
            self.cleanup()
        
        
    def __findi2c(self):
        i2cbus = self.i2c.scan()
        if (self.address in i2cbus) and (len(i2cbus) < 10):
            if self.output:
                print('device found')
            return True
        else:
            print("I2Cbus devices: ", i2cbus)
            raise OSError("SPS30 not found on I2C bus!")
        
    def calc_crc8(self, byte_array):
        """
        Given an array of bytes calculate the CRC8 checksum.

        Args:
            byte_array (bytearray): The bytes needed to calculate the checksum
        
        Returns:
            crc (int): The calculated CRC8 checksum for the provided bytes
        """
        crc = 0xFF
        crc_poly = 0x131
        for byte in byte_array:
            crc ^= byte
            for bit in range(8):
                if crc & 0x80 != 0:
                    crc = (crc << 1) ^ crc_poly
                else:
                    crc = crc << 1
        return crc


    def check_crc8(self, byte_array, crc):
        if self.calc_crc8(byte_array) == crc:
            return True
        else:
            return False
    
    def crc_array(self, bt):
        nbytes = len(bt) // 3
        cleanarray = []
        if len(bt) % 3 != 0:
            print('error!')
        else:
            for ii in range(nbytes):
                start = ii*3
                stop  = ii*3+2
                if self.check_crc8(bt[start:stop], bt[stop]):
                    cleanarray.append(bt[start:stop])
        return cleanarray
    
    def calcFloat(self, byte_array):
        #Shamelessly stolen from the UnravelTec driver
        #TODO: Investigate using the struct library for other byte-bashing
        struct_float = struct.pack('>BBBB', byte_array[0], byte_array[1], byte_array[2], byte_array[3])
        return struct.unpack('>f', struct_float)[0]
            
