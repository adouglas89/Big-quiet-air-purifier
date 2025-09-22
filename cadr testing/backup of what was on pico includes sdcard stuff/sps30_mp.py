# sps30_mp.py
# MicroPython driver for Sensirion SPS30 (I2C) on Raspberry Pi Pico
# Author: ChatGPT (MicroPython port)
#
# Wiring (default in example):
#   Pico GP14 -> SDA, GP15 -> SCL, 3V3 -> VCC, GND -> GND
#
# Public API:
#   s = SPS30(i2c)                               # create driver
#   s.start_measurement()                        # begin measurement (float format)
#   ready = s.data_ready()                       # True/False
#   data = s.read_measurement()                  # dict of floats
#   s.stop_measurement()
#   s.start_fan_cleaning()
#
# Returned 'data' dict keys:
#   'mc_pm1p0', 'mc_pm2p5', 'mc_pm4p0', 'mc_pm10',
#   'nc_pm0p5', 'nc_pm1p0', 'nc_pm2p5', 'nc_pm4p0', 'nc_pm10',
#   'typical_particle_size'
#
# Notes:
# - Commands & CRC behavior follow Sensirion’s SPS30 datasheet.
# - If you need UART mode instead of I2C, say the word—I can provide a UART version.

from machine import I2C
import struct
import time

class SPS30:
    DEFAULT_ADDR = 0x69

    # Command words (big-endian) from Sensirion SPS30 datasheet
    _CMD_START_MEAS      = b'\x00\x10'
    _CMD_STOP_MEAS       = b'\x01\x04'
    _CMD_DATA_READY      = b'\x02\x02'
    _CMD_READ_MEAS       = b'\x03\x00'
    _CMD_START_CLEAN     = b'\x56\x07'

    # Start-measurement argument for float output (0x03 0x00) with CRC per 2-byte word
    _ARG_START_MEAS_FLOAT = b'\x03\x00'

    # Each 16-bit word is followed by one CRC8 byte (poly 0x31, init 0xFF)
    @staticmethod
    def _crc8(word_bytes):
        # word_bytes: 2 bytes
        crc = 0xFF
        for b in word_bytes:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0x31) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def __init__(self, i2c: I2C, addr: int = DEFAULT_ADDR):
        self.i2c = i2c
        self.addr = addr

    def _write_cmd(self, cmd, arg_words=None):
        # cmd: 2 bytes
        # arg_words: list of 2-byte words (bytes objects length 2), each followed by CRC8 we add
        if arg_words:
            payload = bytearray(cmd)
            for w in arg_words:
                if len(w) != 2:
                    raise ValueError("Argument word must be 2 bytes")
                payload += w + bytes([self._crc8(w)])
            self.i2c.writeto(self.addr, payload)
        else:
            self.i2c.writeto(self.addr, cmd)

    def _read_words_with_crc(self, n_words):
        # Reads n_words * (2+1) bytes and verifies CRC per word
        raw = self.i2c.readfrom(self.addr, n_words * 3)
        out = bytearray()
        for i in range(n_words):
            w = raw[3*i:3*i+2]
            crc = raw[3*i+2]
            if self._crc8(w) != crc:
                raise OSError("CRC mismatch on word %d" % i)
            out += w
        return bytes(out)

    def start_measurement(self):
        # Start measurement in float format (0x03 0x00)
        self._write_cmd(self._CMD_START_MEAS, [self._ARG_START_MEAS_FLOAT])
        # Per datasheet, recommend small delay after starting
        time.sleep_ms(50)

    def stop_measurement(self):
        self._write_cmd(self._CMD_STOP_MEAS)
        time.sleep_ms(20)

    def data_ready(self):
        # Read "data ready" flag (1 word)
        self._write_cmd(self._CMD_DATA_READY)
        time.sleep_ms(3)
        words = self._read_words_with_crc(1)  # 2 bytes
        flag = (words[0] << 8) | words[1]
        return flag == 1

    def read_measurement(self):
        # Read 10 floats -> each float is 4 bytes but sent as 2x 16-bit words
        # With CRC per word, that's 10 floats * (2 words/float) * (2+1) bytes = 60 bytes
        self._write_cmd(self._CMD_READ_MEAS)
        time.sleep_ms(3)
        raw_words = self._read_words_with_crc(20)  # 20 words (40 data bytes)
        # Convert 10 big-endian floats
        floats = []
        for i in range(10):
            hi = raw_words[i*4:i*4+2]
            lo = raw_words[i*4+2:i*4+4]
            f = struct.unpack('>f', hi + lo)[0]
            floats.append(f)

        return {
            'mc_pm1p0': floats[0],
            'mc_pm2p5': floats[1],
            'mc_pm4p0': floats[2],
            'mc_pm10':  floats[3],
            'nc_pm0p5': floats[4],
            'nc_pm1p0': floats[5],
            'nc_pm2p5': floats[6],
            'nc_pm4p0': floats[7],
            'nc_pm10':  floats[8],
            'typical_particle_size': floats[9],
        }

    def start_fan_cleaning(self):
        self._write_cmd(self._CMD_START_CLEAN)
        # Cleaning takes some seconds; sensor won’t return data during cleaning.

