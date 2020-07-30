# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_ms8607`
================================================================================

CircuitPython driver for the MS8607 PTH sensor


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* `Adafruit AS7341 Breakout <https:#www.adafruit.com/products/45XX>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https:#github.com/adafruit/circuitpython/releases

 * Adafruit's Bus Device library: https:#github.com/adafruit/Adafruit_CircuitPython_BusDevice
 * Adafruit's Register library: https:#github.com/adafruit/Adafruit_CircuitPython_Register

"""

__version__ = "0.0.0-auto.0"
__repo__ = "https:#github.com/adafruit/Adafruit_CircuitPython_MS8607.git"


from struct import unpack_from
from time import sleep
from micropython import const
import adafruit_bus_device.i2c_device as i2c_device

# # HSENSOR device commands
_MS8607_RESET_COMMAND = const(0xFE)  #
_MS8607_READ_HUMIDITY_W_HOLD_COMMAND = const(0xE5)  #
_MS8607_READ_HUMIDITY_WO_HOLD_COMMAND = const(0xF5)  #
_MS8607_READ_SERIAL_FIRST_8BYTES_COMMAND = const(0xFA0F)  #
_MS8607_READ_SERIAL_LAST_6BYTES_COMMAND = const(0xFCC9)  #
_MS8607_WRITE_USER_REG_COMMAND = const(0xE6)  #
_MS8607_READ_USER_REG_COMMAND = const(0xE7)  #
_MS8607_PRESSURE_CALIB_ROM_ADDR = const(0xA0)  #  16-bit registers through 0xAE


_MS8607_PTSENSOR_ADDR = const(0x76)  #
_MS8607_HSENSOR_ADDR = const(0x40)  #
_MS8607_COEFF_MUL = const(125)  #
_MS8607_COEFF_ADD = const(-6)  #

# enum MS8607_humidity_resolution
# {
#        MS8607_humidity_resolution_12b = 0,
#        MS8607_humidity_resolution_8b,
#        MS8607_humidity_resolution_10b,
#        MS8607_humidity_resolution_11b
# }]


class MS8607Humidity:
    """Library for the MS8607 Humidity Sensor


        :param ~busio.I2C i2c_bus: The I2C bus the MS8607 is connected to.

    """

    def __init__(self, i2c_bus):

        self.humidity_i2c_device = i2c_device.I2CDevice(i2c_bus, _MS8607_HSENSOR_ADDR)
        self.pressure_i2c_device = i2c_device.I2CDevice(i2c_bus, _MS8607_PTSENSOR_ADDR)

        self._buffer = bytearray(3)
        self.reset()
        self.initialize()

    def reset(self):
        """Reset the sensor to an initial unconfigured state"""

    def initialize(self):
        """Configure the sensors with the default settings and state.
        For use after calling `reset()`
        """
        constants = []

        for i in range(7):
            offset = 2 * i
            self._buffer[0] = _MS8607_PRESSURE_CALIB_ROM_ADDR + offset
            with self.pressure_i2c_device as i2c:
                i2c.write_then_readinto(
                    self._buffer,
                    self._buffer,
                    out_start=0,
                    out_end=1,
                    in_start=0,
                    in_end=2,
                )

            # print("got calibration constant %d(%s)"%(calibration_const, hex(calibration_const)))
            constants.extend(unpack_from(">H", self._buffer[0:2]))

        crc_value = (constants[0] & 0xF000) >> 12
        constants.append(0)
        if not self._check_press_calibration_crc(constants, crc_value):
            raise RuntimeError("CRC Error reading humidity calibration constants")

        self.press_sens = constants[1]
        self.press_offset = constants[2]
        self.press_sens_temp_coeff = constants[3]
        self.press_offset_temp_coeff = constants[4]
        self.ref_temp = constants[5]
        self.temp_temp_coeff = constants[6]
        # psensor_resolution_osr = MS8607_PRESSURE_RESOLUTION_OSR_8192
        print("self.press_sens", self.press_sens)
        print("self.press_offset", self.press_offset)

        print("self.press_sens_temp_coeff", self.press_sens_temp_coeff)
        print("self.press_offset_temp_coeff", self.press_offset_temp_coeff)
        print("self.ref_temp", self.ref_temp)
        print("self.temp_temp_coeff", self.temp_temp_coeff)

    @property
    def temperature(self):
        """The current temperature in degrees Celcius"""
        return 5
        # # self._buffer.clear()
        # self._buffer[0] = _MS8607_READ_HUMIDITY_WO_HOLD_COMMAND
        # with self.i2c_device as i2c:
        #     i2c.write(self._buffer, end=1)
        # sleep(0.1)  # _i2cPort->requestFrom((uint8_t)MS8607_HSENSOR_ADDR, 3U)

        # with self.i2c_device as i2c:
        #     i2c.readinto(self._buffer, end=3)

        # # _adc = (buffer[0] << 8) | buffer[1]
        # # crc = buffer[2]
        # raw_humidity = unpack_from(">H", self._buffer)[0]
        # crc_value = unpack_from(">B", self._buffer, offset=2)[0]
        # humidity = (raw_humidity * (_MS8607_COEFF_MUL / (1 << 16))) + _MS8607_COEFF_ADD
        # if not self._check_humidity_crc(raw_humidity, crc_value):
        #     raise RuntimeError("CRC Error reading humidity data")
        # return humidity

    @property
    def relative_humidity(self):
        """The current relative humidity in % rH"""

        # self._buffer.clear()
        self._buffer[0] = _MS8607_READ_HUMIDITY_WO_HOLD_COMMAND
        with self.humidity_i2c_device as i2c:
            i2c.write(self._buffer, end=1)
        sleep(0.1)  # _i2cPort->requestFrom((uint8_t)MS8607_HSENSOR_ADDR, 3U)

        with self.humidity_i2c_device as i2c:
            i2c.readinto(self._buffer, end=3)

        # _adc = (buffer[0] << 8) | buffer[1]
        # crc = buffer[2]
        raw_humidity = unpack_from(">H", self._buffer)[0]
        crc_value = unpack_from(">B", self._buffer, offset=2)[0]
        humidity = (raw_humidity * (_MS8607_COEFF_MUL / (1 << 16))) + _MS8607_COEFF_ADD
        if not self._check_humidity_crc(raw_humidity, crc_value):
            raise RuntimeError("CRC Error reading humidity data")
        return humidity

        # *adc = _adc

        # return status

    @staticmethod
    def _check_humidity_crc(value, crc):
        polynom = 0x988000  # x^8 + x^5 + x^4 + 1
        msb = 0x800000
        mask = 0xFF8000
        result = value << 8  # Pad with zeros as specified in spec

        while msb != 0x80:
            # Check if msb of current value is 1 and apply XOR mask
            if result & msb:
                result = ((result ^ polynom) & mask) | (result & ~mask)

            # Shift by one
            msb >>= 1
            mask >>= 1
            polynom >>= 1

        if result == crc:
            return True
        return False

    @staticmethod
    def _check_press_calibration_crc(calibration_int16s, crc):
        cnt = 0
        n_rem = 0
        n_rem = 0
        crc_read = calibration_int16s[0]
        calibration_int16s[7] = 0
        calibration_int16s[0] = 0x0FFF & (calibration_int16s[0])  # Clear the CRC byte

        for cnt in range(16):
            # Get next byte

            if cnt % 2 == 1:
                n_rem ^= calibration_int16s[cnt >> 1] & 0x00FF
            else:
                n_rem ^= calibration_int16s[cnt >> 1] >> 8

            for _i in range(8, 0, -1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem <<= 1
                # we have to restrict to 16 bits
                n_rem &= 0xFFFF

        n_rem >>= 12
        calibration_int16s[0] = crc_read
        return n_rem == crc


# read_temperature_pressure_humidity(float *t, float *p,

# set_humidity_resolution(enum MS8607_humidity_resolution res)
# void set_humidity_i2c_master_mode(enum MS8607_humidity_i2c_master_mode mode)

# get_compensated_humidity(float temperature, float relative_humidity, float *compensated_humidity)

# /*
# private:
# bool hsensor_is_connected(void)
# hsensor_reset(void)
# hsensor_crc_check(uint16_t value, uint8_t crc)
# hsensor_read_user_register(uint8_t *value)
# hsensor_write_user_register(uint8_t value)
# enum MS8607_humidity_i2c_master_mode hsensor_i2c_master_mode
# hsensor_humidity_conversion_and_read_adc(uint16_t *adc)
# hsensor_read_relative_humidity(float *humidity)
# int err = set_humidity_resolution(MS8607_humidity_resolution_12b) # 12 bits
# hsensor_conversion_time = HSENSOR_CONVERSION_TIME_12b
# hsensor_i2c_master_mode = MS8607_i2c_no_hold
# float humidity = getHumidity()
# float temperature = getTemperature()
# float compensated_RH= get_compensated_humidity(temperature, humidity, &compensated_RH)
# float dew_point = get_dew_point(temperature, humidity, &dew_point)
