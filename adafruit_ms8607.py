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

* `Adafruit AS7341 Breakout <https://www.adafruit.com/products/45XX>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
 * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register

"""

# imports

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MS8607.git"


from struct import unpack_from
from time import sleep
from micropython import const
import adafruit_bus_device.i2c_device as i2c_device

# // HSENSOR device commands
_MS8607_RESET_COMMAND = const(0xFE)  #
_MS8607_READ_HUMIDITY_W_HOLD_COMMAND = const(0xE5)  #
_MS8607_READ_HUMIDITY_WO_HOLD_COMMAND = const(0xF5)  #
_MS8607_READ_SERIAL_FIRST_8BYTES_COMMAND = const(0xFA0F)  #
_MS8607_READ_SERIAL_LAST_6BYTES_COMMAND = const(0xFCC9)  #
_MS8607_WRITE_USER_REG_COMMAND = const(0xE6)  #
_MS8607_READ_USER_REG_COMMAND = const(0xE7)  #

_MS8607_HSENSOR_ADDR = const(0x40)  #

# enum MS8607_humidity_resolution
# {
#        MS8607_humidity_resolution_12b = 0,
#        MS8607_humidity_resolution_8b,
#        MS8607_humidity_resolution_10b,
#        MS8607_humidity_resolution_11b
# };]


class MS8607Humidity:
    """Library for the MS8607 Humidity Sensor


        :param ~busio.I2C i2c_bus: The I2C bus the MS8607 is connected to.

    """

    def __init__(self, i2c_bus):

        self.i2c_device = i2c_device.I2CDevice(i2c_bus, _MS8607_HSENSOR_ADDR)

        # self.reset()
        # self.initialize()
        self._buffer = bytearray(3)

    @property
    def relative_humidity(self):
        """The current relative humidity in % rH"""

        # self._buffer.clear()
        self._buffer[0] = _MS8607_READ_HUMIDITY_WO_HOLD_COMMAND
        with self.i2c_device as i2c:
            i2c.write(self._buffer, end=1)
        sleep(0.1)  # _i2cPort->requestFrom((uint8_t)MS8607_HSENSOR_ADDR, 3U);

        with self.i2c_device as i2c:
            i2c.readinto(self._buffer, end=3)

        # _adc = (buffer[0] << 8) | buffer[1];
        # crc = buffer[2];
        raw_humidity = unpack_from(">H", self._buffer)

        #  *humidity =
        #  adc * HUMIDITY_COEFF_MUL / (1UL << 16) + HUMIDITY_COEFF_ADD;
        return raw_humidity
        # // compute CRC
        # uint8_t crc;
        # status = hsensor_crc_check(_adc, crc);
        # if (status != MS8607_status_ok)
        #   return status;

        # *adc = _adc;

        # return status;


# read_temperature_pressure_humidity(float *t, float *p,

# set_humidity_resolution(enum MS8607_humidity_resolution res);
# void set_humidity_i2c_master_mode(enum MS8607_humidity_i2c_master_mode mode);

# get_compensated_humidity(float temperature, float relative_humidity, float *compensated_humidity);

# /*
# private:
# bool hsensor_is_connected(void);
# hsensor_reset(void);
# hsensor_crc_check(uint16_t value, uint8_t crc);
# hsensor_read_user_register(uint8_t *value);
# hsensor_write_user_register(uint8_t value);
# enum MS8607_humidity_i2c_master_mode hsensor_i2c_master_mode;
# hsensor_humidity_conversion_and_read_adc(uint16_t *adc);
# hsensor_read_relative_humidity(float *humidity);
# int err = set_humidity_resolution(MS8607_humidity_resolution_12b); // 12 bits
# hsensor_conversion_time = HSENSOR_CONVERSION_TIME_12b;
# hsensor_i2c_master_mode = MS8607_i2c_no_hold;
# float humidity = getHumidity();
# float temperature = getTemperature();
# float compensated_RH= get_compensated_humidity(temperature, humidity, &compensated_RH);
# float dew_point = get_dew_point(temperature, humidity, &dew_point);
