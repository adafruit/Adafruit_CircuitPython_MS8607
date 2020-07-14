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

.. todo:: Add links to any specific hardware product page(s), or category page(s). Use unordered list & hyperlink rST
   inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies based on the library's use of either.

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MS8607.git"

from micropython import const
import adafruit_bus_device.i2c_device as i2cdevice
from adafruit_register.i2c_struct import ROUnaryStruct
from adafruit_register.i2c_bits import RWBits, ROBits
from adafruit_register.i2c_bit import RWBit, ROBit

_WHO_AM_I = const(0x0F)

P_T_RESET = const(0x1E)            # Pressure and temperature sensor reset
CONVERT_D1_OSR_256 = const(0x40)   # Sampling rate 256
CONVERT_D1_OSR_512 = const(0x42)   # Sampling rate 512
CONVERT_D1_OSR_1024 = const(0x44)  # Sampling rate 1024
CONVERT_D1_OSR_2048 = const(0x46)  # Sampling rate 2048
CONVERT_D1_OSR_4096 = const(0x48)  # Sampling rate 4096
CONVERT_D1_OSR_8192 = const(0x4A)  # Sampling rate 8192
CONVERT_D2_OSR_256 = const(0x50)   # Sampling rate 256
CONVERT_D2_OSR_512 = const(0x52)   # Sampling rate 512
CONVERT_D2_OSR_1024 = const(0x54)  # Sampling rate 1024
CONVERT_D2_OSR_2048 = const(0x56)  # Sampling rate 2048
CONVERT_D2_OSR_4096 = const(0x58)  # Sampling rate 4096
CONVERT_D2_OSR_8192 = const(0x5A)  # Sampling rate 8192
ADC_READ = const(0x00)             # Command to read from ADC

HUM_RESET = const(0xFE)               # Humidity sensor reset
HUM_WRITE_REGISTER = const(0xE6)      # Humidity sensor write register
HUM_READ_REGISTER = const(0xE7)       # Humidity sensor read register
HUM_MEASURE_RH_HOLD = const(0xE5)     # Humidity sensor measure relative humidity hold master
HUM_MEASURE_RH_NO_HOLD = const(0xF5)  # Humidity sensor measure relative humidity no hold master

_MS8607_CHIP_ID = const("0xXX")
_MS8607_DEFAULT_PT_ADDRESS = 0x76
_MS8607_DEFAULT_RH_ADDRESS = 0x40




class CV:
    """struct helper"""

    @classmethod
    def add_values(cls, value_tuples):
        "creates CV entires"
        cls.string = {}
        cls.lsb = {}

        for value_tuple in value_tuples:
            name, value, string, lsb = value_tuple
            setattr(cls, name, value)
            cls.string[value] = string
            cls.lsb[value] = lsb

    @classmethod
    def is_valid(cls, value):
        "Returns true if the given value is a member of the CV"
        return value in cls.string


class Rate(CV):
    pass  # pylint: disable=unnecessary-pass

class MS8607:  # pylint: disable=too-many-instance-attributes
    """Library for the TE MS8607 Pressure, Humidity, and Temperature Sensor

        :param ~busio.I2C i2c_bus: The I2C bus the MS8607 is connected to.

    """

    def __init__(self, i2c_bus):
        self.i2c_device = i2cdevice.I2CDevice(i2c_bus, _MS8607_DEFAULT_ADDRESS)
        if not self._chip_id in [_MS8607_CHIP_ID]:
            raise RuntimeError(
                "Failed to find MS8607! Found chip ID 0x%x" % self._chip_id
            )

    # This is the closest thing to a software reset. It re-loads the calibration values from flash
    def _boot(self):
        self._boot_bit = True
        # wait for the reset to finish
        while self._boot_bit:
            pass

    @property
    def relative_humidity(self):
        """The current relative humidity measurement in %rH"""
        calibrated_value_delta = self.calib_hum_value_1 - self.calib_hum_value_0
        calibrated_measurement_delta = self.calib_hum_meas_1 - self.calib_hum_meas_0

        calibration_value_offset = self.calib_hum_value_0
        calibrated_measurement_offset = self.calib_hum_meas_0
        zeroed_measured_humidity = self._raw_humidity - calibrated_measurement_offset

        correction_factor = calibrated_value_delta / calibrated_measurement_delta

        adjusted_humidity = (
            zeroed_measured_humidity * correction_factor + calibration_value_offset
        )

        return adjusted_humidity

    @property
    def temperature(self):
        """The current temperature measurement in degrees C"""

        calibrated_value_delta = self.calibrated_value_1 - self.calib_temp_value_0
        calibrated_measurement_delta = self.calib_temp_meas_1 - self.calib_temp_meas_0

        calibration_value_offset = self.calib_temp_value_0
        calibrated_measurement_offset = self.calib_temp_meas_0
        zeroed_measured_temp = self._raw_temperature - calibrated_measurement_offset

        correction_factor = calibrated_value_delta / calibrated_measurement_delta

        adjusted_temp = (
            zeroed_measured_temp * correction_factor
        ) + calibration_value_offset

        return adjusted_temp

    @property
    def data_rate(self):
        """The rate at which the sensor measures ``relative_humidity`` and ``temperature``.
        ``data_rate`` should be set to one of the values of ``adafruit_ms8607.Rate``. Note that
        setting ``data_rate`` to ``Rate.ONE_SHOT`` will cause  ``relative_humidity`` and
        ``temperature`` measurements to only update when ``take_measurements`` is called."""
        return self._data_rate

    @data_rate.setter
    def data_rate(self, value):
        if not Rate.is_valid(value):
            raise AttributeError("data_rate must be a `Rate`")

        self._data_rate = value

    @property
    def humidity_data_ready(self):
        """Returns true if a new relative humidity measurement is available to be read"""
        return self._humidity_status_bit

    @property
    def temperature_data_ready(self):
        """Returns true if a new temperature measurement is available to be read"""
        return self._temperature_status_bit

    def take_measurements(self):
        """Update the value of ``relative_humidity`` and ``temperature`` by taking a single
        measurement. Only meaningful if ``data_rate`` is set to ``ONE_SHOT``"""
        self._one_shot_bit = True
        while self._one_shot_bit:
            pass
