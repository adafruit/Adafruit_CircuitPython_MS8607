# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
# SPDX-License-Identifier: MIT
from time import sleep
import board
import busio
from adafruit_ms8607 import MS8607Humidity

i2c = busio.I2C(board.SCL, board.SDA)
sensor = MS8607Humidity(i2c)

while True:

    print("%.2f %% rH" % sensor.relative_humidity)
    print("\n------------------------------------------------")
    sleep(1)
