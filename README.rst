Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-ms8607/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/ms8607/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_MS8607/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_MS8607/actions
    :alt: Build Status

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

CircuitPython driver for the MS8607 PTH sensor


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_.

Installing from PyPI
=====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-ms8607/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-ms8607

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-ms8607

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-ms8607

Usage Example
=============

.. code-block:: python3

    from time import sleep
    import board
    from adafruit_ms8607 import MS8607

    i2c = board.I2C()
    sensor = MS8607(i2c)

    while True:

        print("Pressure: %.2f hPa" % sensor.pressure)
        print("Temperature: %.2f C" % sensor.temperature)
        print("Humidity: %.2f %% rH" % sensor.relative_humidity)
        sleep(1)



Documentation
=============

API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/ms8607/en/latest/>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_MS8607/blob/main/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
