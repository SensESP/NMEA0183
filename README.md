# GNSS (GPS) Module Input Sensor for SensESP

This repository implements a GNSS sensor library for [SensESP](https://signalk.org/SignalK/SensESP/).
It reads NMEA-0183 formatted GPS data from a serial port and provides SensESP Producers for the different data.
Convenience functions for connecting the providers to Signal K output are also provided.

To use the library in your own projects, you have to include it in your `platformio.ini` `lib_deps` section:

    lib_deps =
        SignalK/SensESP@^2.1.0
        SensESP/GNSS@^2.0.0

See also the [example main file](blob/main/examples/gnss_example.cpp).

For more information on using SensESP and external add-on libraries, see the [SensESP documentation](https://signalk.org/SensESP/docs/).
