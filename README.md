# NMEA 0183 Data Input Sensor for SensESP

This repository implements a NMEA 0183 parser library for [SensESP](https://signalk.org/SignalK/SensESP/).
It reads and parses NMEA 0183 formatted data and creates  SensESP Producers for the different data.

To use the library in your own projects, you have to include it in your `platformio.ini` `lib_deps` section:

    lib_deps =
        SignalK/SensESP@>=3.0.0-beta.1,<4.0.0-alpha.1
        SensESP/NMEA0183@^3.0.0

See also the [example main file](blob/main/examples/gnss_receiver.cpp).

For more information on using SensESP and external add-on libraries, see the [SensESP documentation](https://signalk.org/SensESP/docs/).
