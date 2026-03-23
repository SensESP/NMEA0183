#ifndef SENSESP_NMEA0183_WEATHER_DATA_H_
#define SENSESP_NMEA0183_WEATHER_DATA_H_

#include "sensesp/system/observablevalue.h"

namespace sensesp::nmea0183 {

/**
 * @brief Container for meteorological data from MDA parser.
 */
struct WeatherData {
  ObservableValue<float> barometric_pressure;  // Pascals
  ObservableValue<float> air_temperature;      // Kelvin
  ObservableValue<float> water_temperature;    // Kelvin
  ObservableValue<float> relative_humidity;    // ratio (0-1)
  ObservableValue<float> dew_point;            // Kelvin
  ObservableValue<float> true_wind_direction;  // radians
  ObservableValue<float> true_wind_speed;      // m/s
};

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_WEATHER_DATA_H_
