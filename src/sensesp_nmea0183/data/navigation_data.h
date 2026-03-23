#ifndef SENSESP_NMEA0183_NAVIGATION_DATA_H_
#define SENSESP_NMEA0183_NAVIGATION_DATA_H_

#include "sensesp/system/observablevalue.h"

namespace sensesp::nmea0183 {

/**
 * @brief Container for depth and water temperature data.
 */
struct DepthTemperatureData {
  ObservableValue<float> depth_below_transducer;  // meters
  ObservableValue<float> water_temperature;        // Kelvin
};

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_NAVIGATION_DATA_H_
