#ifndef SENESP_NMEA0183_WIND_DATA_H
#define SENESP_NMEA0183_WIND_DATA_H

#include "sensesp/system/observablevalue.h"
#include "sensesp/types/position.h"

namespace sensesp::nmea0183 {

/**
 * @brief Container for all decoded apparent wind data.
 */
struct ApparentWindData {
  ObservableValue<float> speed;
  ObservableValue<float> angle;
};

/**
 * @brief Container for all decoded true wind data.
 */
struct TrueWindData {
  ObservableValue<float> direction;  // radians (true, meteorological: where FROM)
  ObservableValue<float> speed;      // m/s
};

}  // namespace sensesp

#endif  // SENESP_NMEA0183_WIND_DATA_H
