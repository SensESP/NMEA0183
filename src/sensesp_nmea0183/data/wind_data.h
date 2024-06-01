#ifndef SENESP_NMEA0183_WIND_DATA_H
#define SENESP_NMEA0183_WIND_DATA_H

#include "sensesp/system/observablevalue.h"
#include "sensesp/types/position.h"

namespace sensesp {

/**
 * @brief Container for all decoded apparent wind data.
 */
struct ApparentWindData {
  ObservableValue<float> speed;
  ObservableValue<float> angle;
};

}  // namespace sensesp

#endif  // SENESP_NMEA0183_WIND_DATA_H
