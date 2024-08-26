#ifndef SENESP_NMEA0183_GNSS_DATA_H
#define SENESP_NMEA0183_GNSS_DATA_H

#include "sensesp/system/observablevalue.h"
#include "sensesp/types/position.h"

namespace sensesp::nmea0183 {

/**
 * @brief Convenience container for all decoded NMEA 0183 GNSS data.
 */
struct GNSSData {
  ObservableValue<Position> position;
  ObservableValue<String> gnss_quality;
  ObservableValue<int> num_satellites;
  ObservableValue<float> horizontal_dilution;
  ObservableValue<float> geoidal_separation;
  ObservableValue<float> dgps_age;
  ObservableValue<int> dgps_id;
  ObservableValue<time_t> datetime;
  ObservableValue<float> speed;
  ObservableValue<float> true_course;
  ObservableValue<float> variation;
  ObservableValue<ENUVector> enu_velocity;
};

/**
 * @brief Convenience container for RTK-specific GNSS data.
 */
struct RTKData {
  ObservableValue<Position> position;
  ObservableValue<time_t> datetime;
  ObservableValue<ENUVector> enu_velocity;
  ObservableValue<String> gnss_quality;
  ObservableValue<float> rtk_age;
  ObservableValue<float> rtk_ratio;
  ObservableValue<ENUVector> baseline_projection;
  ObservableValue<float> baseline_length;
  ObservableValue<float> baseline_course;
};

}  // namespace sensesp

#endif  // SENESP_NMEA0183_GNSS_DATA_H
