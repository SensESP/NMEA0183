#ifndef SENESP_NMEA0183_GNSS_DATA_H
#define SENESP_NMEA0183_GNSS_DATA_H

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp/types/nullable.h"
#include "sensesp/types/position.h"

namespace sensesp::nmea0183 {

/***
 * @brief Enumeration of GNSS systems, used in the GNSSSatellite struct.
 */
enum class GNSSSystem {
  unknown,
  gps,
  glonass,
  galileo,
  beidou,
  qzss,
  sbas,
  irnss,
};

/***
 * @brief Struct to hold information about a single GNSS satellite visibility.
 */
struct GNSSSatellite {
  GNSSSystem system;
  int id;
  sensesp::Nullable<float> elevation;
  sensesp::Nullable<float> azimuth;
  int snr;
  String signal;
};

bool convertToJson(const GNSSSystem& value, JsonVariant& dst);

/**
 * @brief Convenience container for all decoded NMEA 0183 GNSS data.
 */
struct GNSSData {
  ObservableValue<Position> position;
  ObservableValue<String> rtk_quality;
  ObservableValue<int> num_satellites;
  ObservableValue<std::vector<GNSSSatellite>> satellites;
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

bool convertToJson(const GNSSSatellite& value, JsonVariant& dst);

/**
 * @brief Convenience container for RTK-specific GNSS data.
 */
struct RTKData {
  ObservableValue<Position> position;
  ObservableValue<time_t> datetime;
  ObservableValue<ENUVector> enu_velocity;
  ObservableValue<String> rtk_quality;
  ObservableValue<float> rtk_age;
  ObservableValue<float> rtk_ratio;
  ObservableValue<ENUVector> baseline_projection;
  ObservableValue<float> baseline_length;
  ObservableValue<float> baseline_course;
  ObservableValue<AttitudeVector> attitude;
  ObservableValue<int> rtk_num_satellites;
};

}  // namespace sensesp::nmea0183

#endif  // SENESP_NMEA0183_GNSS_DATA_H
