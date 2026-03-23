#ifndef SENSESP_NMEA0183_WAYPOINT_DATA_H_
#define SENSESP_NMEA0183_WAYPOINT_DATA_H_

#include "sensesp/system/observablevalue.h"
#include "sensesp/types/position.h"

namespace sensesp::nmea0183 {

/**
 * @brief Container for waypoint navigation data.
 */
struct WaypointData {
  ObservableValue<float> cross_track_error;           // meters (signed)
  ObservableValue<float> bearing_to_destination;      // radians (true)
  ObservableValue<float> range_to_destination;        // meters
  ObservableValue<float> destination_closing_velocity; // m/s
  ObservableValue<String> destination_waypoint_id;
  ObservableValue<Position> waypoint_position;
  ObservableValue<float> heading_to_steer;            // radians (true)
  // Great circle specific
  ObservableValue<float> gc_bearing_true;             // radians
  ObservableValue<float> gc_distance;                 // meters
};

}  // namespace sensesp::nmea0183

#endif  // SENSESP_NMEA0183_WAYPOINT_DATA_H_
