#ifndef SENSEP_NMEA0183_WIRING_H
#define SENSEP_NMEA0183_WIRING_H

#include "sensesp_nmea0183/data/gnss_data.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/data/wind_data.h"

namespace sensesp {

/**
 * @brief GNSSData observable members to SK outputs.
 *
 * @param nmea_input
 */
void ConnectGNSS(NMEA0183* nmea_input, GNSSData* location_data);

/**
 * @brief Wire the RTKData observable members to SK outputs.
 *
 * @param nmea_input
 * @param rtk_data
 */
void ConnectRTK(NMEA0183* nmea_input, RTKData* rtk_data);

/**
 * @brief Wire the ApparentWindData observable members to SK outputs.
 *
 */
void ConnectApparentWind(NMEA0183* nmea_input,
                         ApparentWindData* apparent_wind_data);

}  // namespace sensesp

#endif  // SENSEP_NMEA0183_WIRING_H
