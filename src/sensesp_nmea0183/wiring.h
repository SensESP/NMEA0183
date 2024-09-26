#ifndef SENSEP_NMEA0183_WIRING_H
#define SENSEP_NMEA0183_WIRING_H

#include "sensesp_nmea0183/data/gnss_data.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/data/wind_data.h"

namespace sensesp::nmea0183 {

/**
 * @brief GNSSData observable members to SK outputs.
 *
 * @param nmea_input
 */
void ConnectGNSS(NMEA0183* nmea_input, GNSSData* location_data);

/**
 * @brief Wire the SkyTraq RTK Data observable members to SK outputs.
 *
 * @param nmea_input
 * @param rtk_data
 */
void ConnectSkyTraqRTK(NMEA0183* nmea_input, RTKData* rtk_data);

/**
 * @brief Wire the Quectel RTK Data observable members to SK outputs.
 *
 * @param nmea_input
 * @param rtk_data
 */
void ConnectQuectelRTK(NMEA0183* nmea_input, RTKData* rtk_data);

/**
 * @brief Wire the ApparentWindData observable members to SK outputs.
 *
 */
void ConnectApparentWind(NMEA0183* nmea_input,
                         ApparentWindData* apparent_wind_data);

}  // namespace sensesp

#endif  // SENSEP_NMEA0183_WIRING_H
