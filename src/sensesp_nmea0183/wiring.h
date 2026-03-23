#ifndef SENSEP_NMEA0183_WIRING_H
#define SENSEP_NMEA0183_WIRING_H

#include "sensesp_nmea0183/data/gnss_data.h"
#include "sensesp_nmea0183/data/navigation_data.h"
#include "sensesp_nmea0183/data/waypoint_data.h"
#include "sensesp_nmea0183/data/weather_data.h"
#include "sensesp_nmea0183/data/wind_data.h"
#include "sensesp_nmea0183/nmea0183.h"

namespace sensesp::nmea0183 {

/**
 * @brief GNSSData observable members to SK outputs.
 *
 * @param nmea_input
 */
void ConnectGNSS(NMEA0183Parser* nmea_input, GNSSData* location_data);

/**
 * @brief Wire the SkyTraq RTK Data observable members to SK outputs.
 *
 * @param nmea_input
 * @param rtk_data
 */
void ConnectSkyTraqRTK(NMEA0183Parser* nmea_input, RTKData* rtk_data);

/**
 * @brief Wire the Quectel RTK Data observable members to SK outputs.
 *
 * @param nmea_input
 * @param rtk_data
 */
void ConnectQuectelRTK(NMEA0183Parser* nmea_input, RTKData* rtk_data);

/**
 * @brief Wire the ApparentWindData observable members to SK outputs.
 *
 */
void ConnectApparentWind(NMEA0183Parser* nmea_input,
                         ApparentWindData* apparent_wind_data);

/**
 * @brief Wire DBT and MTW parsers to Signal K outputs.
 */
void ConnectDepthTemperature(NMEA0183Parser* nmea_input,
                             DepthTemperatureData* data);

/**
 * @brief Wire HDM and HDT parsers to Signal K outputs.
 */
void ConnectHeading(NMEA0183Parser* nmea_input, HeadingData* data);

/**
 * @brief Wire MWD parser to Signal K outputs for true wind data.
 */
void ConnectTrueWind(NMEA0183Parser* nmea_input, TrueWindData* data);

/**
 * @brief Wire MDA parser to Signal K outputs for weather data.
 */
void ConnectWeather(NMEA0183Parser* nmea_input, WeatherData* data);

/**
 * @brief Wire RMB, BWC, and APB parsers to Signal K outputs.
 */
void ConnectWaypoint(NMEA0183Parser* nmea_input, WaypointData* data);

}  // namespace sensesp

#endif  // SENSEP_NMEA0183_WIRING_H
