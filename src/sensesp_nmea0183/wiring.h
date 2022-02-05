#pragma once

#include "sensesp_nmea0183/nmea0183.h"

namespace sensesp {

/**
 * @brief Wire the NMEALocationData observable members to SK outputs.
 *
 *
 *
 * @param nmea_input
 */
void ConnectLocationSKOutputs(NMEA0183Input* nmea_input);

}  // namespace sensesp
