#pragma once

#include "sensesp_gnss/gps.h"

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
