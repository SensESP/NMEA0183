#pragma once

#include "sensesp_gnss/gps.h"

namespace sensesp {

NMEA0183Input* setup_gps(Stream* rx_stream);

}  // namespace sensesp
