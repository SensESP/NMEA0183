#pragma once

#include "sensesp_gnss/gps.h"

namespace sensesp {

GPSInput* setup_gps(Stream* rx_stream);

}  // namespace sensesp
