#ifndef _signalk_position_H_
#define _signalk_position_H_

#include <set>

#include "sensesp_gnss/gps.h"
#include "sensesp/signalk/signalk_output.h"

#include "sensesp_gnss/position.h"

namespace sensesp {

///////////////////
// provide correct output formatting for GNSS position

template <>
String SKOutput<Position>::as_signalk();

typedef SKOutput<Position> SKOutputPosition;

}

#endif
