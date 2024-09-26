#include "gnss_data.h"

#include "sensesp.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/types/nullable.h"

namespace sensesp::nmea0183 {

bool convertToJson(const GNSSSystem& value, JsonVariant& dst) {
  switch (value) {
    case GNSSSystem::gps:
      dst.set("GPS");
      break;
    case GNSSSystem::glonass:
      dst.set("GLONASS");
      break;
    case GNSSSystem::galileo:
      dst.set("Galileo");
      break;
    case GNSSSystem::beidou:
      dst.set("Beidou");
      break;
    case GNSSSystem::qzss:
      dst.set("QZSS");
      break;
    case GNSSSystem::sbas:
      dst.set("SBAS");
      break;
    case GNSSSystem::irnss:
      dst.set("IRNSS");
      break;
    default:
      dst.set("Unknown");
      break;
  }
  return true;
}

bool convertToJson(const GNSSSatellite& value, JsonVariant& dst) {
  JsonObject obj = dst.to<JsonObject>();
  obj["system"] = value.system;
  obj["id"] = value.id;
  obj["elevation"] = value.elevation;
  obj["azimuth"] = value.azimuth;
  obj["snr"] = value.snr;
  obj["signal"] = value.signal;
  return true;
}

}  // namespace sensesp::nmea0183
