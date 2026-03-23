#include "navigation_sentence_parser.h"

#include "field_parsers.h"

namespace sensesp::nmea0183 {

bool HDGSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  float heading;
  float deviation;
  float variation;

  // $xxHDG,heading,deviation,E/W,variation,E/W*cs
  // eg. $HCHDG,101.1,,,7.1,W*3C

  if (num_fields < 6) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Magnetic sensor heading, degrees
      FLDP_OPT(Float, &heading),
      // 2   Magnetic deviation, degrees
      FLDP_OPT(Float, &deviation),
      // 3   E/W for deviation
      FLDP_OPT(EW, &deviation),
      // 4   Magnetic variation, degrees
      FLDP_OPT(Float, &variation),
      // 5   E/W for variation
      FLDP_OPT(EW, &variation),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (heading != kInvalidFloat) {
    magnetic_heading_.set(heading * DEG_TO_RAD);
  }
  if (deviation != kInvalidFloat) {
    deviation_.set(deviation * DEG_TO_RAD);
  }
  if (variation != kInvalidFloat) {
    variation_.set(variation * DEG_TO_RAD);
  }

  return true;
}

bool VHWSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  float true_heading;
  float magnetic_heading;
  float speed_knots;
  float speed_kmh;
  char t_char;
  char m_char;
  char n_char;
  char k_char;

  // $xxVHW,true_hdg,T,mag_hdg,M,speed_kn,N,speed_kmh,K*cs
  // eg. $VWVHW,045.0,T,042.0,M,5.43,N,10.06,K*4F

  if (num_fields < 9) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   True heading, degrees
      FLDP_OPT(Float, &true_heading),
      // 2   T = True
      FLDP_OPT(Char, &t_char, 'T'),
      // 3   Magnetic heading, degrees
      FLDP_OPT(Float, &magnetic_heading),
      // 4   M = Magnetic
      FLDP_OPT(Char, &m_char, 'M'),
      // 5   Speed, knots
      FLDP_OPT(Float, &speed_knots),
      // 6   N = Knots
      FLDP_OPT(Char, &n_char, 'N'),
      // 7   Speed, km/h
      FLDP_OPT(Float, &speed_kmh),
      // 8   K = km/h
      FLDP_OPT(Char, &k_char, 'K'),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (true_heading != kInvalidFloat) {
    true_heading_.set(true_heading * DEG_TO_RAD);
  }
  if (magnetic_heading != kInvalidFloat) {
    magnetic_heading_.set(magnetic_heading * DEG_TO_RAD);
  }
  // Prefer knots; convert to m/s
  if (speed_knots != kInvalidFloat) {
    water_speed_.set(speed_knots * 1852.0 / 3600.0);
  } else if (speed_kmh != kInvalidFloat) {
    water_speed_.set(speed_kmh * 1000.0 / 3600.0);
  }

  return true;
}

bool DPTSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  float depth;
  float offset;
  float max_range;

  // $xxDPT,depth,offset,max_range*cs
  // eg. $SDDPT,12.6,-0.5,100*42
  // Some instruments omit offset and max_range fields

  if (num_fields < 2) {
    return false;
  }

  ok &= FLDP_OPT(Float, &depth)(field_strings + field_offsets[1]);
  if (num_fields >= 3) {
    ok &= FLDP_OPT(Float, &offset)(field_strings + field_offsets[2]);
  }

  if (!ok) {
    return false;
  }

  if (depth != kInvalidFloat) {
    depth_.set(depth);
  }
  if (offset != kInvalidFloat) {
    offset_.set(offset);
  }

  return true;
}

bool DBTSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  float depth_feet;
  float depth_meters;
  float depth_fathoms;
  char f_char;
  char m_char;
  char F_char;

  // $xxDBT,depth_feet,f,depth_meters,M,depth_fathoms,F*cs
  // eg. $SDDBT,41.3,f,12.6,M,6.9,F*05

  if (num_fields < 7) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Depth, feet
      FLDP_OPT(Float, &depth_feet),
      // 2   f = feet
      FLDP_OPT(Char, &f_char, 'f'),
      // 3   Depth, meters
      FLDP_OPT(Float, &depth_meters),
      // 4   M = meters
      FLDP_OPT(Char, &m_char, 'M'),
      // 5   Depth, fathoms
      FLDP_OPT(Float, &depth_fathoms),
      // 6   F = fathoms
      FLDP_OPT(Char, &F_char, 'F'),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  // Prefer meters; fallback to feet or fathoms
  if (depth_meters != kInvalidFloat) {
    depth_.set(depth_meters);
  } else if (depth_feet != kInvalidFloat) {
    depth_.set(depth_feet * 0.3048);
  } else if (depth_fathoms != kInvalidFloat) {
    depth_.set(depth_fathoms * 1.8288);
  }

  return true;
}

bool MTWSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  float temperature;
  char c_char;

  // $xxMTW,temperature,C*cs
  // eg. $YXMTW,17.8,C*1B

  if (num_fields < 3) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Temperature, Celsius
      FLDP(Float, &temperature),
      // 2   C = Celsius
      FLDP(Char, &c_char, 'C'),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  water_temperature_.set(temperature + 273.15);

  return true;
}

}  // namespace sensesp::nmea0183
