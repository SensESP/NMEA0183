#include "waypoint_sentence_parser.h"

#include "field_parsers.h"

namespace sensesp::nmea0183 {

bool RMBSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  bool is_valid;
  float xte;
  char steer_dir;
  String origin_wp;
  String dest_wp;
  double dest_lat;
  double dest_lon;
  float range_nm;
  float bearing;
  float closing_velocity;
  char arrival_status;

  // $xxRMB,A,xte,L/R,origin_wp,dest_wp,lat,N/S,lon,E/W,range,bearing,
  //        closing_vel,arrival_status[,mode]*cs
  // eg. $GPRMB,A,0.66,L,003,004,4917.24,N,12309.57,W,001.3,052.5,000.5,V*20

  if (num_fields < 14) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Status A=active, V=void
      FLDP(AV, &is_valid),
      // 2   Cross-track error, nautical miles
      FLDP_OPT(Float, &xte),
      // 3   Direction to steer, L/R
      FLDP_OPT(Char, &steer_dir, 255),
      // 4   Origin waypoint ID
      FLDP_OPT(String, &origin_wp),
      // 5   Destination waypoint ID
      FLDP_OPT(String, &dest_wp),
      // 6   Destination latitude
      FLDP_OPT(LatLon, &dest_lat),
      // 7   N/S
      FLDP_OPT(NS, &dest_lat),
      // 8   Destination longitude
      FLDP_OPT(LatLon, &dest_lon),
      // 9   E/W
      FLDP_OPT(EW, &dest_lon),
      // 10  Range to destination, nautical miles
      FLDP_OPT(Float, &range_nm),
      // 11  Bearing to destination, degrees true
      FLDP_OPT(Float, &bearing),
      // 12  Destination closing velocity, knots
      FLDP_OPT(Float, &closing_velocity),
      // 13  Arrival status (A=arrived, V=not arrived)
      FLDP_OPT(Char, &arrival_status, 255),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (xte != kInvalidFloat) {
    float signed_xte = xte * 1852.0;  // NM to meters
    if (steer_dir == 'L') {
      signed_xte = -signed_xte;
    }
    cross_track_error_.set(signed_xte);
  }
  if (bearing != kInvalidFloat) {
    bearing_to_destination_.set(bearing * DEG_TO_RAD);
  }
  if (range_nm != kInvalidFloat) {
    range_to_destination_.set(range_nm * 1852.0);
  }
  if (closing_velocity != kInvalidFloat) {
    destination_closing_velocity_.set(closing_velocity * 1852.0 / 3600.0);
  }
  if (dest_wp.length() > 0) {
    destination_waypoint_id_.set(dest_wp);
  }

  return true;
}

bool APBSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  bool status1;
  bool status2;
  float xte;
  char steer_dir;
  char xte_units;
  char arrival_circle;
  char perpendicular;
  float bearing_origin_dest;
  char bearing_od_type;
  String dest_wp;
  float bearing_pos_dest;
  char bearing_pd_type;
  float heading_to_steer;
  char heading_type;

  // $xxAPB,A,A,xte,L/R,N,A/V,A/V,bearing_od,M/T,dest_wp,bearing_pd,M/T,
  //        heading_to_steer,M/T[,mode]*cs
  // eg. $GPAPB,A,A,0.10,R,N,V,V,011,M,DEST,011,M,011,M*82

  if (num_fields < 15) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Status 1 (A=active)
      FLDP(AV, &status1),
      // 2   Status 2 (A=active)
      FLDP(AV, &status2),
      // 3   Cross-track error magnitude
      FLDP_OPT(Float, &xte),
      // 4   Direction to steer, L/R
      FLDP_OPT(Char, &steer_dir, 255),
      // 5   Cross-track error units, N=nautical miles
      FLDP_OPT(Char, &xte_units, 'N'),
      // 6   Arrival circle entered (A/V)
      FLDP_OPT(Char, &arrival_circle, 255),
      // 7   Perpendicular passed (A/V)
      FLDP_OPT(Char, &perpendicular, 255),
      // 8   Bearing origin to destination
      FLDP_OPT(Float, &bearing_origin_dest),
      // 9   M/T (magnetic or true)
      FLDP_OPT(Char, &bearing_od_type, 255),
      // 10  Destination waypoint ID
      FLDP_OPT(String, &dest_wp),
      // 11  Bearing, present position to destination
      FLDP_OPT(Float, &bearing_pos_dest),
      // 12  M/T
      FLDP_OPT(Char, &bearing_pd_type, 255),
      // 13  Heading to steer to destination
      FLDP_OPT(Float, &heading_to_steer),
      // 14  M/T
      FLDP_OPT(Char, &heading_type, 255),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (xte != kInvalidFloat) {
    float signed_xte = xte * 1852.0;  // NM to meters
    if (steer_dir == 'L') {
      signed_xte = -signed_xte;
    }
    cross_track_error_.set(signed_xte);
  }
  if (heading_to_steer != kInvalidFloat) {
    heading_to_steer_.set(heading_to_steer * DEG_TO_RAD);
  }

  return true;
}

bool BWCSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  int hour;
  int minute;
  float second;
  double lat;
  double lon;
  float bearing_true;
  float bearing_mag;
  float distance_nm;
  String waypoint_id;
  char t_char;
  char m_char;
  char n_char;

  // $xxBWC,hhmmss.ss,lat,N/S,lon,E/W,bearing_true,T,bearing_mag,M,
  //        distance_nm,N,waypoint_id[,mode]*cs
  // eg. $GPBWC,220516,5130.02,N,00046.34,W,213.8,T,218.0,M,0004.6,N,EGLM*30

  if (num_fields < 13) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   UTC time
      FLDP_OPT(Time, &hour, &minute, &second),
      // 2   Waypoint latitude
      FLDP_OPT(LatLon, &lat),
      // 3   N/S
      FLDP_OPT(NS, &lat),
      // 4   Waypoint longitude
      FLDP_OPT(LatLon, &lon),
      // 5   E/W
      FLDP_OPT(EW, &lon),
      // 6   Bearing, true
      FLDP_OPT(Float, &bearing_true),
      // 7   T = true
      FLDP_OPT(Char, &t_char, 'T'),
      // 8   Bearing, magnetic
      FLDP_OPT(Float, &bearing_mag),
      // 9   M = magnetic
      FLDP_OPT(Char, &m_char, 'M'),
      // 10  Distance, nautical miles
      FLDP_OPT(Float, &distance_nm),
      // 11  N = nautical miles
      FLDP_OPT(Char, &n_char, 'N'),
      // 12  Waypoint ID
      FLDP_OPT(String, &waypoint_id),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  if (bearing_true != kInvalidFloat) {
    bearing_true_.set(bearing_true * DEG_TO_RAD);
  }
  if (bearing_mag != kInvalidFloat) {
    bearing_magnetic_.set(bearing_mag * DEG_TO_RAD);
  }
  if (distance_nm != kInvalidFloat) {
    distance_.set(distance_nm * 1852.0);
  }
  if (waypoint_id.length() > 0) {
    waypoint_id_.set(waypoint_id);
  }
  if (lat != kInvalidDouble && lon != kInvalidDouble) {
    Position pos;
    pos.latitude = lat;
    pos.longitude = lon;
    pos.altitude = kPositionInvalidAltitude;
    waypoint_position_.set(pos);
  }

  return true;
}

bool WPLSentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  double lat;
  double lon;
  String waypoint_id;

  // $xxWPL,lat,N/S,lon,E/W,waypoint_id*cs
  // eg. $GPWPL,4917.16,N,12310.64,W,003*65

  if (num_fields < 6) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Latitude
      FLDP(LatLon, &lat),
      // 2   N/S
      FLDP(NS, &lat),
      // 3   Longitude
      FLDP(LatLon, &lon),
      // 4   E/W
      FLDP(EW, &lon),
      // 5   Waypoint ID
      FLDP(String, &waypoint_id),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  Position pos;
  pos.latitude = lat;
  pos.longitude = lon;
  pos.altitude = kPositionInvalidAltitude;
  position_.set(pos);
  waypoint_id_.set(waypoint_id);

  return true;
}

}  // namespace sensesp::nmea0183
