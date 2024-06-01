#include "wiring.h"

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_time.h"
#include "sensesp/transforms/angle_correction.h"
#include "sensesp_nmea0183/data/gnss_data.h"
#include "sensesp_nmea0183/data/wind_data.h"
#include "sensesp_nmea0183/sentence_parser/gnss_sentence_parser.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"

namespace sensesp {

void ConnectGNSS(NMEA0183Input* nmea_input, GNSSData* location_data) {
  GGASentenceParser* gga_sentence_parser = new GGASentenceParser(
      nmea_input, &location_data->position, &location_data->gnss_quality,
      &location_data->num_satellites, &location_data->horizontal_dilution,
      &location_data->geoidal_separation, &location_data->dgps_age,
      &location_data->dgps_id);

  GLLSentenceParser* gll_sentence_parser =
      new GLLSentenceParser(nmea_input, &location_data->position);

  RMCSentenceParser* rmc_sentence_parser = new RMCSentenceParser(
      nmea_input, &location_data->position, &location_data->datetime,
      &location_data->speed, &location_data->true_course,
      &location_data->variation);

  VTGSentenceParser* vtg_sentence_parser = new VTGSentenceParser(
      nmea_input, &location_data->true_course, &location_data->speed);

  location_data->position.connect_to(
      new SKOutputPosition("navigation.position", "/SK Path/Position"));
  location_data->gnss_quality.connect_to(new SKOutputString(
      "navigation.gnss.methodQuality", "/SK Path/Fix Quality"));
  location_data->num_satellites.connect_to(new SKOutputInt(
      "navigation.gnss.satellites", "/SK Path/Number of Satellites"));
  location_data->horizontal_dilution.connect_to(new SKOutputFloat(
      "navigation.gnss.horizontalDilution", "/SK Path/Horizontal Dilution"));
  location_data->geoidal_separation.connect_to(new SKOutputFloat(
      "navigation.gnss.geoidalSeparation", "/SK Path/Geoidal Separation"));
  location_data->dgps_age.connect_to(new SKOutputFloat(
      "navigation.gnss.differentialAge", "/SK Path/Differential Age"));
  location_data->dgps_id.connect_to(
      new SKOutputFloat("navigation.gnss.differentialReference",
                        "/SK Path/Differential Reference"));
  location_data->datetime.connect_to(
      new SKOutputTime("navigation.datetime", "/SK Path/DateTime"));
  location_data->speed.connect_to(new SKOutputFloat(
      "navigation.speedOverGround", "/SK Path/Speed Over Ground"));
  location_data->true_course.connect_to(new SKOutputFloat(
      "navigation.courseOverGroundTrue", "/SK Path/True Course Over Ground"));
  location_data->variation.connect_to(new SKOutputFloat(
      "navigation.magneticVariation", "/SK Path/Magnetic Variation"));
}

void ConnectRTK(NMEA0183Input* nmea_input, RTKData* rtk_data) {
  PSTI030SentenceParser* psti030_sentence_parser = new PSTI030SentenceParser(
      nmea_input, &rtk_data->position, &rtk_data->datetime,
      &rtk_data->enu_velocity, &rtk_data->gnss_quality, &rtk_data->rtk_age,
      &rtk_data->rtk_ratio);

  PSTI032SentenceParser* psti032_sentence_parser = new PSTI032SentenceParser(
      nmea_input, &rtk_data->datetime, &rtk_data->baseline_projection,
      &rtk_data->baseline_length, &rtk_data->baseline_course,
      &rtk_data->gnss_quality);

  rtk_data->rtk_age.connect_to(new SKOutputFloat(
      "navigation.gnss.rtkAge", "/SK Path/RTK Age",
      new SKMetadata("s", "RTK Solution Age", "The age of the RTK solution",
                     "RTK Age", 30)));
  rtk_data->rtk_ratio.connect_to(new SKOutputFloat(
      "navigation.gnss.rtkRatio", "/SK Path/RTK Ratio",
      new SKMetadata("", "RTK Ratio", "RTK solution quality indicator",
                     "RTK Ratio", 30)));
  rtk_data->baseline_length.connect_to(new SKOutputFloat(
      "navigation.gnss.rtkBaselineLength", "/SK Path/RTK Baseline Length",
      new SKMetadata("m", "RTK Baseline Length",
                     "Distance between the RTK antennas", "RTK Baseline Length",
                     30)));
  rtk_data->baseline_course
      .connect_to(new SKOutputFloat(
          "navigation.gnss.rtkBaselineCourse", "/SK Path/RTK Baseline Course",
          new SKMetadata("deg", "RTK Baseline Course",
                         "Angle between baseline vector and north",
                         "RTK Baseline Course", 30)))
      ->connect_to(new AngleCorrection(0, 0, "/RTK/Heading Correction"))
      ->connect_to(new SKOutputFloat("navigation.headingTrue",
                                     "/SK Path/RTK Heading True"));
}

void ConnectApparentWind(NMEA0183Input* nmea_input,
                         ApparentWindData* apparent_wind_data) {
  WIMWVSentenceParser* wind_sentence_parser = new WIMWVSentenceParser(
      nmea_input, &apparent_wind_data->speed, &apparent_wind_data->angle);

  apparent_wind_data->angle.connect_to(new SKOutputFloat(
      "environment.wind.angleApparent", "/SK Path/Apparent Wind Angle"));
  apparent_wind_data->speed.connect_to(new SKOutputFloat(
      "environment.wind.speedApparent", "/SK Path/Apparent Wind Speed"));
}

}  // namespace sensesp
