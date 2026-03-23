#include "wiring.h"

#include <memory>

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_time.h"
#include "sensesp/transforms/angle_correction.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/types/json.h"
#include "sensesp_nmea0183/data/gnss_data.h"
#include "sensesp_nmea0183/data/wind_data.h"
#include "sensesp_nmea0183/sentence_parser/gnss_sentence_parser.h"
#include "sensesp_nmea0183/sentence_parser/navigation_sentence_parser.h"
#include "sensesp_nmea0183/sentence_parser/weather_sentence_parser.h"
#include "sensesp_nmea0183/sentence_parser/wind_sentence_parser.h"

namespace sensesp::nmea0183 {

void ConnectGNSS(NMEA0183Parser* nmea_input, GNSSData* location_data) {
  GGASentenceParser* gga_sentence_parser = new GGASentenceParser(nmea_input);

  GLLSentenceParser* gll_sentence_parser = new GLLSentenceParser(nmea_input);

  RMCSentenceParser* rmc_sentence_parser = new RMCSentenceParser(nmea_input);

  VTGSentenceParser* vtg_sentence_parser = new VTGSentenceParser(nmea_input);

  GSVSentenceParser* gsv_sentence_parser = new GSVSentenceParser(nmea_input);

  auto* gsa_sentence_parser = new GSASentenceParser(nmea_input);

  auto* zda_sentence_parser = new ZDASentenceParser(nmea_input);

  gga_sentence_parser->position_.connect_to(&location_data->position);
  gga_sentence_parser->gnss_quality_.connect_to(&location_data->rtk_quality);
  gga_sentence_parser->num_satellites_.connect_to(
      &location_data->num_satellites);
  gga_sentence_parser->horizontal_dilution_.connect_to(
      &location_data->horizontal_dilution);
  gga_sentence_parser->geoidal_separation_.connect_to(
      &location_data->geoidal_separation);
  gga_sentence_parser->dgps_age_.connect_to(&location_data->dgps_age);
  gga_sentence_parser->dgps_id_.connect_to(&location_data->dgps_id);

  gll_sentence_parser->position_.connect_to(&location_data->position);

  rmc_sentence_parser->position_.connect_to(&location_data->position);
  rmc_sentence_parser->datetime_.connect_to(&location_data->datetime);
  rmc_sentence_parser->speed_.connect_to(&location_data->speed);
  rmc_sentence_parser->variation_.connect_to(&location_data->variation);

  vtg_sentence_parser->true_course_.connect_to(&location_data->true_course);

  gsv_sentence_parser->num_satellites_.connect_to(
      &location_data->num_satellites);
  gsv_sentence_parser->satellites_.connect_to(&location_data->satellites);

  gsa_sentence_parser->fix_type_.connect_to(&location_data->fix_type);
  gsa_sentence_parser->hdop_.connect_to(&location_data->horizontal_dilution);
  gsa_sentence_parser->pdop_.connect_to(&location_data->pdop);
  gsa_sentence_parser->vdop_.connect_to(&location_data->vdop);

  zda_sentence_parser->datetime_.connect_to(&location_data->datetime);

  location_data->position.connect_to(
      new SKOutput<Position>("navigation.position", "/SK Path/Position"));
  location_data->rtk_quality.connect_to(new SKOutputString(
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
  location_data->satellites.connect_to(new SKOutput<std::vector<GNSSSatellite>>(
      "navigation.gnss.satellitesInView", "/SK Path/Satellites in View"));
}

void ConnectSkyTraqRTK(NMEA0183Parser* nmea_input, RTKData* rtk_data) {
  SkyTraqPSTI030SentenceParser* psti030_sentence_parser =
      new SkyTraqPSTI030SentenceParser(nmea_input);

  SkyTraqPSTI032SentenceParser* psti032_sentence_parser =
      new SkyTraqPSTI032SentenceParser(nmea_input);

  psti030_sentence_parser->position_.connect_to(&rtk_data->position);
  psti030_sentence_parser->datetime_.connect_to(&rtk_data->datetime);
  psti030_sentence_parser->enu_velocity_.connect_to(&rtk_data->enu_velocity);
  psti030_sentence_parser->gnss_quality_.connect_to(&rtk_data->rtk_quality);
  psti030_sentence_parser->rtk_age_.connect_to(&rtk_data->rtk_age);
  psti030_sentence_parser->rtk_ratio_.connect_to(&rtk_data->rtk_ratio);

  psti032_sentence_parser->baseline_projection_.connect_to(
      &rtk_data->baseline_projection);
  psti032_sentence_parser->baseline_length_.connect_to(
      &rtk_data->baseline_length);
  psti032_sentence_parser->baseline_course_.connect_to(
      &rtk_data->baseline_course);

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

void ConnectQuectelRTK(NMEA0183Parser* nmea_input, RTKData* rtk_data) {
  QuectelPQTMTARSentenceParser* pqtmtar_sentence_parser =
      new QuectelPQTMTARSentenceParser(nmea_input);

  pqtmtar_sentence_parser->datetime_.connect_to(&rtk_data->datetime);
  pqtmtar_sentence_parser->rtk_quality_.connect_to(&rtk_data->rtk_quality);
  pqtmtar_sentence_parser->baseline_length_.connect_to(
      &rtk_data->baseline_length);
  pqtmtar_sentence_parser->attitude_
      .connect_to(new LambdaTransform<sensesp::AttitudeVector, float>(
          [](const AttitudeVector& attitude) { return attitude.yaw; }))
      ->connect_to(new SKOutputFloat("navigation.gnss.rtkBaselineCourse",
                                     "/SK Path/RTK Yaw"))
      ->connect_to(new AngleCorrection(0, 0, "/RTK/Heading Correction"))
      ->connect_to(new SKOutputFloat("navigation.headingTrue",
                                     "/SK Path/RTK Heading True"));
  pqtmtar_sentence_parser->attitude_.connect_to(&rtk_data->attitude);
  pqtmtar_sentence_parser->hdg_num_satellites_.connect_to(
      &rtk_data->rtk_num_satellites);

  rtk_data->rtk_age.connect_to(new SKOutputFloat(
      "navigation.gnss.rtkAge", "/SK Path/RTK Age",
      new SKMetadata("s", "RTK Solution Age", "The age of the RTK solution",
                     "RTK Age", 30)));
  rtk_data->rtk_ratio.connect_to(new SKOutputFloat(
      "navigation.gnss.rtkRatio", "/SK Path/RTK Ratio",
      new SKMetadata("", "RTK Ratio", "RTK solution quality indicator",
                     "RTK Ratio", 30)));
}

void ConnectApparentWind(NMEA0183Parser* nmea_input,
                         ApparentWindData* apparent_wind_data) {
  WIMWVSentenceParser* wind_sentence_parser =
      new WIMWVSentenceParser(nmea_input);

  wind_sentence_parser->apparent_wind_speed_.connect_to(
      &apparent_wind_data->speed);
  wind_sentence_parser->apparent_wind_angle_.connect_to(
      &apparent_wind_data->angle);

  apparent_wind_data->angle.connect_to(new SKOutputFloat(
      "environment.wind.angleApparent", "/SK Path/Apparent Wind Angle"));
  apparent_wind_data->speed.connect_to(new SKOutputFloat(
      "environment.wind.speedApparent", "/SK Path/Apparent Wind Speed"));
}

void ConnectDepthTemperature(NMEA0183Parser* nmea_input,
                             DepthTemperatureData* data) {
  auto* dbt = new DBTSentenceParser(nmea_input);
  auto* mtw = new MTWSentenceParser(nmea_input);

  dbt->depth_.connect_to(&data->depth_below_transducer);
  mtw->water_temperature_.connect_to(&data->water_temperature);

  data->depth_below_transducer.connect_to(new SKOutputFloat(
      "environment.depth.belowTransducer",
      "/SK Path/Depth Below Transducer"));
  data->water_temperature.connect_to(new SKOutputFloat(
      "environment.water.temperature",
      "/SK Path/Water Temperature"));
}

void ConnectHeading(NMEA0183Parser* nmea_input, HeadingData* data) {
  auto* hdm = new HDMSentenceParser(nmea_input);
  auto* hdt = new HDTSentenceParser(nmea_input);

  hdm->magnetic_heading_.connect_to(&data->magnetic_heading);
  hdt->true_heading_.connect_to(&data->true_heading);

  data->magnetic_heading.connect_to(new SKOutputFloat(
      "navigation.headingMagnetic", "/SK Path/Heading Magnetic"));
  data->true_heading.connect_to(new SKOutputFloat(
      "navigation.headingTrue", "/SK Path/Heading True"));
}

void ConnectTrueWind(NMEA0183Parser* nmea_input, TrueWindData* data) {
  auto* mwd = new MWDSentenceParser(nmea_input);

  mwd->true_wind_direction_.connect_to(&data->direction);
  mwd->true_wind_speed_.connect_to(&data->speed);

  data->direction.connect_to(new SKOutputFloat(
      "environment.wind.directionTrue", "/SK Path/True Wind Direction"));
  data->speed.connect_to(new SKOutputFloat(
      "environment.wind.speedTrue", "/SK Path/True Wind Speed"));
}

void ConnectWeather(NMEA0183Parser* nmea_input, WeatherData* data) {
  auto* mda = new MDASentenceParser(nmea_input);

  mda->barometric_pressure_.connect_to(&data->barometric_pressure);
  mda->air_temperature_.connect_to(&data->air_temperature);
  mda->water_temperature_.connect_to(&data->water_temperature);
  mda->relative_humidity_.connect_to(&data->relative_humidity);
  mda->dew_point_.connect_to(&data->dew_point);
  mda->true_wind_direction_.connect_to(&data->true_wind_direction);
  mda->true_wind_speed_.connect_to(&data->true_wind_speed);

  data->barometric_pressure.connect_to(new SKOutputFloat(
      "environment.outside.pressure", "/SK Path/Barometric Pressure"));
  data->air_temperature.connect_to(new SKOutputFloat(
      "environment.outside.temperature", "/SK Path/Air Temperature"));
  data->water_temperature.connect_to(new SKOutputFloat(
      "environment.water.temperature", "/SK Path/Water Temperature (MDA)"));
  data->relative_humidity.connect_to(new SKOutputFloat(
      "environment.outside.humidity", "/SK Path/Relative Humidity"));
  data->dew_point.connect_to(new SKOutputFloat(
      "environment.outside.dewPointTemperature", "/SK Path/Dew Point"));
  data->true_wind_direction.connect_to(new SKOutputFloat(
      "environment.wind.directionTrue", "/SK Path/True Wind Direction (MDA)"));
  data->true_wind_speed.connect_to(new SKOutputFloat(
      "environment.wind.speedTrue", "/SK Path/True Wind Speed (MDA)"));
}

}  // namespace sensesp::nmea0183
