#include "wiring.h"

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_time.h"
#include "sensesp/transforms/angle_correction.h"

namespace sensesp {

GPSInput* setup_gps(Stream* rx_stream) {
  GPSInput* gps = new GPSInput(rx_stream);
  gps->nmea_data_.position.connect_to(
      new SKOutputPosition("navigation.position", ""));
  gps->nmea_data_.gnss_quality.connect_to(
      new SKOutputString("navigation.methodQuality", ""));
  gps->nmea_data_.num_satellites.connect_to(
      new SKOutputInt("navigation.satellites", ""));
  gps->nmea_data_.horizontal_dilution.connect_to(
      new SKOutputFloat("navigation.horizontalDilution", ""));
  gps->nmea_data_.geoidal_separation.connect_to(
      new SKOutputFloat("navigation.geoidalSeparation", ""));
  gps->nmea_data_.dgps_age.connect_to(
      new SKOutputFloat("navigation.differentialAge", ""));
  gps->nmea_data_.dgps_id.connect_to(
      new SKOutputFloat("navigation.differentialReference", ""));
  gps->nmea_data_.datetime.connect_to(
      new SKOutputTime("navigation.datetime", ""));
  gps->nmea_data_.speed.connect_to(
      new SKOutputFloat("navigation.speedOverGround", ""));
  gps->nmea_data_.true_course.connect_to(
      new SKOutputFloat("navigation.courseOverGroundTrue", ""));
  gps->nmea_data_.variation.connect_to(
      new SKOutputFloat("navigation.magneticVariation", ""));
  gps->nmea_data_.rtk_age.connect_to(
      new SKOutputFloat("navigation.rtkAge", ""));
  gps->nmea_data_.rtk_ratio.connect_to(
      new SKOutputFloat("navigation.rtkRatio", ""));
  gps->nmea_data_.baseline_length.connect_to(
      new SKOutputFloat("navigation.rtkBaselineLength", ""));
  gps->nmea_data_.baseline_course
      .connect_to(new SKOutputFloat("navigation.rtkBaselineCourse"))
      ->connect_to(new AngleCorrection(0, 0, "/sensors/heading/correction"))
      ->connect_to(new SKOutputFloat("navigation.headingTrue", ""));

  return gps;
}

} // namespace sensesp
