#include "wiring.h"

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_time.h"
#include "sensesp/transforms/angle_correction.h"

namespace sensesp {

NMEA0183Input* setup_gps(Stream* rx_stream) {
  NMEA0183Input* nmea_input = new NMEA0183Input(rx_stream);
  nmea_input->nmea_data_.position.connect_to(
      new SKOutputPosition("navigation.position", ""));
  nmea_input->nmea_data_.gnss_quality.connect_to(
      new SKOutputString("navigation.methodQuality", ""));
  nmea_input->nmea_data_.num_satellites.connect_to(
      new SKOutputInt("navigation.satellites", ""));
  nmea_input->nmea_data_.horizontal_dilution.connect_to(
      new SKOutputFloat("navigation.horizontalDilution", ""));
  nmea_input->nmea_data_.geoidal_separation.connect_to(
      new SKOutputFloat("navigation.geoidalSeparation", ""));
  nmea_input->nmea_data_.dgps_age.connect_to(
      new SKOutputFloat("navigation.differentialAge", ""));
  nmea_input->nmea_data_.dgps_id.connect_to(
      new SKOutputFloat("navigation.differentialReference", ""));
  nmea_input->nmea_data_.datetime.connect_to(
      new SKOutputTime("navigation.datetime", ""));
  nmea_input->nmea_data_.speed.connect_to(
      new SKOutputFloat("navigation.speedOverGround", ""));
  nmea_input->nmea_data_.true_course.connect_to(
      new SKOutputFloat("navigation.courseOverGroundTrue", ""));
  nmea_input->nmea_data_.variation.connect_to(
      new SKOutputFloat("navigation.magneticVariation", ""));
  nmea_input->nmea_data_.rtk_age.connect_to(
      new SKOutputFloat("navigation.rtkAge", ""));
  nmea_input->nmea_data_.rtk_ratio.connect_to(
      new SKOutputFloat("navigation.rtkRatio", ""));
  nmea_input->nmea_data_.baseline_length.connect_to(
      new SKOutputFloat("navigation.rtkBaselineLength", ""));
  nmea_input->nmea_data_.baseline_course
      .connect_to(new SKOutputFloat("navigation.rtkBaselineCourse"))
      ->connect_to(new AngleCorrection(0, 0, "/sensors/heading/correction"))
      ->connect_to(new SKOutputFloat("navigation.headingTrue", ""));

  return nmea_input;
}

} // namespace sensesp
