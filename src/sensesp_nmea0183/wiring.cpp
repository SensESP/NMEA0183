#include "wiring.h"

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_time.h"
#include "sensesp/transforms/angle_correction.h"

namespace sensesp {

void ConnectLocationSKOutputs(NMEA0183Input* nmea_input) {
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
  nmea_input->nmea_data_.rtk_age.connect_to(new SKOutputFloat(
      "navigation.rtkAge", "",
      new SKMetadata("s", "RTK Solution Age", "The age of the RTK solution",
                     "RTK Age", 30)));
  nmea_input->nmea_data_.rtk_ratio.connect_to(new SKOutputFloat(
      "navigation.rtkRatio", "",
      new SKMetadata("", "RTK Ratio", "RTK solution quality indicator",
                     "RTK Ratio", 30)));
  nmea_input->nmea_data_.baseline_length.connect_to(
      new SKOutputFloat("navigation.rtkBaselineLength", "",
                        new SKMetadata("m", "RTK Baseline Length",
                                       "Distance between the RTK antennas",
                                       "RTK Baseline Length", 30)));
  nmea_input->nmea_data_.baseline_course
      .connect_to(new SKOutputFloat(
          "navigation.rtkBaselineCourse",
          new SKMetadata("deg", "RTK Baseline Course",
                         "Angle between baseline vector and north",
                         "RTK Baseline Course", 30)))
      ->connect_to(new AngleCorrection(0, 0, "/sensors/heading/correction"))
      ->connect_to(new SKOutputFloat("navigation.headingTrue", ""));
}

}  // namespace sensesp
