#include "wiring.h"

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_time.h"
#include "sensesp/transforms/angle_correction.h"

namespace sensesp {

void ConnectLocationSKOutputs(NMEA0183Input* nmea_input) {
  nmea_input->nmea_data_.position.connect_to(
      new SKOutputPosition("navigation.position", "/SK Path/Position"));
  nmea_input->nmea_data_.gnss_quality.connect_to(new SKOutputString(
      "navigation.gnss.methodQuality", "/SK Path/Fix Quality"));
  nmea_input->nmea_data_.num_satellites.connect_to(new SKOutputInt(
      "navigation.gnss.satellites", "/SK Path/Number of Satellites"));
  nmea_input->nmea_data_.horizontal_dilution.connect_to(
      new SKOutputFloat("navigation.gnss.horizontalDilution",
                        "/SK Path/Horizontal Dilution"));
  nmea_input->nmea_data_.geoidal_separation.connect_to(new SKOutputFloat(
      "navigation.gnss.geoidalSeparation", "/SK Path/Geoidal Separation"));
  nmea_input->nmea_data_.dgps_age.connect_to(new SKOutputFloat(
      "navigation.gnss.differentialAge", "/SK Path/Differential Age"));
  nmea_input->nmea_data_.dgps_id.connect_to(
      new SKOutputFloat("navigation.gnss.differentialReference",
                        "/SK Path/Differential Reference"));
  nmea_input->nmea_data_.datetime.connect_to(
      new SKOutputTime("navigation.datetime", "/SK Path/DateTime"));
  nmea_input->nmea_data_.speed.connect_to(new SKOutputFloat(
      "navigation.speedOverGround", "/SK Path/Speed Over Ground"));
  nmea_input->nmea_data_.true_course.connect_to(
      new SKOutputFloat("navigation.courseOverGroundTrue",
                        "/SK Path/True Course Over Ground"));
  nmea_input->nmea_data_.variation.connect_to(new SKOutputFloat(
      "navigation.magneticVariation", "/SK Path/Magnetic Variation"));
  nmea_input->nmea_data_.rtk_age.connect_to(new SKOutputFloat(
      "navigation.gnss.rtkAge", "/SK Path/RTK Age",
      new SKMetadata("s", "RTK Solution Age", "The age of the RTK solution",
                     "RTK Age", 30)));
  nmea_input->nmea_data_.rtk_ratio.connect_to(new SKOutputFloat(
      "navigation.gnss.rtkRatio", "/SK Path/RTK Ratio",
      new SKMetadata("", "RTK Ratio", "RTK solution quality indicator",
                     "RTK Ratio", 30)));
  nmea_input->nmea_data_.baseline_length.connect_to(new SKOutputFloat(
      "navigation.gnss.rtkBaselineLength", "/SK Path/RTK Baseline Length",
      new SKMetadata("m", "RTK Baseline Length",
                     "Distance between the RTK antennas", "RTK Baseline Length",
                     30)));
  nmea_input->nmea_data_.baseline_course
      .connect_to(new SKOutputFloat(
          "navigation.gnss.rtkBaselineCourse",
          "/SK Path/RTK Baseline Course",
          new SKMetadata("deg", "RTK Baseline Course",
                         "Angle between baseline vector and north",
                         "RTK Baseline Course", 30)))
      ->connect_to(new AngleCorrection(0, 0, "/RTK/Heading Correction"))
      ->connect_to(new SKOutputFloat("navigation.headingTrue",
                                     "/SK Path/RTK Heading True"));
}

}  // namespace sensesp
