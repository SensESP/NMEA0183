#include "weather_sentence_parser.h"

#include "field_parsers.h"
#include "sensesp/types/nullable.h"
using sensesp::Nullable;

namespace sensesp::nmea0183 {

bool MDASentenceParser::parse_fields(const char* field_strings,
                                     const int field_offsets[],
                                     int num_fields) {
  bool ok = true;

  Nullable<float> pressure_inhg;
  Nullable<float> pressure_bars;
  Nullable<float> air_temp;
  Nullable<float> water_temp;
  Nullable<float> humidity;
  Nullable<float> dew_point;
  Nullable<float> wind_dir_true;
  Nullable<float> wind_dir_mag;
  Nullable<float> abs_humidity;
  Nullable<float> wind_speed_kn;
  Nullable<float> wind_speed_ms;
  char i_char;
  char b_char;
  char c1_char;
  char c2_char;
  char c3_char;
  char t_char;
  char m_char;
  char n_char;
  char ms_char;

  // $xxMDA,press_inhg,I,press_bars,B,air_temp,C,water_temp,C,
  //        humidity,,dew_point,C,wind_dir_true,T,wind_dir_mag,M,
  //        wind_speed_kn,N,wind_speed_ms,M*cs
  // eg. $WIMDA,29.7544,I,1.0076,B,16.1,C,,,42.6,,11.2,C,225.0,T,220.0,M,12.5,N,6.4,M*58

  if (num_fields < 21) {
    return false;
  }

  std::function<bool(const char*)> fps[] = {
      // 1   Barometric pressure, inches of mercury
      FLDP_OPT(Float, pressure_inhg.ptr()),
      // 2   I = inches of mercury
      FLDP_OPT(Char, &i_char, 'I'),
      // 3   Barometric pressure, bars
      FLDP_OPT(Float, pressure_bars.ptr()),
      // 4   B = bars
      FLDP_OPT(Char, &b_char, 'B'),
      // 5   Air temperature, Celsius
      FLDP_OPT(Float, air_temp.ptr()),
      // 6   C = Celsius
      FLDP_OPT(Char, &c1_char, 'C'),
      // 7   Water temperature, Celsius
      FLDP_OPT(Float, water_temp.ptr()),
      // 8   C = Celsius
      FLDP_OPT(Char, &c2_char, 'C'),
      // 9   Relative humidity, percent
      FLDP_OPT(Float, humidity.ptr()),
      // 10  Absolute humidity (rarely populated, ignored)
      FLDP_OPT(Float, abs_humidity.ptr()),
      // 11  Dew point, Celsius
      FLDP_OPT(Float, dew_point.ptr()),
      // 12  C = Celsius
      FLDP_OPT(Char, &c3_char, 'C'),
      // 13  Wind direction, degrees true
      FLDP_OPT(Float, wind_dir_true.ptr()),
      // 14  T = true
      FLDP_OPT(Char, &t_char, 'T'),
      // 15  Wind direction, degrees magnetic
      FLDP_OPT(Float, wind_dir_mag.ptr()),
      // 16  M = magnetic
      FLDP_OPT(Char, &m_char, 'M'),
      // 17  Wind speed, knots
      FLDP_OPT(Float, wind_speed_kn.ptr()),
      // 18  N = knots
      FLDP_OPT(Char, &n_char, 'N'),
      // 19  Wind speed, m/s
      FLDP_OPT(Float, wind_speed_ms.ptr()),
      // 20  M = m/s
      FLDP_OPT(Char, &ms_char, 'M'),
  };

  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }

  if (!ok) {
    return false;
  }

  // Prefer bars; fallback to inches of mercury
  if (pressure_bars.is_valid()) {
    barometric_pressure_.set(pressure_bars * 100000.0);
  } else if (pressure_inhg.is_valid()) {
    barometric_pressure_.set(pressure_inhg * 3386.389);
  }

  if (air_temp.is_valid()) {
    air_temperature_.set(air_temp + 273.15);
  }
  if (water_temp.is_valid()) {
    water_temperature_.set(water_temp + 273.15);
  }
  if (humidity.is_valid()) {
    relative_humidity_.set(humidity / 100.0);
  }
  if (dew_point.is_valid()) {
    dew_point_.set(dew_point + 273.15);
  }
  if (wind_dir_true.is_valid()) {
    true_wind_direction_.set(wind_dir_true * DEG_TO_RAD);
  }
  // Prefer m/s; fallback to knots
  if (wind_speed_ms.is_valid()) {
    true_wind_speed_.set(wind_speed_ms);
  } else if (wind_speed_kn.is_valid()) {
    true_wind_speed_.set(wind_speed_kn * 1852.0 / 3600.0);
  }

  return true;
}

}  // namespace sensesp::nmea0183
