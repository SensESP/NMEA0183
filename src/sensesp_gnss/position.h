#pragma once

#include <limits>

namespace sensesp {

/// Value used to indicate an invalid or missing altitude
constexpr float kPositionInvalidAltitude = std::numeric_limits<float>::lowest();

/**
 * @brief Position data container.
 * 
 * Position data as latitudes and longitudes, in decimal degrees, and altitude,
 * in meters.
 * 
 */
struct Position {
  double latitude;
  double longitude;
  float altitude = kPositionInvalidAltitude;
};

/**
 * @brief Container for local tangent plane coordinates.
 * 
 * East-North-Up coordinates, in reference to the local tangent plane.
 * Static locations are in meters, velocities in m/s.
 * 
 */
struct ENUVector {
  float east;
  float north;
  float up = kPositionInvalidAltitude;
};

}
