#pragma once


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
  float altitude;
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
  float up;
};
