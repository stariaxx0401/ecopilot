#ifndef SENSOR_H
#define SENSOR_H

#include "vehicle.h"

/* An obstacle is a simple axis-aligned rectangle (wall). */
typedef struct {
    float x;      /* top-left corner */
    float y;
    float width;
    float height;
} Obstacle;

/* Casts a single ray from the vehicle's position, at vehicle->angle + angle_offset_degrees.
   Checks against all obstacles and returns the distance to the nearest hit.
   If nothing is hit within max_distance, returns max_distance. */
float sensor_cast_ray(const Vehicle *vehicle, float angle_offset_degrees,
                       const Obstacle *obstacles, int obstacle_count,
                       float max_distance);

/* Given an array of ray distances (as produced by sensor_cast_ray, spread
   evenly across a field of view), decides whether the vehicle should steer
   away from a nearby obstacle. Returns -1.0 (steer left), 0.0 (no obstacle
   close enough to react to), or +1.0 (steer right) - the caller multiplies
   this by their own turn speed and delta time. */
float sensor_compute_avoidance_direction(const float *ray_distances, int ray_count,
                                          float avoidance_threshold);

#endif /* SENSOR_H */