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

#endif /* SENSOR_H */