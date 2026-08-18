#define _USE_MATH_DEFINES
#include "sensor.h"
#include <math.h>

/* Ray-vs-axis-aligned-rectangle intersection using the "slab method".
   Returns 1 and sets *out_distance if the ray hits the rectangle, else returns 0. */
static int ray_intersects_rect(float origin_x, float origin_y,
                                float dir_x, float dir_y,
                                const Obstacle *rect,
                                float *out_distance) {
    /* Avoid division by zero: treat near-zero direction components as a tiny value. */
    float inv_dir_x = (dir_x != 0.0f) ? (1.0f / dir_x) : 1e30f;
    float inv_dir_y = (dir_y != 0.0f) ? (1.0f / dir_y) : 1e30f;

    float t1 = (rect->x - origin_x) * inv_dir_x;
    float t2 = (rect->x + rect->width - origin_x) * inv_dir_x;
    float t3 = (rect->y - origin_y) * inv_dir_y;
    float t4 = (rect->y + rect->height - origin_y) * inv_dir_y;

    float tmin = fmaxf(fminf(t1, t2), fminf(t3, t4));
    float tmax = fminf(fmaxf(t1, t2), fmaxf(t3, t4));

    /* If tmax < 0, the rectangle is behind the ray. If tmin > tmax, no intersection. */
    if (tmax < 0.0f || tmin > tmax) {
        return 0;
    }

    /* Use tmin if it's in front of the origin, otherwise the ray starts inside the rect. */
    float t = (tmin >= 0.0f) ? tmin : tmax;
    if (t < 0.0f) {
        return 0;
    }

    *out_distance = t;
    return 1;
}

float sensor_cast_ray(const Vehicle *vehicle, float angle_offset_degrees,
                       const Obstacle *obstacles, int obstacle_count,
                       float max_distance) {
    float angle_radians = (vehicle->angle + angle_offset_degrees) * (float)M_PI / 180.0f;
    float dir_x = cosf(angle_radians);
    float dir_y = sinf(angle_radians);

    float closest_distance = max_distance;

    for (int i = 0; i < obstacle_count; i++) {
        float distance;
        if (ray_intersects_rect(vehicle->x, vehicle->y, dir_x, dir_y, &obstacles[i], &distance)) {
            if (distance < closest_distance) {
                closest_distance = distance;
            }
        }
    }

    return closest_distance;
}