#define _USE_MATH_DEFINES
#include "vehicle.h"
#include <math.h>

Vehicle vehicle_create(float x, float y) {
    Vehicle v;
    v.x = x;
    v.y = y;
    v.angle = 0.0f;
    v.speed = 0.0f;
    return v;
}

void vehicle_update(Vehicle *vehicle, float delta_time_seconds) {
    /* Convert angle to radians and move the vehicle forward along its heading. */
    float radians = vehicle->angle * (float)M_PI / 180.0f;
    vehicle->x += cosf(radians) * vehicle->speed * delta_time_seconds;
    vehicle->y += sinf(radians) * vehicle->speed * delta_time_seconds;
}