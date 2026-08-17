#ifndef VEHICLE_H
#define VEHICLE_H

/* Represents the simulated car: position, heading, and current speed. */
typedef struct {
    float x;          /* position on screen (pixels) */
    float y;
    float angle;       /* heading, in degrees (0 = facing right) */
    float speed;        /* current speed (pixels per second) */
} Vehicle;

/* Creates a vehicle at the given starting position. */
Vehicle vehicle_create(float x, float y);

/* Updates the vehicle's position based on its speed/angle and elapsed time. */
void vehicle_update(Vehicle *vehicle, float delta_time_seconds);

#endif /* VEHICLE_H */