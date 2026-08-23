#include "energy.h"
#include <math.h>

/* Tuning constants for the consumption model. */
#define DRAG_COEFFICIENT 0.002f       /* energy per (pixel/sec)^2 per second */
#define ACCEL_PENALTY_COEFFICIENT 0.05f /* extra energy per unit of |speed change| */

/* Reference consumption rate used to normalize the eco-score (units/sec at a
   "typical" cruising speed). Tune this once real driving data is observed. */
#define REFERENCE_CONSUMPTION_RATE 5.0f

EnergyTracker energy_tracker_create(void) {
    EnergyTracker tracker;
    tracker.total_energy = 0.0f;
    tracker.previous_speed = 0.0f;
    return tracker;
}

void energy_update(EnergyTracker *tracker, float current_speed, float delta_time_seconds) {
    /* Base consumption: aerodynamic-drag-like term, proportional to speed squared. */
    float drag_consumption = DRAG_COEFFICIENT * current_speed * current_speed * delta_time_seconds;

    /* Penalty for hard acceleration or braking (large speed change between frames). */
    float speed_change = fabsf(current_speed - tracker->previous_speed);
    float accel_penalty = ACCEL_PENALTY_COEFFICIENT * speed_change;

    tracker->total_energy += drag_consumption + accel_penalty;
    tracker->previous_speed = current_speed;
}

int energy_calculate_eco_score(const EnergyTracker *tracker, float elapsed_time_seconds) {
    if (elapsed_time_seconds <= 0.0f) {
        return 100;
    }

    float average_rate = tracker->total_energy / elapsed_time_seconds;
    float ratio = average_rate / REFERENCE_CONSUMPTION_RATE;

    /* Lower ratio (less consumption than reference) -> higher score. */
    int score = (int)(100.0f - (ratio * 50.0f));
    if (score > 100) score = 100;
    if (score < 0) score = 0;
    return score;
}