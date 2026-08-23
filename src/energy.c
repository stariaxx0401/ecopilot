#include "energy.h"
#include <math.h>

/* Tuning constants for the consumption model. */
#define DRAG_COEFFICIENT 0.002f       /* energy per (pixel/sec)^2 per second */
#define ACCEL_PENALTY_COEFFICIENT 0.05f /* extra energy per unit of |speed change| */

/* Reference consumption rate used to normalize the eco-score (units/sec at a
   "typical" cruising speed, roughly half of max speed). */
#define REFERENCE_CONSUMPTION_RATE 20.0f

/* Time constant (seconds) for the exponential moving average - roughly how
   long recent driving "matters" before it fades from the score. */
#define SMOOTHING_TIME_CONSTANT 2.0f

EnergyTracker energy_tracker_create(void) {
    EnergyTracker tracker;
    tracker.total_energy = 0.0f;
    tracker.previous_speed = 0.0f;
    tracker.smoothed_rate = 0.0f;
    return tracker;
}

void energy_update(EnergyTracker *tracker, float current_speed, float delta_time_seconds) {
    /* Base consumption: aerodynamic-drag-like term, proportional to speed squared. */
    float drag_consumption = DRAG_COEFFICIENT * current_speed * current_speed * delta_time_seconds;

    /* Penalty for hard acceleration or braking (large speed change between frames). */
    float speed_change = fabsf(current_speed - tracker->previous_speed);
    float accel_penalty = ACCEL_PENALTY_COEFFICIENT * speed_change;

    float frame_consumption = drag_consumption + accel_penalty;
    tracker->total_energy += frame_consumption;
    tracker->previous_speed = current_speed;

    /* Update the smoothed (recent) consumption rate using an exponential
       moving average, so the score reflects recent driving rather than
       an all-time average that can never recover. */
    if (delta_time_seconds > 0.0f) {
        float instant_rate = frame_consumption / delta_time_seconds;
        float alpha = 1.0f - expf(-delta_time_seconds / SMOOTHING_TIME_CONSTANT);
        tracker->smoothed_rate += alpha * (instant_rate - tracker->smoothed_rate);
    }
}

int energy_calculate_eco_score(const EnergyTracker *tracker) {
    /* Smooth falloff: score = 100 when recent consumption is zero, drops
       toward 0 as the smoothed rate grows. At rate == reference, score is 50. */
    int score = (int)(100.0f * REFERENCE_CONSUMPTION_RATE /
                       (REFERENCE_CONSUMPTION_RATE + tracker->smoothed_rate));
    if (score > 100) score = 100;
    if (score < 0) score = 0;
    return score;
}