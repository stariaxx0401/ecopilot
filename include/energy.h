#ifndef ENERGY_H
#define ENERGY_H

/* Tracks cumulative energy consumption and the vehicle's previous speed
   (needed to compute acceleration between frames). */
typedef struct {
    float total_energy;     /* accumulated consumption, arbitrary units (for display) */
    float previous_speed;    /* speed from the last frame, for acceleration calc */
    float smoothed_rate;      /* recent consumption rate (units/sec), exponential moving average */
} EnergyTracker;

/* Creates a fresh energy tracker with zero accumulated consumption. */
EnergyTracker energy_tracker_create(void);

/* Updates the tracker for one frame: adds this frame's consumption to the total.
   Consumption = base drag (proportional to speed^2) + a penalty for hard
   acceleration/braking (proportional to |change in speed|). */
void energy_update(EnergyTracker *tracker, float current_speed, float delta_time_seconds);

/* Converts the tracker's recent (smoothed) consumption rate into a 0-100
   eco-score (100 = most efficient). Reflects recent driving, not the
   lifetime average, so the score can recover after a period of gentle driving. */
int energy_calculate_eco_score(const EnergyTracker *tracker);

#endif /* ENERGY_H */