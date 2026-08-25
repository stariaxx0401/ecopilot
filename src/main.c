#define _USE_MATH_DEFINES
#include <SDL2/SDL.h>
#include <stdio.h>
#include <math.h>
#include "vehicle.h"
#include "sensor.h"
#include "energy.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

#define OBSTACLE_COUNT 3
#define WALL_COUNT 4
#define TOTAL_OBSTACLE_COUNT (OBSTACLE_COUNT + WALL_COUNT)
#define WALL_THICKNESS 4.0f
#define MAX_SENSOR_RANGE 400.0f

/* Lidar configuration: cast this many rays spread across a field of view. */
#define LIDAR_RAY_COUNT 8
#define LIDAR_FOV_DEGREES 120.0f  /* total field of view, centered on vehicle's heading */

/* If any ray detects an obstacle closer than this, start avoiding. */
#define AVOIDANCE_THRESHOLD 100.0f
#define AVOIDANCE_TURN_SPEED 120.0f /* degrees/sec, applied on top of manual turning */

/* Vehicle rendering size (rectangle dimensions in pixels). */
#define VEHICLE_LENGTH 40.0f
#define VEHICLE_WIDTH 20.0f

/* How fast the vehicle accelerates/turns in response to input. */
#define ACCELERATION 150.0f   /* pixels/sec^2 */
#define MAX_SPEED 200.0f      /* pixels/sec */
#define TURN_SPEED 180.0f     /* degrees/sec */

/* Draws the vehicle as a rotated rectangle using SDL_RenderGeometry
   (four points forming two triangles). */
static void draw_vehicle(SDL_Renderer *renderer, const Vehicle *v) {
    float radians = v->angle * (float)M_PI / 180.0f;
    float cos_a = cosf(radians);
    float sin_a = sinf(radians);

    float half_len = VEHICLE_LENGTH / 2.0f;
    float half_wid = VEHICLE_WIDTH / 2.0f;

    /* Local-space corners of the rectangle (before rotation). */
    SDL_FPoint local_corners[4] = {
        { -half_len, -half_wid },
        {  half_len, -half_wid },
        {  half_len,  half_wid },
        { -half_len,  half_wid }
    };

    SDL_Vertex vertices[4];
    for (int i = 0; i < 4; i++) {
        float lx = local_corners[i].x;
        float ly = local_corners[i].y;
        float rotated_x = lx * cos_a - ly * sin_a;
        float rotated_y = lx * sin_a + ly * cos_a;

        vertices[i].position.x = v->x + rotated_x;
        vertices[i].position.y = v->y + rotated_y;
        vertices[i].color.r = 80;
        vertices[i].color.g = 200;
        vertices[i].color.b = 120;
        vertices[i].color.a = 255;
        vertices[i].tex_coord.x = 0;
        vertices[i].tex_coord.y = 0;
    }

    int indices[6] = { 0, 1, 2, 0, 2, 3 };
    SDL_RenderGeometry(renderer, NULL, vertices, 4, indices, 6);
}

/* Draws an obstacle as a filled rectangle. */
static void draw_obstacle(SDL_Renderer *renderer, const Obstacle *obstacle) {
    SDL_SetRenderDrawColor(renderer, 200, 80, 80, 255);
    SDL_FRect rect = { obstacle->x, obstacle->y, obstacle->width, obstacle->height };
    SDL_RenderFillRectF(renderer, &rect);
}

/* Draws a single ray as a line from the vehicle out to the hit point (or max range). */
static void draw_ray(SDL_Renderer *renderer, const Vehicle *v, float angle_offset_degrees, float distance) {
    float angle_radians = (v->angle + angle_offset_degrees) * (float)M_PI / 180.0f;
    float end_x = v->x + cosf(angle_radians) * distance;
    float end_y = v->y + sinf(angle_radians) * distance;

    SDL_SetRenderDrawColor(renderer, 255, 220, 80, 255);
    SDL_RenderDrawLineF(renderer, v->x, v->y, end_x, end_y);
}

/* HUD layout constants. */
#define HUD_BAR_X 20
#define HUD_BAR_WIDTH 200
#define HUD_BAR_HEIGHT 16
#define HUD_BAR_SPACING 28

/* Draws a single labeled bar: a dark background rect with a colored fill
   proportional to (value / max_value), clamped to [0, 1]. */
static void draw_hud_bar(SDL_Renderer *renderer, int y, float value, float max_value,
                          Uint8 r, Uint8 g, Uint8 b) {
    float fraction = value / max_value;
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    /* Background (empty part of the bar). */
    SDL_SetRenderDrawColor(renderer, 60, 60, 65, 255);
    SDL_FRect background = { (float)HUD_BAR_X, (float)y, (float)HUD_BAR_WIDTH, (float)HUD_BAR_HEIGHT };
    SDL_RenderFillRectF(renderer, &background);

    /* Filled portion. */
    SDL_SetRenderDrawColor(renderer, r, g, b, 255);
    SDL_FRect fill = { (float)HUD_BAR_X, (float)y, (float)HUD_BAR_WIDTH * fraction, (float)HUD_BAR_HEIGHT };
    SDL_RenderFillRectF(renderer, &fill);
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow(
        "ecopilot",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN
    );

    if (!window) {
        fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    Vehicle vehicle = vehicle_create(WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT / 2.0f);
    EnergyTracker energy_tracker = energy_tracker_create();
    float total_elapsed_time = 0.0f;
    float last_score_print_time = 0.0f;

    /* A few fixed obstacles scattered around the map, plus four boundary
       walls so the vehicle can't drive off-screen. */
    Obstacle obstacles[TOTAL_OBSTACLE_COUNT] = {
        { 550.0f, 150.0f, 80.0f, 200.0f },
        { 150.0f, 400.0f, 200.0f, 60.0f },
        { 350.0f, 80.0f, 120.0f, 40.0f },
        /* Top wall */
        { 0.0f, 0.0f, (float)WINDOW_WIDTH, WALL_THICKNESS },
        /* Bottom wall */
        { 0.0f, (float)WINDOW_HEIGHT - WALL_THICKNESS, (float)WINDOW_WIDTH, WALL_THICKNESS },
        /* Left wall */
        { 0.0f, 0.0f, WALL_THICKNESS, (float)WINDOW_HEIGHT },
        /* Right wall */
        { (float)WINDOW_WIDTH - WALL_THICKNESS, 0.0f, WALL_THICKNESS, (float)WINDOW_HEIGHT }
    };

    int running = 1;
    SDL_Event event;
    Uint64 last_time = SDL_GetPerformanceCounter();

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = 0;
            }
        }

        /* Compute delta time so movement speed is independent of frame rate. */
        Uint64 current_time = SDL_GetPerformanceCounter();
        float delta_time = (float)(current_time - last_time) / (float)SDL_GetPerformanceFrequency();
        last_time = current_time;

        /* Read keyboard state for continuous movement (held keys). */
        const Uint8 *keys = SDL_GetKeyboardState(NULL);

        if (keys[SDL_SCANCODE_UP]) {
            vehicle.speed += ACCELERATION * delta_time;
            if (vehicle.speed > MAX_SPEED) vehicle.speed = MAX_SPEED;
        } else if (keys[SDL_SCANCODE_DOWN]) {
            vehicle.speed -= ACCELERATION * delta_time;
            if (vehicle.speed < -MAX_SPEED / 2.0f) vehicle.speed = -MAX_SPEED / 2.0f;
        } else {
            /* Gradually slow down when no key is pressed (friction). */
            if (vehicle.speed > 0) {
                vehicle.speed -= ACCELERATION * delta_time;
                if (vehicle.speed < 0) vehicle.speed = 0;
            } else if (vehicle.speed < 0) {
                vehicle.speed += ACCELERATION * delta_time;
                if (vehicle.speed > 0) vehicle.speed = 0;
            }
        }

        if (keys[SDL_SCANCODE_LEFT]) {
            vehicle.angle -= TURN_SPEED * delta_time;
        }
        if (keys[SDL_SCANCODE_RIGHT]) {
            vehicle.angle += TURN_SPEED * delta_time;
        }

        vehicle_update(&vehicle, delta_time);

        /* Hard safety net: clamp the vehicle inside the screen bounds even if
           the avoidance logic didn't react quickly enough. Keeps the vehicle's
           rectangle fully inside the walls. */
        float half_diagonal = sqrtf((VEHICLE_LENGTH / 2.0f) * (VEHICLE_LENGTH / 2.0f) +
                                     (VEHICLE_WIDTH / 2.0f) * (VEHICLE_WIDTH / 2.0f));
        float min_x = WALL_THICKNESS + half_diagonal;
        float max_x = (float)WINDOW_WIDTH - WALL_THICKNESS - half_diagonal;
        float min_y = WALL_THICKNESS + half_diagonal;
        float max_y = (float)WINDOW_HEIGHT - WALL_THICKNESS - half_diagonal;

        if (vehicle.x < min_x) vehicle.x = min_x;
        if (vehicle.x > max_x) vehicle.x = max_x;
        if (vehicle.y < min_y) vehicle.y = min_y;
        if (vehicle.y > max_y) vehicle.y = max_y;

        /* Hard collision prevention against the scattered obstacles (not the
           walls, already handled above): treat the vehicle as a circle and
           push it back out if it overlaps a rectangle. */
        for (int i = 0; i < OBSTACLE_COUNT; i++) {
            const Obstacle *obs = &obstacles[i];
            /* Closest point on the rectangle to the vehicle's center. */
            float closest_x = vehicle.x;
            if (closest_x < obs->x) closest_x = obs->x;
            if (closest_x > obs->x + obs->width) closest_x = obs->x + obs->width;
            float closest_y = vehicle.y;
            if (closest_y < obs->y) closest_y = obs->y;
            if (closest_y > obs->y + obs->height) closest_y = obs->y + obs->height;

            float dx = vehicle.x - closest_x;
            float dy = vehicle.y - closest_y;
            float dist_sq = dx * dx + dy * dy;

            if (dist_sq < half_diagonal * half_diagonal) {
                float dist = sqrtf(dist_sq);
                if (dist > 0.0001f) {
                    /* Push the vehicle out along the vector from the closest
                       point to the vehicle's center, with a small extra margin
                       so it doesn't immediately re-trigger next frame. */
                    float push = (half_diagonal - dist) + 0.5f;
                    vehicle.x += (dx / dist) * push;
                    vehicle.y += (dy / dist) * push;
                } else {
                    /* Vehicle center is exactly on the rectangle edge (rare) -
                       push straight up as a fallback. */
                    vehicle.y -= half_diagonal;
                }
                /* Note: we don't kill speed here - the distance-based slowdown
                   above already handles that. Doing both was causing the
                   vehicle to get stuck (speed reset every frame while close). */
            }
        }

        /* Track energy consumption and print the eco-score to the console
           roughly once per second (HUD display comes on Day 7). */
        total_elapsed_time += delta_time;
        energy_update(&energy_tracker, vehicle.speed, delta_time);
        if (total_elapsed_time - last_score_print_time >= 1.0f) {
            int score = energy_calculate_eco_score(&energy_tracker);
            printf("Eco-score: %d (total energy: %.1f)\n", score, energy_tracker.total_energy);
            last_score_print_time = total_elapsed_time;
        }

        /* Clear screen (dark background). */
        SDL_SetRenderDrawColor(renderer, 25, 25, 30, 255);
        SDL_RenderClear(renderer);

        for (int i = 0; i < TOTAL_OBSTACLE_COUNT; i++) {
            draw_obstacle(renderer, &obstacles[i]);
        }

        /* Cast multiple rays spread across the field of view (basic lidar simulation). */
        float ray_distances[LIDAR_RAY_COUNT];
        for (int i = 0; i < LIDAR_RAY_COUNT; i++) {
            /* Spread rays evenly from -FOV/2 to +FOV/2 around the vehicle's heading. */
            float angle_offset = -LIDAR_FOV_DEGREES / 2.0f +
                (LIDAR_FOV_DEGREES * i) / (float)(LIDAR_RAY_COUNT - 1);
            float distance = sensor_cast_ray(&vehicle, angle_offset, obstacles, TOTAL_OBSTACLE_COUNT, MAX_SENSOR_RANGE);
            ray_distances[i] = distance;
            draw_ray(renderer, &vehicle, angle_offset, distance);
        }

        /* Ask the sensor module whether we should steer away from something close. */
        float avoidance_direction = sensor_compute_avoidance_direction(
            ray_distances, LIDAR_RAY_COUNT, AVOIDANCE_THRESHOLD);
        vehicle.angle += avoidance_direction * AVOIDANCE_TURN_SPEED * delta_time;

        /* Also cap the speed when something is close - turning alone isn't
           enough to avoid a collision if the vehicle is moving fast. Using a
           speed CAP (not a per-frame multiply) avoids repeatedly crushing the
           speed every frame, which felt sluggish when boxed in near a wall. */
        float closest_distance = ray_distances[0];
        for (int i = 1; i < LIDAR_RAY_COUNT; i++) {
            if (ray_distances[i] < closest_distance) closest_distance = ray_distances[i];
        }
        if (closest_distance < AVOIDANCE_THRESHOLD) {
            float speed_cap_factor = closest_distance / AVOIDANCE_THRESHOLD;
            if (speed_cap_factor < 0.7f) speed_cap_factor = 0.7f; /* always allow reasonable escape speed */
            float speed_cap = MAX_SPEED * speed_cap_factor;
            if (vehicle.speed > speed_cap) vehicle.speed = speed_cap;
            if (vehicle.speed < -speed_cap) vehicle.speed = -speed_cap;
        }

        draw_vehicle(renderer, &vehicle);

        /* HUD: speed bar (green), consumption rate bar (orange), eco-score bar (blue). */
        int current_score = energy_calculate_eco_score(&energy_tracker);
        draw_hud_bar(renderer, 20, fabsf(vehicle.speed), MAX_SPEED, 90, 200, 120);
        draw_hud_bar(renderer, 20 + HUD_BAR_SPACING, energy_tracker.smoothed_rate, 80.0f, 230, 150, 60);
        draw_hud_bar(renderer, 20 + HUD_BAR_SPACING * 2, (float)current_score, 100.0f, 90, 150, 230);

        SDL_RenderPresent(renderer);
        SDL_Delay(16); /* cap roughly at 60 FPS */
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}