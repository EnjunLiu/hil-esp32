#pragma once

#include <stdint.h>
#include <stdbool.h>

struct ControlInputPlain {
    uint32_t seq = 0;
    int64_t stamp_us = 0;

    float desired_x = 0.0f;
    float desired_y = 0.0f;

    float surge_velocity = 0.0f;
    float yaw_rate = 0.0f;

    bool valid = false;
};

struct ControllerParamsPlain {
    uint32_t version = 0;

    float time_constant = 10.0f;
    float v_max = 0.5f;
    float e_max = 0.2f;
    float delta_t = 0.1f;

    float gamma_rl = 0.99f;
    float lambda_rls = 0.99f;

    float max_force = 0.5f;
    float max_moment = 0.1f;

    float i_bound_force = 0.1f;
    float d_bound_force = 0.1f;
    float i_bound_moment = 0.0075f;
    float d_bound_moment = 0.005f;
};

struct WrenchPlain {
    uint32_t seq = 0;
    int64_t stamp_us = 0;
    float force = 0.0f;
    float moment = 0.0f;
    bool valid = false;
};

struct DebugPlain {
    uint32_t seq = 0;
    int64_t stamp_us = 0;

    float x_hat = 0.0f;
    float y_hat = 0.0f;
    float v_hat_x = 0.0f;
    float v_hat_y = 0.0f;

    float v_hat = 0.0f;
    float theta = 0.0f;
    float e_track_x = 0.0f;
    float e_track_y = 0.0f;
    float a = 0.0f;
    float varepsilon = 0.0f;

    float delta_v = 0.0f;
    float delta_psi = 0.0f;

    float p_f = 0.0f;
    float i_f = 0.0f;
    float d_f = 0.0f;

    float p_m = 0.0f;
    float i_m = 0.0f;
    float d_m = 0.0f;
};
