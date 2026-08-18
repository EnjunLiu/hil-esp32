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
