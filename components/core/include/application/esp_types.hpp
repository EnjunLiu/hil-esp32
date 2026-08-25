#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * 输入结构体
 * 包含时间戳、期望位置、期望速度、期望航向角、是否有效
 */
struct Input {
    int64_t stamp_us = 0;
    float desired_x = 0.0f;
    float desired_y = 0.0f;
    float surge_velocity = 0.0f;
    float yaw_rate = 0.0f;
    bool valid = false;
};

/*
 * 参数结构体
 * 包含参数版本号、时间常数、最大速度、最大误差、积分时间、RL参数、衰减系数、积分上限、微分上限、输入超时时间
 */
struct Params {
    uint32_t version = 0; // 参数版本号

    float time_constant = 10.0f; // 时间常数
    float v_max = 0.5f;
    float e_max = 0.2f;
    float delta_t = 0.1f;

    float gamma_rl = 0.99f;
    float lambda_rls = 0.99f;

    float decay_force = 0.999f;
    float decay_moment = 0.99f;
    float w_bound = 5.0f; 
    float pid_gain_max = 100.0f;
    float pid_gain_min = 0.01f;

    float max_force = 0.5f; 
    float max_moment = 0.1f;

    float i_bound_force = 0.1f;
    float d_bound_force = 0.1f;
    float i_bound_moment = 0.0075f;
    float d_bound_moment = 0.005f;

    float input_timeout_s = 0.5f;
};

/*
 * 输出结构体
 * 包含时间戳、力、力矩、是否有效
 */
struct Output {
    int64_t stamp_us = 0;
    float force = 0.0f;
    float moment = 0.0f;
    bool valid = false;
};
