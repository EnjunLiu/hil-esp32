#pragma once

#include "IQmathLib.h"

struct Observer {
    _iq T;
    _iq v_max;
    _iq e_max;
    _iq Delta_t;
    _iq state;
    _iq state_hat;
    _iq v_state_hat;

    Observer();

    void configure(float time_constant, float v_max, float e_max, float delta_t);
    void resetState();

    void update(const _iq new_state);
};

struct GuidanceLaw {
    _iq T;
    _iq theta;
    _iq A;
    _iq e_track_x;
    _iq e_track_y;
    _iq varepsilon;
    _iq Delta_t;
    _iq desired_velocity;
    _iq desired_angle;

    GuidanceLaw();

    void configure(float time_constant, float delta_t);
    void resetState();

    void update(_iq v_hat_x, _iq v_hat_y, _iq x_hat, _iq y_hat);
};
