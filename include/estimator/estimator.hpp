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

    Observer(float _T, float _v_max, float _e_max, float _Delta_t)
        : T(_IQ(_T)), v_max(_IQ(_v_max)), e_max(_IQ(_e_max)), Delta_t(_IQ(_Delta_t)),
          state(_IQ(0)), state_hat(_IQ(0)), v_state_hat(_IQ(0)) {}

    /* 观测器状态更新 */
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

    GuidanceLaw(float _T, float _Delta_t)
        : T(_IQ(_T)), theta(_IQ(0)), A(_IQ(0)),
          e_track_x(_IQ(0)), e_track_y(_IQ(0)),
          varepsilon(_IQ(0)), Delta_t(_IQ(_Delta_t)),
          desired_velocity(_IQ(0)), desired_angle(_IQ(0)) {}

    /* 导引律状态更新 */
    void update(_iq v_hat_x, _iq v_hat_y, _iq x_hat, _iq y_hat);
};
