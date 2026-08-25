#include "estimator/estimator.hpp"
#include "iq_math_extension/iq_scalar.hpp"

#define PI 3.14159265

Observer::Observer()
    : T(_IQ(0)), v_max(_IQ(0)), e_max(_IQ(0)), Delta_t(_IQ(0)),
      state(_IQ(0)), state_hat(_IQ(0)), v_state_hat(_IQ(0)) {}

// Configure the observer with the time constant, maximum velocity, maximum error, and delta time
void Observer::configure(float time_constant, float v_max, float e_max, float delta_t)
{
    T = _IQ(time_constant);
    this->v_max = _IQ(v_max);
    this->e_max = _IQ(e_max);
    Delta_t = _IQ(delta_t);
}

void Observer::resetState()
{
    state = state_hat = v_state_hat = _IQ(0);
}

void Observer::update(const _iq new_state) {
    state = new_state;
    _iq e_state = state - state_hat;

    v_state_hat = _IQmpy(_IQmpy(_IQdiv(_IQ(1.0), T),
        (_IQsqrt(_IQmpy(_IQabs(e_state), _IQmpy(e_state, e_state))) + _IQsqrt(_IQabs(e_state)))),
        _IQsign(e_state))
        + _IQmpy(v_max, _IQdiv(_IQsat(e_state, e_max, -e_max), e_max));

    state_hat = state_hat + _IQmpy(Delta_t, v_state_hat);
}

GuidanceLaw::GuidanceLaw()
    : T(_IQ(0)), theta(_IQ(0)), A(_IQ(0)),
      e_track_x(_IQ(0)), e_track_y(_IQ(0)), varepsilon(_IQ(0)), Delta_t(_IQ(0)),
      desired_velocity(_IQ(0)), desired_angle(_IQ(0)) {}

void GuidanceLaw::configure(float time_constant, float delta_t)
{
    T = _IQ(time_constant);
    Delta_t = _IQ(delta_t);
}

void GuidanceLaw::resetState()
{
    theta = A = e_track_x = e_track_y = varepsilon = _IQ(0);
    desired_velocity = desired_angle = _IQ(0);
}

void GuidanceLaw::update(_iq v_hat_x, _iq v_hat_y, _iq x_hat, _iq y_hat) {
    _iq v_hat = _IQhypot(v_hat_x, v_hat_y);
    theta = _IQatan2(v_hat_y, v_hat_x);
    e_track_x =   _IQmpy(x_hat, _IQcos(theta)) + _IQmpy(y_hat, _IQsin(theta));
    e_track_y = - _IQmpy(x_hat, _IQsin(theta)) + _IQmpy(y_hat, _IQcos(theta));
    A = _IQxi(e_track_x) + _IQmpy(T, v_hat);
    varepsilon = _IQ(1.0) + _IQsign(_IQabs(theta) - _IQ(PI / 2));

    desired_velocity = _IQmpy(_IQdiv(_IQpowi(_IQ(-1.0), _IQdiv(varepsilon, 2)), T),
        _IQsqrt(_IQmpy(A, A) + _IQmpy(_IQxi(e_track_y), _IQxi(e_track_y))));
        
    _iq Delta_psi = theta + _IQatan2(_IQxi(e_track_y), A) + _IQmpy(varepsilon, _IQ(PI / 2));
    while (Delta_psi > _IQ(PI / 2)) Delta_psi -= _IQ(PI);
    while (Delta_psi < _IQ(-PI / 2)) Delta_psi += _IQ(PI);
    desired_angle = Delta_psi;
}
