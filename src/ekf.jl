using ForwardDiff: jacobian!
using DiffResults: JacobianResult, value, jacobian

"""
    Extended Kalman filter

The EKF allows you to estimate the states of a nonlinear
dynamical system under white Gaussian noise.

EKF deals with the nonlinearities by linearizing
the system at the current state estimate and then
effectively applying the linear Kalman filter on that.
"""
struct ExtendedKalmanFilter <: KalmanFilter end

"""
    forward_step(::ExtendedKalmanFilter,
                 f::StateEquation{DiscreteTime},
                 prev_state::UncertainValue,
                 input,
                 process_noise::UncertainValue)

Perform a time step of an extended Kalman filter.

This will estimate the state of `model` at the next time
step given the belief of `prev_state` and knowledge of `input`.
"""
function forward_step(::ExtendedKalmanFilter,
    f::StateEquation{DiscreteTime},
    prev_state::UncertainValue,
    input,
    process_noise::UncertainValue)

    x = mean(prev_state)
    u = input

    ad_f = JacobianResult(x)
    jacobian!(ad_f, (x_) -> f(x_, u), x)

    direct_forward_step(
        jacobian(ad_f),
        prev_state,
        value(ad_f) + mean(process_noise),
        process_noise
    )
end

"""
    data_step(::ExtendedKalmanFilter,
              g::MeasurementEquation,
              prior::UncertainValue,
              input,
              observation::UncertainValue)

Perform a data step of an extended Kalman filter.

This allows you to reduce the undertainty of a state estimate
by incorporating new information from a state measurement.
"""
function data_step(::ExtendedKalmanFilter,
    g::MeasurementEquation,
    prior::UncertainValue,
    input,
    observation::UncertainValue)

    x = mean(prior)
    u = input
    y = mean(observation)

    ad_g = JacobianResult(y, x)
    jacobian!(ad_g, (x_) -> g(x_, u), x)

    innovation_data_step(
        jacobian(ad_g),
        prior,
        y - value(ad_g),
        observation
    )
end
