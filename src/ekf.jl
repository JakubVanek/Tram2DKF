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

    direct_forward_step(
        linearize(f, x, u).A,
        prev_state,
        f(x, u) + mean(process_noise),
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

    innovation_data_step(
        linearize(g, x, u).C,
        prior,
        mean(observation) - g(x, u),
        observation
    )
end
