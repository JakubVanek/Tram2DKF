struct LinearKalmanFilter <: KalmanFilter end

function forward_step(::LinearKalmanFilter,
    model::LTIStateEquation{DiscreteTime},
    prev_state::UncertainState,
    input::Vector{Float64},
    process_noise_covariance::Matrix{Float64})

    x = model.A * prev_state.state + model.B * input
    Pxx = model.A * prev_state.covariance * model.A' + process_noise_covariance
    UncertainState(x, Pxx)
end

function backward_step(::LinearKalmanFilter,
    model::LTIStateEquation{DiscreteTime},
    current_posterior::UncertainState,
    next_prior::UncertainState,
    next_smoothed::UncertainState)

    F = current_posterior.covariance * model.A' / next_smoothed.covariance

    new_x = current_posterior.state + F * (next_smoothed.state - next_prior.state)
    new_Pxx = current_posterior.covariance - F * (next_prior.covariance - next_smoothed.covariance) * F'
    UncertainState(new_x, new_Pxx)
end

function data_step(::LinearKalmanFilter,
    model::LTIMeasurementEquation{DiscreteTime},
    prior::UncertainState,
    observation::UncertainMeasurement)
    
    R = observation.covariance
    Pxx = prior.covariance
    Pxy = Pxx * model.C'
    Pyx = Pxy'
    Pyy = model.C * Pxx * model.C' + R

    pred_y = model.C * prior.state
    y = observation.measurement
    new_x = prior.state + Pxy * (Pyy \ (y - pred_y))
    new_Pxx = Pxx - Pxy * (Pyy \ Pyx)
    UncertainState(new_x, new_Pxx)
end
