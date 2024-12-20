struct ExtendedKalmanFilter <: KalmanFilter end

function forward_step(::ExtendedKalmanFilter,
    f::StateEquation{DiscreteTime},
    prev_state::UncertainValue,
    input,
    process_noise::UncertainValue)

    x = mean(prev_state)
    u = input

    direct_forward_step(
        linearize(f, x, u),
        prev_state,
        f(x, u) + mean(process_noise),
        process_noise
    )
end

function data_step(::ExtendedKalmanFilter,
    g::MeasurementEquation,
    prior::UncertainValue,
    input,
    observation::UncertainValue)

    x = mean(prior)
    u = input

    innovation_data_step(
        linearize(g, x, u),
        prior,
        mean(observation) - g(x, u),
        observation
    )
end
