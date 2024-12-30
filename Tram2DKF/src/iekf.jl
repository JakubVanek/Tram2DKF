# https://ieeexplore.ieee.org/document/10705771


struct IteratedExtendedKalmanFilter{StepperT <: StepSizeControl} <: KalmanFilter
    step_control::StepperT
    min_step_size_norm::Float64
    max_iekf_iters::Int
end

forward_step(::IteratedExtendedKalmanFilter,
    f::StateEquation{DiscreteTime},
    prev_state::UncertainValue,
    input,
    process_noise::UncertainValue) = forward_step(ExtendedKalmanFilter(), f, prev_state, input, process_noise)

function data_step(iekf::IteratedExtendedKalmanFilter,
    g::MeasurementEquation,
    prior::UncertainValue,
    input,
    observation::UncertainValue)

    current_estimate = mean(prior)

    for iter in 1:iekf.max_iekf_iters
        gn_x = mean(iekf_gauss_newton_step(g, prior, current_estimate, input, observation))
        gn_step = gn_x - current_estimate

        map_criterion(x) = -logpdf(observation, g(x, input)) - logpdf(prior, x)
        ls_step = iekf.step_control(map_criterion, current_estimate, gn_step)
        current_estimate .+= ls_step

        if norm(ls_step) < iekf.min_step_size_norm
            break
        end
    end

    iekf_finalize(g, prior, current_estimate, input, observation)
end

function iekf_gauss_newton_step(
    g::MeasurementEquation,
    prior::UncertainValue,
    current_est,
    input,
    observation::UncertainValue)

    lin_msr = linearize(g, current_est, input)
    innovation = mean(observation) - g(current_est, input) - lin_msr.C * (mean(prior) - current_est)
    innovation_data_step(lin_msr, prior, innovation, observation)
end

function iekf_finalize(
    g::MeasurementEquation,
    prior::UncertainValue,
    map_estimate,
    input,
    observation::UncertainValue)

    last_gn_step = iekf_gauss_newton_step(g, prior, map_estimate, input, observation)
    Gaussian(map_estimate, covariance(last_gn_step))
end

function iekf_finalize(
    g::MeasurementEquation,
    prior::SqrtGaussian,
    map_estimate,
    input,
    observation::SqrtGaussian)

    # shall return SqrtGaussian
    last_gn_step = iekf_gauss_newton_step(g, prior, map_estimate, input, observation)
    SqrtGaussian(map_estimate, last_gn_step.L)
end
