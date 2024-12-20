struct LinearKalmanFilter <: KalmanFilter end

function forward_step(::LinearKalmanFilter,
    model::LTIStateEquation{DiscreteTime},
    prev_state::UncertainValue,
    input::Vector{Float64},
    process_noise::UncertainValue)

    x = model.A * mean(prev_state) + model.B * input + mean(process_noise) # nonstandard, but hopefully OK
    Pxx = model.A * covariance(prev_state) * model.A' + covariance(process_noise)
    Gaussian(x, Pxx)
end

function backward_step(::LinearKalmanFilter,
    model::LTIStateEquation{DiscreteTime},
    current_posterior::UncertainValue,
    next_prior::UncertainValue,
    next_smoothed::UncertainValue)

    F = covariance(current_posterior) * model.A' / covariance(next_smoothed)

    new_x = mean(current_posterior) + F * (mean(next_smoothed) - mean(next_prior))
    new_Pxx = covariance(current_posterior) - F * (covariance(next_prior) - covariance(next_smoothed)) * F'
    Gaussian(new_x, new_Pxx)
end

function data_step(::LinearKalmanFilter,
    model::LTIMeasurementEquation,
    prior::UncertainValue,
    input::Vector{Float64},
    observation::UncertainValue)
    
    R = covariance(observation)
    Pxx = covariance(prior)
    Pxy = Pxx * model.C'
    Pyx = Pxy'
    Pyy = model.C * Pxx * model.C' + R

    innovation = mean(observation) - (model.C * mean(prior) + model.D * input)
    new_x = mean(prior) + Pxy * (Pyy \ innovation)
    new_Pxx = Pxx - Pxy * (Pyy \ Pyx)
    Gaussian(new_x, new_Pxx)
end


# [1] T. M. Chin, „Square-root formulas for Kalman filter, information filter, and RTS smoother: Links via boomerang prediction residual".

function forward_step(::LinearKalmanFilter,
    model::LTIStateEquation{DiscreteTime},
    prev_state::SqrtGaussian,
    input::Vector{Float64},
    process_noise::SqrtGaussian)

    next_x = model.A * prev_state.x + model.B * input + process_noise.x
    next_L = lq([process_noise.L  model.A * prev_state.L]).L

    SqrtGaussian(next_x, LowerTriangular(next_L))
end

function data_step(::LinearKalmanFilter,
    model::LTIMeasurementEquation,
    prior::SqrtGaussian,
    input::Vector{Float64},
    observation::SqrtGaussian)

    n = nstates(model)
    p = noutputs(model)

    joint_cov_M = [
        observation.L   model.C * prior.L;
        zeros(n, p)     prior.L
    ]
    joint_cov_L = lq(joint_cov_M).L

    posterior_Ly = joint_cov_L[1:p, 1:p]
    posterior_Lx = joint_cov_L[p+1:p+n, p+1:p+n]
    almost_K = joint_cov_L[p+1:p+n, 1:p]

    innovation = observation.x - (model.C * prior.x + model.D * input)
    posterior_x = prior.x + almost_K * (LowerTriangular(posterior_Ly) \ innovation)

    SqrtGaussian(posterior_x, LowerTriangular(posterior_Lx))
end

# TODO: maybe http://kth.diva-portal.org/smash/get/diva2:808731/FULLTEXT01.pdf and low-rank updates
