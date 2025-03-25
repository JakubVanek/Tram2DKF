"""
    LinearKalmanFilter

The LKF allows you to optimally estimate states of linear
dynamical systems under white Gaussian noise.
"""
struct LinearKalmanFilter <: KalmanSmoother end

"""
    forward_step(::LinearKalmanFilter,
                 model::LTIStateEquation{DiscreteTime},
                 prev_state::UncertainValue,
                 input,
                 process_noise::UncertainValue)

Perform a time step of a linear Kalman filter.

This will estimate the state of `model` at the next time
step given the belief of `prev_state` and knowledge of `input`.
"""
function forward_step(::LinearKalmanFilter,
    model::LTIStateEquation{DiscreteTime},
    prev_state::UncertainValue,
    input,
    process_noise::UncertainValue)

    prior_mean = mean(prev_state)

    external_in = !isempty(model.B) ? model.B * input : zeros(eltype(prior_mean), nstates(model))
    x = model.A * prior_mean + external_in + mean(process_noise) # nonstandard, but hopefully OK
    direct_forward_step(model.A, prev_state, x, process_noise)
end

"""
    backward_step(::LinearKalmanFilter,
                  model::LTIStateEquation{DiscreteTime},
                  current_posterior::UncertainValue,
                  next_prior::UncertainValue,
                  next_smoothed::UncertainValue)

Perform a smoothing step of a linear Rauch-Tung-Striebel smoother.

This allows you to propagate information from new measurements back in time
and make the estimates of past states less noisy.
"""
function backward_step(::LinearKalmanFilter,
    model::LTIStateEquation{DiscreteTime},
    current_posterior::UncertainValue,
    next_prior::UncertainValue,
    next_smoothed::UncertainValue)

    F = covariance(current_posterior) * model.A' / covariance(next_prior)

    new_x = mean(current_posterior) + F * (mean(next_smoothed) - mean(next_prior))
    new_Pxx = covariance(current_posterior) - F * (covariance(next_prior) - covariance(next_smoothed)) * F'
    Gaussian(new_x, new_Pxx)
end

"""
    data_step(::LinearKalmanFilter,
              model::LTIMeasurementEquation,
              prior::UncertainValue,
              input,
              observation::UncertainValue)

Perform a data step of a linear Kalman filter.

This allows you to reduce the undertainty of a state estimate
by incorporating new information from a linear state measurement.
"""
function data_step(::LinearKalmanFilter,
    model::LTIMeasurementEquation,
    prior::UncertainValue,
    input,
    observation::UncertainValue)

    observation_mean = mean(observation)

    feedthrough = !isempty(model.D) ? model.D * input : zeros(eltype(observation_mean), noutputs(model))
    innovation = observation_mean - (model.C * mean(prior) + feedthrough)

    innovation_data_step(model.C, prior, innovation, observation)
end


"""
    direct_forward_step(
        model::LTIStateEquation{DiscreteTime},
        prev_state::UncertainValue,
        new_state,
        process_noise::UncertainValue)

Compute the covariance of a state estimate at time k+1 given data up to time k.
Then attach it to a precomputed mean `new_state` and return it as a new belief.
"""
function direct_forward_step(
    A::AbstractMatrix{<:Real},
    prev_state::UncertainValue,
    new_state,
    process_noise::UncertainValue)

    Pxx = A * covariance(prev_state) * A' + covariance(process_noise)
    Gaussian(new_state, Pxx)
end

"""
    direct_forward_step(
        model::LTIStateEquation{DiscreteTime},
        prev_state::SqrtGaussian,
        new_state,
        process_noise::SqrtGaussian)

Compute the covariance of a state estimate at time k+1 given data up to time k.
Then attach it to a precomputed mean `new_state` and return it as a new belief.

Compared to the [`direct_forward_step`](@ref) above, this method uses the QR factorization
to compute the final covariance. This may be more numerically stable.

For details about the math, see
T. M. Chin, „Square-root formulas for Kalman filter, information filter, and RTS smoother: Links via boomerang prediction residual".
"""
function direct_forward_step(
    A::AbstractMatrix{<:Real},
    prev_state::SqrtGaussian,
    new_state,
    process_noise::SqrtGaussian)

    next_L = lq!([process_noise.L  A * prev_state.L]).L
    SqrtGaussian(new_state, LowerTriangular(next_L))
end

"""
    innovation_data_step(
        model::LTIMeasurementEquation,
        prior::UncertainValue,
        innovation,
        observation::UncertainValue)

Compute a corrected state estimate given a new measurement.

This method is intended as a reusable piece of [`data_step`](@ref).
It takes in a `innovation`, which is the difference between
the actual measured value and the value predicted by the state estimate `prior`.
Thanks to this, it can be, in principle, applied to nonlinear models as well.
"""
function innovation_data_step(
    C::AbstractMatrix{<:Real},
    prior::UncertainValue,
    innovation,
    observation::UncertainValue)

    R = covariance(observation)
    Pxx = covariance(prior)
    Pxy = Pxx * C'
    Pyy = C * Pxx * C' + R
    K = Pxy / Pyy
    IKH = I - K * C

    new_x = mean(prior) + K * innovation
    new_Pxx = IKH*Pxx*IKH' + K*R*K'
    # ^ Joseph form from https://en.wikipedia.org/wiki/Kalman_filter ,
    # it should be a bit more numerically robust
    Gaussian(new_x, new_Pxx)
end


"""
    innovation_data_step(
        model::LTIMeasurementEquation,
        prior::SqrtGaussian,
        innovation,
        observation::SqrtGaussian)

Compute a corrected state estimate given a new measurement.

This method is intended as a reusable piece of [`data_step`](@ref).
It takes in a `innovation`, which is the difference between
the actual measured value and the value predicted by the state estimate `prior`.
Thanks to this, it can be, in principle, applied to nonlinear models as well.

Compared to the [`innovation_data_step`](@ref) above, this method uses
the QR factorization to compute the final covariance. This may be more numerically stable.

For details about the math, see
T. M. Chin, „Square-root formulas for Kalman filter, information filter, and RTS smoother: Links via boomerang prediction residual".
"""
function innovation_data_step(
    C::AbstractMatrix{<:Real},
    prior::SqrtGaussian,
    innovation,
    observation::SqrtGaussian)

    n = size(C, 2)
    p = size(C, 1)

    joint_cov_M = [
        observation.L                    C * prior.L;
        zeros(eltype(prior.L), n, p)     prior.L
    ]
    joint_cov_L = lq!(joint_cov_M).L

    posterior_Ly = joint_cov_L[1:p, 1:p]
    posterior_Lx = joint_cov_L[p+1:p+n, p+1:p+n]
    almost_K = joint_cov_L[p+1:p+n, 1:p]

    posterior_x = prior.x + almost_K * (LowerTriangular(posterior_Ly) \ innovation)

    SqrtGaussian(posterior_x, LowerTriangular(posterior_Lx))
end

# TODO: maybe http://kth.diva-portal.org/smash/get/diva2:808731/FULLTEXT01.pdf and low-rank updates
