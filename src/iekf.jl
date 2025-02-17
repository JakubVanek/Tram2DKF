"""
    IteratedExtendedKalmanFilter{StepSizeControlT, StepNormT <: Real}

The IEKF allows you to estimate the states of a nonlinear
dynamical system under white Gaussian noise.

Compared to the classical EKF, the IEKF may work better for
systems with strongly nonlinear measurement functions.
This is because the IEKF actually finds the maximum aposteriori
estimate when combining the state estimate with measurements.

The IEKF was implemented based on the article
https://ieeexplore.ieee.org/document/10705771 . This article
also illustrates that the IEKF is equivalent to a Gauss-Newton
iteration that maximizes some criterion. The GN method is then
extended to include step-size control to speed up convergence.
"""
struct IteratedExtendedKalmanFilter{StepSizeControlT, StepNormT <: Real} <: KalmanFilter
    """
    Method to use for controlling Gauss-Newton step length.

    A callable with the following interface should be provided:
    
        f(V, x0, xstep0) -> step

    where `V` is a ℝⁿ → ℝ function that is to be minimized,
    `x0` is the starting point and `xstep0` is the step direction
    suggested by the base Gauss-Newton algorithm. The function
    then needs to return an updated `step` that leads to a
    "good™" decrease of `V`.

    See [`IdentityStepping`](@ref) and [`BacktrackingLineSearch`](@ref)
    """
    step_control::StepSizeControlT

    "Stop the GN iterations once the step size norm is smaller than this."
    min_step_size_norm::StepNormT

    "Perform at most this many GN iterations."
    max_iekf_iters::Int
end


"""
    forward_step(::IteratedExtendedKalmanFilter,
                 f::StateEquation{DiscreteTime},
                 prev_state::UncertainValue,
                 input,
                 process_noise::UncertainValue)

Perform a time step of an iterated extended Kalman filter.

This will estimate the state of `model` at the next time
step given the belief of `prev_state` and knowledge of `input`.

Given that the IEKF does not iterate the time update,
this is fully equivalent to the EKF forward step.
"""
forward_step(::IteratedExtendedKalmanFilter,
    f::StateEquation{DiscreteTime},
    prev_state::UncertainValue,
    input,
    process_noise::UncertainValue) = forward_step(ExtendedKalmanFilter(), f, prev_state, input, process_noise)

"""
    data_step(iekf::IteratedExtendedKalmanFilter,
              g::MeasurementEquation,
              prior::UncertainValue,
              input,
              observation::UncertainValue)

Perform a data step of an iterated extended Kalman filter.

This allows you to reduce the undertainty of a state estimate
by incorporating new information from a state measurement.

This method actually computes the maximum aposteriori estimate
of the state. Uncertainty propagation is handled by linearizing
the system around this MAP estimate and then propagating
the Gaussians just like the linear KF does.
"""
function data_step(iekf::IteratedExtendedKalmanFilter,
    g::MeasurementEquation,
    prior::UncertainValue,
    input,
    observation::UncertainValue)

    current_estimate = mean(prior)

    for _ in 1:iekf.max_iekf_iters
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

"""
    iekf_gauss_newton_step(
        g::MeasurementEquation,
        prior::UncertainValue,
        current_est,
        input,
        observation::UncertainValue)

Perform one iteration of the internal Gauss-Newton-like method.

This method does not perform step size control. Instead, the
unscaled step is returned.

See https://ieeexplore.ieee.org/document/10705771 for details.
"""
function iekf_gauss_newton_step(
    g::MeasurementEquation,
    prior::UncertainValue,
    current_est,
    input,
    observation::UncertainValue)

    lin_msr = linearize(g, current_est, input)
    innovation = mean(observation) - g(current_est, input) - lin_msr.C * (mean(prior) - current_est)

    # this handles both square root and normal Gaussians
    innovation_data_step(lin_msr.C, prior, innovation, observation)
end

"""
    iekf_finalize(
        g::MeasurementEquation,
        prior::UncertainValue,
        map_estimate,
        input,
        observation::UncertainValue)

Compute the final state estimate covariance around the MAP estimate.
"""
function iekf_finalize(
    g::MeasurementEquation,
    prior::UncertainValue,
    map_estimate,
    input,
    observation::UncertainValue)

    last_gn_step = iekf_gauss_newton_step(g, prior, map_estimate, input, observation)
    Gaussian(map_estimate, covariance(last_gn_step))
end

"""
    iekf_finalize(
        g::MeasurementEquation,
        prior::SqrtGaussian,
        map_estimate,
        input,
        observation::SqrtGaussian)

Compute the final state estimate covariance around the MAP estimate.

This variant uses square-root-filtering algoritms for increased accuracy.
"""
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
