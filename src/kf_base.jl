using LinearAlgebra

"""
    UncertainValue

Represents a randomly distributed variable
with a well-defined mean and covariance.

# Interface

    mean(variable::UncertainValue)::Vector{<:Real}

Return the expected value of the random variable.

    covariance(variable::UncertainValue)::Matrix{<:Real}

Return the covariance (second central moment) of the random variable.

    pdf(variable::UncertainValue, value::Vector{<:Real})::Real

Evaluate the probability density function at the given point.

    logpdf(variable::UncertainValue, value::Vector{<:Real})::Real

Evaluate the logarithm of the probability density function at the given point.
This function can be more numerically stable than pdf() when small probabilities
are involved (e.g. tails of a Gaussian).
"""
abstract type UncertainValue end


"""
    Gaussian{MeanT,CovT} <: UncertainValue

Represents a multidimensional normally distributed random variable.
"""
struct Gaussian{MeanT,CovT} <: UncertainValue where {MeanT <: AbstractVector{<:Real}, CovT <: AbstractMatrix{<:Real}}
    x::MeanT
    P::CovT
end

"""
    Gaussian(g::UncertainValue)

Create a moment-matched Gaussian random variable for the given random variable `g`.
"""
Gaussian(g::UncertainValue) = Gaussian(mean(g), covariance(g))

"""
    mean(g::Gaussian)

Return the mean of the Gaussian.
"""
mean(g::Gaussian) = g.x

"""
    covariance(g::Gaussian)

Return the covariance of the Gaussian.
"""
covariance(g::Gaussian) = g.P

"""
    pdf(g::Gaussian, value)

Evaluate the probability density function at the given point.
"""
function pdf(g::Gaussian, value)
    d = length(g.x)
    offset = value -  g.x
    exponent = -1/2 * (offset' * (g.P \ offset))
    gain = 1/(2*pi)^(d/2) * 1/sqrt(det(g.P))
    gain * exp(exponent)
end

"""
    logpdf(g::Gaussian, value)

Evaluate the logarithm of the probability density function at the given point.
"""
function logpdf(g::Gaussian, value)
    d = length(g.x)
    offset = value -  g.x
    exponent = -1/2 * (offset' * (g.P \ offset))
    -d/2*log(2*pi) -1/2*logdet(g.P) + exponent
end


"""
    SqrtGaussian

Represents a multidimensional normally distributed random variable.

Compared to [`Gaussian`](@ref), this struct represents
the covariance in a LLᵀ factorized form. Some Kalman filter
implementations can exploit this to be more numerically stable.
"""
struct SqrtGaussian{MeanT,CovT} <: UncertainValue where {MeanT <: AbstractVector{<:Real}, CovT <: LowerTriangular{<:Real}}
    x::MeanT
    L::CovT # P = LL^T (Cholesky factorization)
end

"""
    Gaussian(g::UncertainValue)

Create a moment-matched Gaussian random variable for the given random variable `g`.
"""
SqrtGaussian(g::UncertainValue) = SqrtGaussian(mean(g), cholesky(covariance(g)).L)

"""
    mean(g::SqrtGaussian)

Return the mean of the Gaussian.
"""
mean(g::SqrtGaussian) = g.x

"""
    covariance(g::SqrtGaussian)

Return the covariance of the Gaussian.
"""
covariance(g::SqrtGaussian) = g.L * g.L'

"""
    pdf(g::SqrtGaussian, value)

Evaluate the probability density function at the given point.
"""
function pdf(g::SqrtGaussian, value)
    d = length(g.x)
    offset = value -  g.x
    deskewed_offset = g.L' \ offset # better than inversion of full LLᵀ
    exponent = -1/2 * sum(deskewed_offset.^2)
    gain = 1/(2*pi)^(d/2) * 1/det(g.L)
    gain * exp(exponent)
end

"""
    logpdf(g::SqrtGaussian, value)

Evaluate the logarithm of the probability density function at the given point.
"""
function logpdf(g::SqrtGaussian, value)
    d = length(g.x)
    offset = value -  g.x
    deskewed_offset = g.L' \ offset # better than inversion of full LLᵀ
    exponent = -1/2 * sum(deskewed_offset.^2)
    -d/2*log(2*pi) -logdet(g.L) + exponent
end


"""
    KalmanFilter

Algorithm for fusing measurements from multiple sensors
and for estimating a state of a system whose state is measured.

# Interface

    forward_step(kf::KalmanFilter,
                 model::StateEquation{DiscreteTime},
                 prev_state::UncertainValue,
                 input,
                 process_noise::UncertainValue)::UncertainValue

Estimate the state of `model` at the next time step.

The system is assumed to be in state `prev_state` with the specified
`input` applied over the entire sampling period. Additionally, there
is an unknown random disturbance described by `process_noise`.

Return the Gaussian belief about the state at the next time step.

This represents the classic Kalman "time step" / "forward step".

    data_step(kf::KalmanFilter,
              model::StateEquation{DiscreteTime},
              prior::UncertainValue,
              input,
              observation::UncertainValue)::UncertainValue

Correct the current `model` state estimate `prior` using the `observation`.

The system is assumed to be in state `prior` with the specified
`input` applied over the current sampling period.

Return the updated Gaussian belief about the state at the current time step.

This represents the classic Kalman "data step".
"""
abstract type KalmanFilter end


"""
    KalmanSmoother

Extension of [`KalmanFilter`](@ref) that supports smoothing
(propagation of information back in time).

# Interface

    backward_step(kf::KalmanFilter,
                  model::StateEquation{DiscreteTime},
                  current_posterior::UncertainValue,
                  next_prior::UncertainValue,
                  next_smoothed::UncertainValue)::UncertainValue

Smooth the state of `model` at the current time step.

This represents the classic Rauch-Tung-Striebel smoothing algorithm
that allows you to propagate information back in time. Given:
* the estimate of state at time `k` given data up to time `k` (`current_posterior`),
* the estimate of state at time `k+1` given data up to time `k` (`next_prior`),
* the estimate of state at time `k+1` given data up to time `l > k` (`next_smoothed`),
the algorithm gives you an estimate at time `k` based on data up to time `l > k`.
"""
abstract type KalmanSmoother <: KalmanFilter end
