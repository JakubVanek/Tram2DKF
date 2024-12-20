using LinearAlgebra

abstract type UncertainValue end
# mean(state)::Vector{Float64}
# covariance(state)::Matrix{Float64}
# pdf(state, value)::Float64
# logpdf(state, value)::Float64


struct Gaussian <: UncertainValue
    x::Vector{Float64}
    P::Matrix{Float64}
end
mean(g::Gaussian) = g.x
covariance(g::Gaussian) = g.P
Gaussian(g::UncertainValue) = Gaussian(mean(g), covariance(g))
function pdf(g::Gaussian, value::Vector{Float64})
    d = length(g.x)
    offset = value -  g.x
    exponent = -1/2 * (offset' * (g.P \ offset))
    gain = 1/(2*pi)^(d/2) * 1/sqrt(det(g.P))
    gain * exp(exponent)
end
function logpdf(g::Gaussian, value::Vector{Float64})
    d = length(g.x)
    offset = value -  g.x
    exponent = -1/2 * (offset' * (g.P \ offset))
    -d/2*log(2*pi) -1/2*logdet(g.P) + exponent
end


struct SqrtGaussian <: UncertainValue
    x::Vector{Float64}
    L::LowerTriangular{Float64, Matrix{Float64}} # P = LL^T (Cholesky factorization)
end
mean(g::SqrtGaussian) = g.x
covariance(g::SqrtGaussian) = g.L * g.L'
SqrtGaussian(g::UncertainValue) = SqrtGaussian(mean(g), cholesky(covariance(g)).L)
function pdf(g::SqrtGaussian, value::Vector{Float64})
    d = length(g.x)
    offset = value -  g.x
    deskewed_offset = g.L' \ offset
    exponent = -1/2 * sum(deskewed_offset.^2)
    gain = 1/(2*pi)^(d/2) * 1/det(g.L)
    gain * exp(exponent)
end
function logpdf(g::SqrtGaussian, value::Vector{Float64})
    d = length(g.x)
    offset = value -  g.x
    deskewed_offset = g.L' \ offset
    exponent = -1/2 * sum(deskewed_offset.^2)
    -d/2*log(2*pi) -logdet(g.L) + exponent
end


abstract type KalmanFilter end
# forward_step(kf::KalmanFilter, model::StateEquation{DiscreteTime}, prev_state::UncertainValue, input::Vector{Float64}, process_noise::UncertainValue) -> new_state::UncertainValue
# backward_step(kf::KalmanFilter, model::StateEquation{DiscreteTime}, current_posterior::UncertainValue, next_prior::UncertainValue, next_smoothed::UncertainValue) -> current_smoothed::UncertainValue
# data_step(kf::KalmanFilter, model::StateEquation{DiscreteTime}, prior::UncertainValue, input::Vector{Float64}, observation::UncertainValue) -> posterior::UncertainValue
