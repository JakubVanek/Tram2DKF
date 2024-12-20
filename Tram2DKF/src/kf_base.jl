using LinearAlgebra

abstract type UncertainValue end
# mean(state)::Vector{Float64}
# covariance(state)::Matrix{Float64}


struct Gaussian <: UncertainValue
    x::Vector{Float64}
    P::Matrix{Float64}
end
mean(g::Gaussian) = g.x
covariance(g::Gaussian) = g.P
Gaussian(g::UncertainValue) = Gaussian(mean(g), covariance(g))


struct SqrtGaussian <: UncertainValue
    x::Vector{Float64}
    L::LowerTriangular{Float64, Matrix{Float64}} # P = LL^T (Cholesky factorization)
end
mean(g::SqrtGaussian) = g.x
covariance(g::SqrtGaussian) = g.L * g.L'
SqrtGaussian(g::UncertainValue) = SqrtGaussian(mean(g), cholesky(covariance(g)).L)


abstract type KalmanFilter end
# forward_step(kf::KalmanFilter, model::StateEquation{DiscreteTime}, prev_state::UncertainValue, input::Vector{Float64}, process_noise::UncertainValue) -> new_state::UncertainValue
# backward_step(kf::KalmanFilter, model::StateEquation{DiscreteTime}, current_posterior::UncertainValue, next_prior::UncertainValue, next_smoothed::UncertainValue) -> current_smoothed::UncertainValue
# data_step(kf::KalmanFilter, model::StateEquation{DiscreteTime}, prior::UncertainValue, observation::UncertainValue) -> posterior::UncertainValue
