struct UncertainState
    state::Vector{Float64}
    covariance::Matrix{Float64}
end

struct UncertainMeasurement
    measurement::Vector{Float64}
    covariance::Matrix{Float64}
end

abstract type KalmanFilter end
# forward_step(kf, model, prev_state, input, process_noise_covariance) -> new_state
# backward_step(kf, model, current_posterior, next_prior, next_smoothed) -> current_smoothed
# data_step(kf, model, prior, observation) -> posterior
