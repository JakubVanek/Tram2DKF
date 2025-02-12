using LinearAlgebra

EARTH_GRAVITY = 9.81

simulate_gyro(states::Vector{Vector{Float64}}, wn_std) =
    states .|> state -> wn_std * randn(Float64, 3) + [0, 0, state[IDX_SPEED] * state[IDX_CURVATURE]]

simulate_accelerometer(states::Vector{Vector{Float64}}, wn_std) =
    states .|> state -> wn_std * randn(Float64, 3) + [
        state[IDX_ACCELERATION],
        state[IDX_SPEED]^2 * state[IDX_CURVATURE],
        EARTH_GRAVITY
    ]

simulate_odometry(states::Vector{Vector{Float64}}, granularity) =
    states .|> state -> granularity * round(state[IDX_DISTANCE] / granularity)

function simulate_gnss(states::Vector{Vector{Float64}}, altitude, cov_matrix)
    cov_sqrt = cholesky(cov_matrix).U
    return states .|> state -> cov_sqrt * randn(Float64, 3) + [
        state[IDX_X_COORD],
        state[IDX_Y_COORD],
        altitude
    ]
end
