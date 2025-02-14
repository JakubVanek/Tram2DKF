using LinearAlgebra

"Earth gravitational acceleration, in m/s^2."
const EARTH_GRAVITY = 9.81

"""
    simulate_gyro(states, wn_std)

Simulate 3-axis gyro measurements for the trajectory defined by the `states` sequence.

The gyro is assumed to be aligned with the vehicle body (X forward, Y left, Z up).
Additionally, the measurements are corrupted by a white Gaussian noise with standard
deviation equal to `wn_std`.
"""
simulate_gyro(states, wn_std) =
    states .|> state -> wn_std * randn(eltype(state), 3) + [0, 0, state[IDX_SPEED] * state[IDX_CURVATURE]]

"""
    simulate_accelerometer(states, wn_std)

Simulate 3-axis accelerometer measurements for the trajectory defined by the `states` sequence.

The accelerometer is assumed to be aligned with the vehicle body (X forward, Y left, Z up).
Additionally, the measurements are corrupted by a white Gaussian noise with standard
deviation equal to `wn_std`.
"""
simulate_accelerometer(states, wn_std) =
    states .|> state -> wn_std * randn(eltype(state), 3) + [
        state[IDX_ACCELERATION],
        state[IDX_SPEED]^2 * state[IDX_CURVATURE],
        EARTH_GRAVITY
    ]

"""
    simulate_odometry(states, granularity)

Simulate odometry measurements for the trajectory defined by the `states` sequence.

The odometer is assumed to be counting teeth on a wheel encoder.
Thus, a position quantization is simulated.
However, no slip is simulated.

The returned distances are still in metres.
"""
simulate_odometry(states, granularity) =
    states .|> state -> granularity * round(state[IDX_DISTANCE] / granularity)

"""
    simulate_gnss(states, altitude, cov_matrix)

Simulate GNSS measurements for the trajectory defined by the `states` sequence.

The GNSS will return directly the X and Y state components in metres.
Additionally, it will return the specified `altitude`.
Finally, all the components are corrupted by a white Gaussian noise with
`cov_matrix` covariance matrix.
"""
function simulate_gnss(states, altitude, cov_matrix)
    cov_sqrt = cholesky(cov_matrix).U
    return states .|> state -> cov_sqrt * randn(eltype(state), 3) + [
        state[IDX_X_COORD],
        state[IDX_Y_COORD],
        altitude
    ]
end
