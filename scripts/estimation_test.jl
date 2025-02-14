using Tram2DKF
using Tram2DKF: PolarCTRA, UncertainValue
using LinearAlgebra
import Tram2DKF.nstates, Tram2DKF.ninputs, Tram2DKF.noutputs
using Plots

# Define helper functions for extracting values from vectors-of-vectors

extract_ts(array::Vector{Vector{Float64}}, index::Int) =
    array .|> item -> item[index]

extract_ts(array::Vector{<:UncertainValue}, index::Int) =
    array .|> item -> mean(item)[index]

extract_cov_ts(array::Vector{<:UncertainValue}, i::Int, j::Int) =
    array .|> item -> covariance(item)[i,j]

# Sampling time
Ts = 0.1 # seconds

## PHASE 1: DATA GENERATION

# Define track profile
track = [
    StraightTrack(distance=100.0),
    TrackTurn(angle=deg2rad(+90), radius=20.0, transition_curve_length=0.0),
    StraightTrack(distance=100.0),
    TrackTurn(angle=deg2rad(-45), radius=100.0, transition_curve_length=10.0),
    StraightTrack(distance=100.0),
]

# Define speed profile
trip = [
    Stop(duration=1.0),
    Accelerate(to_speed=10.0, acceleration=1.0),
    Accelerate(to_speed=5.0, acceleration=1.0),
    ConstantSpeed(speed=5.0, distance=50.0),
    Accelerate(to_speed=0.0, acceleration=1.0),
    Stop(duration=10.0),
    Accelerate(to_speed=8.0, acceleration=1.0),
    ConstantSpeed(speed=8.0, distance=120.0),
    Accelerate(to_speed=0.0, acceleration=1.0),
    Stop(duration=10.0),
]

# Generate ground truth data
states = render_trip(track, trip, Ts, 10)
t  = states .|> x -> x[1]
s  = states .|> x -> x[2];
cx = states .|> x -> x[3];
cy = states .|> x -> x[4];
v  = states .|> x -> x[5];
a  = states .|> x -> x[6];

# Simulate data corrupted by noise
gyro_std = 0.02
acc_std = 0.1
odo_res = 0.01
gnss_std = 1.0
gnss_cov = I(3) * gnss_std^2
gyro = simulate_gyro(states, gyro_std)
acc  = simulate_accelerometer(states, acc_std)
odo  = simulate_odometry(states, odo_res)
gnss = simulate_gnss(states, 200, gnss_cov)

odo_v = vcat([0], (odo[2:end] - odo[1:end-1]) / Ts)
odo_std = (odo_res/Ts)/sqrt(12)

wz = gyro .|> x -> x[3];


## PHASE 2: DEFINE MODEL

# state transition model

transition_model_ct = PolarCTRA()
transition_model = discretize(transition_model_ct, RK4, Ts, 1)

process_noise = Gaussian(
    zeros(Float64, nstates(transition_model)),
    diagm([1e-9, 1e-9, 1e-9, 0.1, 1e-9, 0.01])
)

# measurement model

struct GNSS <: MeasurementEquation end
(::GNSS)(x,u) = [x[1], x[2]]
ninputs(::GNSS) = 0
nstates(::GNSS) = 6
noutputs(::GNSS) = 2

struct DeadReckoning <: MeasurementEquation end
(::DeadReckoning)(x,u) = [x[3], x[6]]
ninputs(::DeadReckoning) = 0
nstates(::DeadReckoning) = 6
noutputs(::DeadReckoning) = 2

DeadReckoningUpdate = DeadReckoning()
CombinedUpdate = CompositeMeasurement([GNSS(), DeadReckoningUpdate])


## PHASE 3: RUN THE KALMAN FILTER

current_hypothesis = Gaussian(
    zeros(Float64, nstates(transition_model)),
    diagm([10, 10, 0.01, 0.01, 2*pi, 0.01])
)

hypotheses = Vector{Gaussian}()
filter = ExtendedKalmanFilter()

steps = length(states)
for i in 1:steps
    global current_hypothesis

    if mod(i, 10) == 1
        measurement = Gaussian(
            [gnss[i][1], gnss[i][2], odo_v[i], wz[i]],
            diagm([gnss_std^2, gnss_std^2, odo_std^2, gyro_std^2])
        )
        posterior = data_step(filter, CombinedUpdate, current_hypothesis, [], measurement)
    else
        measurement = Gaussian(
            [odo_v[i], wz[i]],
            diagm([odo_std^2, gyro_std^2])
        )
        posterior = data_step(filter, DeadReckoningUpdate, current_hypothesis, [], measurement)
    end

    push!(hypotheses, posterior)

    current_hypothesis = forward_step(filter, transition_model, posterior, [], process_noise)
end


## PHASE 4: VISUALIZE THE RESULTS

estx = extract_ts(hypotheses, 1)
esty = extract_ts(hypotheses, 2)
estv = extract_ts(hypotheses, 3)
plot(cx, cy; aspect_ratio=:equal, lc=:red, label="Ground truth")
plot!(estx, esty; aspect_ratio=:equal, lc=:blue, label="Filtered value")
xlabel!("X coordinate [m]")
ylabel!("Y coordinate [m]")

# save plot into PNG
png("estimate.png")
