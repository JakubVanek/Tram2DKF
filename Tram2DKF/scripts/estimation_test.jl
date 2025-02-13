using Tram2DKF
using Tram2DKF: PolarCTRA
using LinearAlgebra
import Tram2DKF.nstates, Tram2DKF.ninputs, Tram2DKF.noutputs


Ts = 0.1

track = [
    StraightTrack(distance=100.0),
    TrackTurn(angle=deg2rad(+90), radius=20.0, transition_curve_length=0.0),
    StraightTrack(distance=100.0),
    TrackTurn(angle=deg2rad(-45), radius=100.0, transition_curve_length=10.0),
    StraightTrack(distance=100.0),
]

trip = [
    Stop(duration=1.0),
    Accelerate(to_speed=10, acceleration=1.0),
    Accelerate(to_speed=5, acceleration=1.0),
    ConstantSpeed(speed=5, distance=50),
    Accelerate(to_speed=0, acceleration=1.0),
    Stop(duration=10.0),
    Accelerate(to_speed=8, acceleration=1.0),
    ConstantSpeed(speed=8, distance=120),
    Accelerate(to_speed=0, acceleration=1.0),
    Stop(duration=10.0),
]

states = render_trip(track, trip, Ts, 10)

gyro_std = 0.02
acc_std = 0.1
odo_res = 0.01
gnss_std = 1.0
gnss_cov = I(3) * gnss_std^2
gyro = simulate_gyro(states, gyro_std)
acc  = simulate_accelerometer(states, acc_std)
odo  = simulate_odometry(states, odo_res)
gnss = simulate_gnss(states, 200, gnss_cov)

odo_v = (odo[2:end] - odo[1:end-1]) / Ts
odo_std = (odo_res/Ts)/sqrt(12)

ax = acc  .|> x -> x[1];
ay = acc  .|> x -> x[2];
wz = gyro .|> x -> x[3];

t  = states .|> x -> x[1]
s  = states .|> x -> x[2];
cx = states .|> x -> x[3];
cy = states .|> x -> x[4];
v  = states .|> x -> x[5];
a  = states .|> x -> x[6];
heading    = states .|> x -> x[7];
curvature  = states .|> x -> x[8];
dcurvature = states .|> x -> x[9];

# x[1] ... X position
# x[2] ... Y position
# x[3] ... forward velocity
# x[4] ... forward acceleration
# x[5] ... heading
# x[6] ... yaw rate

struct GNSS <: MeasurementEquation end
(::GNSS)(x,u) = [x[1], x[2]]
ninputs(::GNSS) = 0
nstates(::GNSS) = 6
noutputs(::GNSS) = 2

struct Odometry <: MeasurementEquation end
(::Odometry)(x,u) = [x[3]]
ninputs(::Odometry) = 0
nstates(::Odometry) = 6
noutputs(::Odometry) = 1

struct YawRate <: MeasurementEquation end
(::YawRate)(x,u) = [x[6]]
ninputs(::YawRate) = 0
nstates(::YawRate) = 6
noutputs(::YawRate) = 1

ODO_IMU = CompositeMeasurement([Odometry(), YawRate()])
GNSS_ODO_IMU = CompositeMeasurement([GNSS(), ODO_IMU])

transition_model_ct = PolarCTRA()
transition_model = discretize(transition_model_ct, RK4(), Ts, 1)

process_noise = Gaussian(
    zeros(Float64, nstates(transition_model)),
    diagm([0.0, 0.0, 0.0, 0.1, 0.0, 0.1])
)

current_hypothesis = Gaussian(
    zeros(Float64, nstates(transition_model)),
    diagm([10, 10, 0.1, 0.1, 2*pi, 0.1])
)

hypotheses = [current_hypothesis]
filter = ExtendedKalmanFilter()

steps = length(states)
for i in 1:steps
    global current_hypothesis

    if mod(i, 10) == 1
        meas_model = GNSS_ODO_IMU
        meas_cov = diagm([gnss_std^2, gnss_std^2, odo_std^2, gyro_std^2])
        meas_val = [gnss[i][1], gnss[i][2], odo[i], wz[i]]
    else
        meas_model = ODO_IMU
        meas_cov = diagm([odo_std^2, gyro_std^2])
        meas_val = [odo[i], wz[i]]
    end

    posterior = data_step(filter, meas_model, current_hypothesis, [], Gaussian(meas_val, meas_cov))
    push!(hypotheses, posterior)

    current_hypothesis = forward_step(filter, transition_model, posterior, [], process_noise)
end

estx = hypotheses .|> h -> mean(h)[1]
esty = hypotheses .|> h -> mean(h)[2]
