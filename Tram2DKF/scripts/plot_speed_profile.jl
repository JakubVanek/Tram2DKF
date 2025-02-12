using Tram2DKF
using Tram2DKF: IDX_TIME, IDX_SPEED
using Plots

track = [
    StraightTrack(distance=100.0), # test that jumping to next segment works
    StraightTrack(distance=900.0)
]

trip = [
    Stop(duration=1.0),
    Accelerate(to_speed=10.0, acceleration=1.0),
    ConstantSpeed(speed=10.0, distance=100.0),
    Accelerate(to_speed=0.0, acceleration=1.0),
    Stop(duration=10.0)
]

states = render_trip(track, trip, 0.01)

t = states .|> x -> x[IDX_TIME]
v = states .|> x -> x[IDX_SPEED]

plot(t, v)
xlabel!("Time [s]")
ylabel!("Speed [m/s]")
