using Tram2DKF
using Tram2DKF: IDX_TIME, IDX_DISTANCE, IDX_SPEED, IDX_ACCELERATION, IDX_JERK
using Plots

# define track profile
track = [
    StraightTrack(distance=100.0), # test that jumping to next segment works
    StraightTrack(distance=900.0)
]

# define speed profile
trip = [
    Stop(duration=1.0),
    SmoothlyAccelerate(to_speed=10.0, acceleration=1.0, jerk=1.0),
    ConstantSpeed(speed=10.0, distance=100.0),
    SmoothlyAccelerate(to_speed=0.0, acceleration=1.0, jerk=1.0),
    Stop(duration=10.0)
]

# calculate the trajectory
states = render_trip(track, trip, 0.01)

# and visualize it

t = states .|> x -> x[IDX_TIME]
s = states .|> x -> x[IDX_DISTANCE]
v = states .|> x -> x[IDX_SPEED]
a = states .|> x -> x[IDX_ACCELERATION]
j = states .|> x -> x[IDX_JERK]

plot(t, v)
xlabel!("Time [s]")
ylabel!("Speed [m/s]")

# save plot into PNG
png("speed_profile.png")
