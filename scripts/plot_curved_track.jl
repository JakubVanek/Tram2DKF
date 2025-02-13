using Tram2DKF
using Tram2DKF: IDX_X_COORD, IDX_Y_COORD
using Plots

# define track profile
track = [
    StraightTrack(distance=100.0),
    TrackTurn(angle=pi/2, radius=20.0, transition_curve_length=5.0),
    StraightTrack(distance=50.0),
    TrackTurn(angle=-pi/2, radius=50.0, transition_curve_length=50.0),
    StraightTrack(distance=100.0),
]

# define speed profile: constant speed -> 1 meter = 1 second -> equidistant trajectory sampling
trip = [
    ConstantSpeed(speed=1.0, distance=10000.0)
]

# calculate the trajectory
states = render_trip(track, trip, 0.1)

# and visualize it

x = states .|> x -> x[IDX_X_COORD]
y = states .|> x -> x[IDX_Y_COORD]

plot(x, y, aspect_ratio=:equal)
xlabel!("X [m]")
ylabel!("Y [m]")
