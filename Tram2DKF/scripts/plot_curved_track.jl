using Tram2DKF
using Tram2DKF: IDX_X_COORD, IDX_Y_COORD
using Plots

track = [
    StraightTrack(distance=100.0),
    TrackTurn(angle=pi/2, radius=20.0, transition_curve_length=5.0),
    StraightTrack(distance=50.0),
    TrackTurn(angle=-pi/2, radius=50.0, transition_curve_length=50.0),
    StraightTrack(distance=100.0),
]

trip = [
    ConstantSpeed(speed=1.0, distance=10000.0)
]

states = render_trip(track, trip, 0.1)

x = states .|> x -> x[IDX_X_COORD]
y = states .|> x -> x[IDX_Y_COORD]

plot(x, y, aspect_ratio=:equal)
xlabel!("X [m]")
ylabel!("Y [m]")
