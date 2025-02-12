using Tram2DKF
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

t = states .|> x -> x[1]
s = states .|> x -> x[2]
x = states .|> x -> x[3]
y = states .|> x -> x[4]
v = states .|> x -> x[5]
a = states .|> x -> x[6]
φ = states .|> x -> x[7]
c = states .|> x -> x[8]
dc = states .|> x -> x[9]

plot(x, y, aspect_ratio=:equal)
xlabel!("X [m]")
ylabel!("Y [m]")
