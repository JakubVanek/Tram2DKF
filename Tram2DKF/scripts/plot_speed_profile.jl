using Tram2DKF
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

t = states .|> x -> x[1]
s = states .|> x -> x[2]
x = states .|> x -> x[3]
y = states .|> x -> x[4]
v = states .|> x -> x[5]
a = states .|> x -> x[6]
φ = states .|> x -> x[7]
c = states .|> x -> x[8]
dc = states .|> x -> x[9]

plot(t, v)
xlabel!("Time [s]")
ylabel!("Speed [m/s]")
