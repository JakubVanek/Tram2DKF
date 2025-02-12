mutable struct TrackChainer
    segments::Vector{<:TrackSegment}
    active_index::Int
    active_segment::Any
end
function TrackChainer(segments::Vector{<:TrackSegment}, s0::Float64)
    TrackChainer(
        segments, 1,
        activate(segments[1], s0)
    )
end
function sample(chainer::TrackChainer, s::Float64)::Union{Nothing,TrackCurvature}
    if chainer.active_index > length(chainer.segments)
        return nothing
    end

    result = curvature(chainer.active_segment, s)
    if isnothing(result)
        chainer.active_index += 1
        if chainer.active_index <= length(chainer.segments)
            chainer.active_segment = activate(chainer.segments[chainer.active_index], s)
        end
        # recurse to re-do checks
        return sample(chainer, s)
    end
    return result
end

mutable struct TrajectoryChainer
    segments::Vector{<:TrajectorySegment}
    active_index::Int
    active_segment::Any
end
function TrajectoryChainer(segments::Vector{<:TrajectorySegment}, t0::Float64, s0::Float64, v0::Float64, a0::Float64)
    TrajectoryChainer(
        segments, 1,
        activate(segments[1], t0, s0, v0, a0)
    )
end
function sample(chainer::TrajectoryChainer, t::Float64, s::Float64, v::Float64, a::Float64)::Union{Nothing,TrajectoryDrive}
    if chainer.active_index > length(chainer.segments)
        return nothing
    end

    result = drive(chainer.active_segment, t, s, v, a)
    if isnothing(result)
        chainer.active_index += 1
        if chainer.active_index <= length(chainer.segments)
            chainer.active_segment = activate(chainer.segments[chainer.active_index], t, s, v, a)
        end
        # recurse to re-do checks
        return sample(chainer, t, s, v, a)
    end
    return result
end

# state vector layout:
# x[1] ... travelled time
# x[2] ... travelled distance
# x[3] ... X position
# x[4] ... Y position
# x[5] ... forward speed
# x[6] ... forward acceleration
# x[7] ... track heading
# x[8] ... track curvature
# x[9] ... track curvature derivative
struct TramMotionModel <: StateEquation{ContinuousTime} end
ninputs(::TramMotionModel) = 0
nstates(::TramMotionModel) = 9
function (::TramMotionModel)(x, u)
    t, s, x, y, v, a, φ, c, dc = x
    return [1, v, v * cos(φ), v * sin(φ), a, 0, v*c, v*dc, 0]
end


function render_trip(track_segments::Vector{<:TrackSegment}, tram_segments::Vector{<:TrajectorySegment}, dt::Float64)
    generating_system = discretize(TramMotionModel(), RK4(), dt)

    states = Vector{Vector{Float64}}()
    push!(states, zeros(nstates(generating_system)))

    t = 0.0 # time [s]
    s = 0.0 # track distance [m]
    x = 0.0 # X coordinate [m]
    y = 0.0 # Y coordinate [m]
    v = 0.0 # forward speed [m/s]
    a = 0.0 # forward acceleration [m/s^2]
    φ = 0.0 # heading [rad]
    c = 0.0 # curvature [1/m]
    dc = 0.0 # curvature derivative [1/m^2]

    track = TrackChainer(track_segments, s)
    speed_profile = TrajectoryChainer(tram_segments, t, s, v, a)

    while true
        geometry = sample(track, s)
        drive    = sample(speed_profile, t, s, v, a)

        isnothing(geometry) && break
        isnothing(drive)    && break

        # overwrite system states in-place
        c = geometry.curvature
        dc = geometry.dcurvature
        v = drive.speed
        a = drive.accel

        # simulate system evolution till next keyframe
        x₀ = [t, s, x, y, v, a, φ, c, dc]
        x₁ = generating_system(x₀, [])
        push!(states, x₁)
        t, s, x, y, v, a, φ, c, dc = x₁
    end

    return states
end
