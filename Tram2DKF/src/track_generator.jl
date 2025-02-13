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
IDX_TIME         = 1 # [s]
IDX_DISTANCE     = 2 # [m]
IDX_X_COORD      = 3 # [m]
IDX_Y_COORD      = 4 # [m]
IDX_SPEED        = 5 # [m/s]
IDX_ACCELERATION = 6 # [m/s^2]
IDX_HEADING      = 7 # [rad]
IDX_CURVATURE    = 8 # [1/m]
IDX_DCURVATURE   = 9 # [1/m^2]

struct TramMotionModel <: StateEquation{ContinuousTime} end
ninputs(::TramMotionModel) = 0
nstates(::TramMotionModel) = 9
function (::TramMotionModel)(x, u)
    t, s, x, y, v, a, φ, c, dc = x
    return [1, v, v * cos(φ), v * sin(φ), a, 0, v*c, v*dc, 0]
end


function render_trip(track_segments::Vector{<:TrackSegment}, tram_segments::Vector{<:TrajectorySegment}, dt::Float64, subsamples::Int = 1)
    generating_system = discretize(TramMotionModel(), RK4, dt / subsamples)

    states = Vector{Vector{Float64}}()
    state = zeros(Float64, nstates(generating_system))
    push!(states, state)
    idx = 1

    track = TrackChainer(track_segments, state[IDX_DISTANCE])
    speed_profile = TrajectoryChainer(tram_segments,
        state[IDX_TIME],
        state[IDX_DISTANCE],
        state[IDX_SPEED],
        state[IDX_ACCELERATION]
    )

    while true
        # write time without accumulated rounding errors
        state[IDX_TIME] = (idx - 1) * dt / subsamples

        geometry = sample(track, state[IDX_DISTANCE])
        drive    = sample(speed_profile,
            state[IDX_TIME],
            state[IDX_DISTANCE],
            state[IDX_SPEED],
            state[IDX_ACCELERATION]
        )

        isnothing(geometry) && break
        isnothing(drive)    && break

        # overwrite system states in-place
        state[IDX_CURVATURE] = geometry.curvature
        state[IDX_DCURVATURE] = geometry.dcurvature
        state[IDX_SPEED] = drive.speed
        state[IDX_ACCELERATION] = drive.accel

        # simulate system evolution till next keyframe
        state = generating_system(state, [])

        # save state if needed
        if mod(idx, subsamples) == 0
            push!(states, state)
        end
        idx += 1
    end

    return states
end
