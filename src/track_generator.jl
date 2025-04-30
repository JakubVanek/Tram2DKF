"""
    SegmentChainer{SegmentCollection}

Helper object for chaining track and speed profile segments.
"""
mutable struct SegmentChainer{SegmentCollection}
    "List of segment definitions"
    segments::SegmentCollection

    "Currently active segment definition"
    active_index::Int

    "Segment generator for the current definition (result of [`activate`](@ref))"
    active_segment::Any
end

"""
    SegmentChainer(segments, arg_pack)

Create an object for chaining of segments of differing types.

# Arguments
* The `segments` argument should be an iterable of segment descriptions
  (e.g. `TrackSegment`s or `SpeedProfileSegment`s)
* The `arg_pack` argument should be a tuple of additional state arguments
  to pass to the [`activate`](@ref) function
"""
SegmentChainer(segments, arg_pack) =
    SegmentChainer(
        segments,
        1,
        activate(segments[1], arg_pack...)
    )

"""
    kick!(chainer::SegmentChainer, arg_pack)

Ensure that the active segment generator is still working.
If not, obtain a generator for the next segment.
If there no more segments, return false.
On the other hand, if the generator in the `active_segment` field was successfully refreshed, return true.
"""
function kick!(chainer::SegmentChainer, arg_pack)
    # if we're past the end, stop early
    chainer.active_index > length(chainer.segments) && return false

    # if the sampler works, return success
    is_working(chainer.active_segment, arg_pack) && return true

    # the sampler doesn't work, try to pull a new one
    chainer.active_index += 1
    if chainer.active_index <= length(chainer.segments)
        chainer.active_segment = activate(chainer.segments[chainer.active_index], arg_pack...)
    end
    # recurse to re-do checks
    return kick!(chainer, arg_pack)
end

"""
    is_working(seg::ActiveTrackSegment, arg_pack::NTuple{1, <:AbstractFloat})

Check that the current track segment has not ended.
"""
is_working(seg::ActiveTrackSegment, arg_pack::NTuple{1, <:AbstractFloat}) =
    !isnothing(curvature(seg, arg_pack...))

"""
    is_working(seg::ActiveSpeedProfileSegment, arg_pack::NTuple{4, <:AbstractFloat})

Check that the current speed profile segment has not ended.
"""
is_working(seg::ActiveSpeedProfileSegment, arg_pack::NTuple{4, <:AbstractFloat}) =
    !isnothing(drive(seg, arg_pack...))





"""
    TramMotionModel

State space model used for generating the tram trajectory.

The model basically a combination of three parts:
* X/Y coordinates of the tram
* longitudinal dynamics: distance, speed, acceleration
* track geometry: heading, curvature and its derivative

The state trajectories of this model should contain
at least straight lines, circular arcs and clothoids.

# State vector layout
* `x[IDX_TIME]` ... Time elapsed since start, in seconds
* `x[IDX_DISTANCE]` ... Distance travelled since start
* `x[IDX_X_COORD]` ... X coordinate of the tram, in metres
* `x[IDX_Y_COORD]` ... Y coordinate of the tram, in metres
* `x[IDX_SPEED]` ... Forward speed of the tram, in m/s
* `x[IDX_ACCELERATION]` ... Forward acceleration of the tram, in m/s^2
* `x[IDX_HEADING]` ... Track heading, in radians
* `x[IDX_CURVATURE]` ... Track curvature, in 1/m
* `x[IDX_DCURVATURE]` ... Track curvature derivative, in 1/m^2
"""
struct TramMotionModel <: StateEquation{ContinuousTime} end
ninputs(::TramMotionModel) = 0
nstates(::TramMotionModel) = 10
function (::TramMotionModel)(x, u)
    t, s, x, y, v, a, j, φ, c, dc = x
    return [1, v, v * cos(φ), v * sin(φ), a, j, 0, v*c, v*dc, 0]
end

"Index into TramMotionModel state: Time elapsed since start, in seconds"
const IDX_TIME         = 1 # [s]
"Index into TramMotionModel state: Distance travelled since start"
const IDX_DISTANCE     = 2 # [m]
"Index into TramMotionModel state: X coordinate of the tram, in metres"
const IDX_X_COORD      = 3 # [m]
"Index into TramMotionModel state: Y coordinate of the tram, in metres"
const IDX_Y_COORD      = 4 # [m]
"Index into TramMotionModel state: Forward speed of the tram, in m/s"
const IDX_SPEED        = 5 # [m/s]
"Index into TramMotionModel state: Forward acceleration of the tram, in m/s^2"
const IDX_ACCELERATION = 6 # [m/s^2]
"Index into TramMotionModel state: Forward jerk of the tram, in m/s^3"
const IDX_JERK         = 7 # [m/s^2]
"Index into TramMotionModel state: Track heading, in radians"
const IDX_HEADING      = 8 # [rad]
"Index into TramMotionModel state: Track curvature, in 1/m"
const IDX_CURVATURE    = 9 # [1/m]
"Index into TramMotionModel state: Track curvature derivative, in 1/m^2"
const IDX_DCURVATURE   = 10 # [1/m^2]





"""
    GeneratorState{NumT, SystemT, TimeT}

Helper structure for carrying track generator state between functions.
"""
mutable struct GeneratorState{NumT <: AbstractFloat, SystemT <: StateEquation{DiscreteTime}, TimeT <: AbstractFloat}
    "Discretized [`TramMotionModel`](@ref)"
    tram_model::SystemT

    "Vector of simulated states"
    states::Vector{Vector{NumT}}

    "Current simulation state"
    current_state::Vector{NumT}

    "Counter of minor sampling periods"
    iteration_index::Int

    "Major sampling period"
    dt::TimeT

    "Divide major sampling period into this many minor periods"
    subsamples::Int
end


"""
    render_trip(track_segments::AbstractVector{<:TrackSegment}, speed_segments::AbstractVector{<:SpeedProfileSegment}, dt::Float64, subsamples::Int = 1, state0::Vector{Float64} = zeros(Float64, nstates(ct_model)))

Simulate a state-space trajectory of a tram going along `track_segments`
and following a speed profile composed of `speed_segments`. Assume that
the trajectory starts with state `state0`.

The simulation will return one sample for every `dt` seconds elapsed.
However, the ODE can be invoked more frequently that that in order to
increase accuracy. To do so, provide a `subsamples` value greater than 1.

The state trajectory is returned as a vector of (state) vectors. The components
of the state vector can be accessed using helper constants, e.g. vector[IDX_SPEED].
"""
function render_trip(track_segments::AbstractVector{<:TrackSegment}, speed_segments::AbstractVector{<:SpeedProfileSegment}, dt::Float64, subsamples::Int = 1, state0::Vector{Float64} = zeros(Float64, nstates(ct_model)))
    ct_model = TramMotionModel()

    gen = GeneratorState(
        discretize(ct_model, RK4, dt / subsamples),
        Vector{Vector{Float64}}(),
        copy(state0),
        1,
        dt,
        subsamples
    )

    push!(gen.states, gen.current_state)

    track = SegmentChainer(track_segments, (gen.current_state[IDX_DISTANCE],))
    speed_profile = SegmentChainer(speed_segments, (
        gen.current_state[IDX_TIME],
        gen.current_state[IDX_DISTANCE],
        gen.current_state[IDX_SPEED],
        gen.current_state[IDX_ACCELERATION]
    ))

    while true
        kick!(track, (gen.current_state[IDX_DISTANCE],)) || break
        kick!(speed_profile, (
            gen.current_state[IDX_TIME],
            gen.current_state[IDX_DISTANCE],
            gen.current_state[IDX_SPEED],
            gen.current_state[IDX_ACCELERATION]
        )) || break

        inner_generator_loop!(gen, track.active_segment, speed_profile.active_segment)
    end

    return gen.states
end

"""
    inner_generator_loop!(gen::GeneratorState, track::ActiveTrackSegment, speed_profile::ActiveSpeedProfileSegment)

Hot loop of the trajectory generator.

This function is factored out in order to have type-stable `track` and `speed_profile` arguments.
The Julia compiler can then avoid calling into dynamic dispatch.

The function will return whenever the `track` or `speed_profile` generators
start returning `nothing`. The calling function should then fetch next segments
where needed and then call this function again.
"""
function inner_generator_loop!(gen::GeneratorState, track::ActiveTrackSegment, speed_profile::ActiveSpeedProfileSegment)
    while true
        # write time without accumulated rounding errors
        gen.current_state[IDX_TIME] = (gen.iteration_index - 1) * gen.dt / gen.subsamples

        # attempt to sample generators
        geometry = curvature(track, gen.current_state[IDX_DISTANCE])
        isnothing(geometry) && break

        speed_command = drive(speed_profile,
            gen.current_state[IDX_TIME],
            gen.current_state[IDX_DISTANCE],
            gen.current_state[IDX_SPEED],
            gen.current_state[IDX_ACCELERATION]
        )
        isnothing(speed_command) && break

        # overwrite system states in-place
        gen.current_state[IDX_CURVATURE] = geometry.curvature
        gen.current_state[IDX_DCURVATURE] = geometry.dcurvature
        gen.current_state[IDX_SPEED] = speed_command.speed
        gen.current_state[IDX_ACCELERATION] = speed_command.accel
        gen.current_state[IDX_JERK] = speed_command.jerk

        # simulate system evolution till next keyframe
        gen.current_state = gen.tram_model(gen.current_state, [])

        # save state if needed
        if mod(gen.iteration_index, gen.subsamples) == 0
            push!(gen.states, gen.current_state)
        end
        gen.iteration_index += 1
    end
end
