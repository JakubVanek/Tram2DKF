"""
    SpeedProfileSegment

Specifies a part of a tram speed profile.

# Interface

    activate(segment::SpeedProfileSegment, time, position, speed, acceleration)::ActiveSpeedProfileSegment

Returns a description of the segment starting from a given tram motion state.
"""
abstract type SpeedProfileSegment end

"""
    TrajectoryDrive{NumT}

Represents the local shape of the speed profile.
"""
@kwdef struct TrajectoryDrive{NumT <: AbstractFloat}
    "Current tram speed, in m/s. It should linearly change if `acceleration` is nonzero."
    speed::NumT
    "Current tram acceleration, in m/s^2."
    accel::NumT
    "Current tram jerk, in m/s^3."
    jerk::NumT
end

"""
    ActiveSpeedProfileSegment

Generates speed profile information for its specific segment.

# Interface

    drive(segment::ActiveSpeedProfileSegment, time, position, speed, acceleration)::Union{TrajectoryDrive, Nothing}

Returns the speed and acceleration at the given point on the
speed profile. The `position` and `time` values are **not**
relative wrt. the segment start, they are relative to the start
of the entire journey.

The method needs to return `nothing` when this segment has ended.
"""
abstract type ActiveSpeedProfileSegment end





# TRAM STOP

"""
    Stop{NumT}

The tram should stand still for `duration` seconds.
"""
@kwdef struct Stop{NumT <: Real} <: SpeedProfileSegment
    duration::NumT
end

"""
    StopState{NumT}

Generate zero speed until the `end_at` time mark.
"""
struct StopState{NumT <: AbstractFloat} <: ActiveSpeedProfileSegment
    end_at::NumT
end

# implementation of the TrackSegment and ActiveTrackSegment interfaces

"""
    activate(stop::Stop, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat}

Return a description of the segment starting from a given tram motion state.
"""
activate(stop::Stop, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat} =
    StopState{NumT}(time + stop.duration)

"""
    drive(stop::StopState{NumT}, time, pos, speed, accel) where {NumT}

Return the speed and acceleration at the given point on the
speed profile. The `position` and `time` values are **not**
relative wrt. the segment start, they are relative to the start
of the entire journey.

Return `nothing` when this segment has ended.
"""
function drive(stop::StopState{NumT}, time, pos, speed, accel) where {NumT}
    time < stop.end_at && return TrajectoryDrive{NumT}(
        speed = zero(NumT),
        accel = zero(NumT),
        jerk = zero(NumT)
    )
    return nothing
end





# TRAM ACCELERATION

"""
    Accelerate{NumT}

The tram should accelerate from its current speed to a new
`to_speed` at a given `acceleration`.
"""
@kwdef struct Accelerate{NumT <: Real} <: SpeedProfileSegment
    to_speed::NumT
    acceleration::NumT
end

"""
    AccelerateState{NumT}

Generate linearly ramped speed profile:
* at `from_time` the speed is `from_speed`
* at `to_time` the speed is `to_speed`
* `acceleration` is provided for convenience; it has the correct sign.
"""
struct AccelerateState{NumT <: AbstractFloat} <: ActiveSpeedProfileSegment
    from_time::NumT
    to_time::NumT
    from_speed::NumT
    to_speed::NumT
    acceleration::NumT
end

# implementation of the TrackSegment and ActiveTrackSegment interfaces

"""
    activate(acc::Accelerate, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat}

Return a description of the segment starting from a given tram motion state.
"""
activate(acc::Accelerate, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat} =
    AccelerateState{NumT}(
        time,
        time + abs((acc.to_speed - speed) / acc.acceleration),
        speed,
        acc.to_speed,
        copysign(acc.acceleration, acc.to_speed - speed)
    )

"""
    drive(acc::AccelerateState{NumT}, time, pos, speed, accel) where {NumT}

Return the speed and acceleration at the given point on the
speed profile. The `position` and `time` values are **not**
relative wrt. the segment start, they are relative to the start
of the entire journey.

Return `nothing` when this segment has ended.
"""
function drive(acc::AccelerateState{NumT}, time, pos, speed, accel) where {NumT}
    time < acc.from_time && return TrajectoryDrive{NumT}(
        speed = acc.from_speed,
        accel = zero(NumT),
        jerk = zero(NumT),
    )
    time < acc.to_time && return TrajectoryDrive{NumT}(
        speed = lerp(acc.from_time, acc.from_speed, acc.to_time, acc.to_speed, time),
        accel = acc.acceleration,
        jerk = zero(NumT),
    )
    return nothing
end





# TRAM COASTING

"""
    ConstantSpeed{NumT}

The tram should move at a constant `speed` until it travels a given `distance`.
"""
@kwdef struct ConstantSpeed{NumT <: Real} <: SpeedProfileSegment
    speed::NumT
    distance::NumT
end

"""
    ConstantSpeedState{NumT}

Generate a constant speed profile until the `to_point` time mark.
"""
struct ConstantSpeedState{NumT <: AbstractFloat} <: ActiveSpeedProfileSegment
    speed::NumT
    to_point::NumT
end


# implementation of the TrackSegment and ActiveTrackSegment interfaces

"""
    activate(spd::ConstantSpeed, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat}

Return a description of the segment starting from a given tram motion state.
"""
activate(spd::ConstantSpeed, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat} =
    ConstantSpeedState{NumT}(spd.speed, pos + spd.distance)

"""
    drive(spd::ConstantSpeedState{NumT}, time, pos, speed, accel) where {NumT}

Return the speed and acceleration at the given point on the
speed profile. The `position` and `time` values are **not**
relative wrt. the segment start, they are relative to the start
of the entire journey.

Return `nothing` when this segment has ended.
"""
function drive(spd::ConstantSpeedState{NumT}, time, pos, speed, accel) where {NumT}
    pos < spd.to_point && return TrajectoryDrive{NumT}(
        speed = spd.speed,
        accel = zero(NumT),
        jerk = zero(NumT)
    )
    return nothing
end
