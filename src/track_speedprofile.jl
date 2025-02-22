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

    distance(segment::ActiveSpeedProfileSegment)

Get the distance travelled during this segment in metres.

    duration(segment::ActiveSpeedProfileSegment)

Get the duration in seconds that it will take the tram to traverse this segment.

    end_state(segment::ActiveSpeedProfileSegment)::TrajectoryDrive

Return the tram command at the segment end.
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
    start_at::NumT
    end_at::NumT
end

# implementation of the TrackSegment and ActiveTrackSegment interfaces

"""
    activate(stop::Stop, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat}

Return a description of the segment starting from a given tram motion state.
"""
activate(stop::Stop, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat} =
    StopState{NumT}(time, time + stop.duration)

"""
    drive(stop::StopState{NumT}, time, pos, speed, accel) where {NumT}

Return the speed and acceleration at the given point on the
speed profile. The `position` and `time` values are **not**
relative wrt. the segment start, they are relative to the start
of the entire journey.

Return `nothing` when this segment has ended.
"""
function drive(stop::StopState{NumT}, time, pos, speed, accel) where {NumT}
    time < stop.end_at && return end_state(stop)
    return nothing
end

distance(stop::StopState{NumT}) where {NumT} = zero(NumT)
duration(stop::StopState) = stop.end_at - stop.start_at
end_state(::StopState{NumT}) where {NumT} = TrajectoryDrive{NumT}(
    speed = zero(NumT),
    accel = zero(NumT),
    jerk = zero(NumT)
)




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

function distance(acc::AccelerateState{NumT}) where {NumT}
    time = duration(acc)
    return acc.from_speed * time + 0.5 * acc.acceleration * time^2
end
duration(acc::AccelerateState{NumT}) where {NumT} = acc.to_time - acc.from_time
end_state(acc::AccelerateState{NumT}) where {NumT} = TrajectoryDrive{NumT}(
    speed = acc.to_speed,
    accel = zero(NumT),
    jerk = zero(NumT),
)








# SMOOTH TRAM ACCELERATION

"""
    SmoothlyAccelerate{NumT}

The tram should accelerate from its current speed to a new
`to_speed` at a given `acceleration`, while the rate of acceleration
change is capped by a given `jerk`.
"""
@kwdef struct SmoothlyAccelerate{NumT <: Real} <: SpeedProfileSegment
    to_speed::NumT
    acceleration::NumT
    jerk::NumT
end

"""
    SmoothAccelerateState{NumT}

Generate ramped speed profile with limited jerk.
"""
struct SmoothAccelerateState{NumT <: AbstractFloat} <: ActiveSpeedProfileSegment
    initial_speed::NumT
    final_speed::NumT
    max_accel::NumT
    initial_jerk::NumT

    time_start_rampup::NumT
    time_start_steady::NumT
    time_start_rampdown::NumT
    time_finish::NumT

    speed_at_steady_start::NumT
    speed_at_steady_end::NumT
end

# implementation of the TrackSegment and ActiveTrackSegment interfaces

"""
    activate(acc::SmoothlyAccelerate, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat}

Return a description of the segment starting from a given tram motion state.
"""
function activate(acc::SmoothlyAccelerate, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat}
    required_speed_delta = abs(acc.to_speed - speed)
    time_to_max_accel = abs(acc.acceleration / acc.jerk)
    speed_delta_in_jerk = abs(time_to_max_accel * acc.acceleration)

    if speed_delta_in_jerk < required_speed_delta
        # there is a constant-acceleration segment
        remaining_delta = required_speed_delta - speed_delta_in_jerk
        time_for_remaining = remaining_delta / abs(acc.acceleration)
        true_accel = copysign(acc.acceleration, acc.to_speed - speed)

        return SmoothAccelerateState{NumT}(
            speed,
            acc.to_speed,
            true_accel,
            copysign(acc.jerk, true_accel),

            time,
            time + time_to_max_accel,
            time + time_to_max_accel + time_for_remaining,
            time + 2*time_to_max_accel + time_for_remaining,

            speed + true_accel * time_to_max_accel / 2,
            speed + true_accel * (time_to_max_accel / 2 + time_for_remaining)
        )
    else
        # there are only the ramp parts
        max_accel = sqrt(abs(acc.jerk * required_speed_delta))
        slope_time = abs(max_accel / acc.jerk)
        true_accel = copysign(max_accel, acc.to_speed - speed)

        return SmoothAccelerateState{NumT}(
            speed,
            acc.to_speed,
            true_accel,
            copysign(acc.jerk, true_accel),

            time,
            time + slope_time,
            time + slope_time,
            time + 2*slope_time,

            speed + true_accel * slope_time / 2,
            speed + true_accel * slope_time / 2
        )
    end
end

"""
    drive(acc::SmoothAccelerateState{NumT}, time, pos, speed, accel) where {NumT}

Return the speed and acceleration at the given point on the
speed profile. The `position` and `time` values are **not**
relative wrt. the segment start, they are relative to the start
of the entire journey.

Return `nothing` when this segment has ended.
"""
function drive(acc::SmoothAccelerateState{NumT}, time, pos, speed, accel) where {NumT}
    time < acc.time_start_rampup && return TrajectoryDrive{NumT}(
        speed = acc.initial_jerk,
        accel = 0.0,
        jerk  = 0.0
    )
    time < acc.time_start_steady && return TrajectoryDrive{NumT}(
        speed = acc.initial_speed + 0.5*acc.initial_jerk * (time - acc.time_start_rampup)^2,
        accel = lerp(acc.time_start_rampup, 0, acc.time_start_steady, acc.max_accel, time),
        jerk  = acc.initial_jerk
    )
    time < acc.time_start_rampdown && return TrajectoryDrive{NumT}(
        speed = acc.speed_at_steady_start + acc.max_accel * (time - acc.time_start_steady),
        accel = acc.max_accel,
        jerk = 0.0
    )
    time < acc.time_finish && return TrajectoryDrive{NumT}(
        speed = acc.speed_at_steady_end
                + acc.max_accel * (time - acc.time_start_rampdown)
                - 0.5 * acc.initial_jerk * (time - acc.time_start_rampdown)^2,
        accel = lerp(acc.time_start_rampdown, acc.max_accel, acc.time_finish, 0, time),
        jerk = -acc.initial_jerk
    )
    return nothing
end

function distance(acc::SmoothAccelerateState{NumT}) where {NumT}
    t_ramp = acc.time_start_steady - acc.time_start_rampup
    t_steady = acc.time_start_rampdown - acc.time_start_steady

    v_initial = acc.initial_speed
    v_after_rampup = v_initial + acc.initial_jerk * t_ramp^2/2
    v_after_steady = v_after_rampup + acc.max_accel * t_steady

    s_rampup   = v_initial      * t_ramp                                   + acc.initial_jerk * t_ramp^3/6
    s_steady   = v_after_rampup * t_steady + acc.max_accel * t_steady^2/2
    s_rampdown = v_after_steady * t_ramp   + acc.max_accel * t_ramp^2/2    - acc.initial_jerk * t_ramp^3/6

    return s_rampup + s_steady + s_rampdown
end

duration(acc::SmoothAccelerateState) = acc.time_finish - acc.time_start_rampup

end_state(acc::SmoothAccelerateState{NumT}) where {NumT} = TrajectoryDrive{NumT}(
    speed = acc.final_speed,
    accel = zero(NumT),
    jerk = zero(NumT),
)





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
    from_point::NumT
    to_point::NumT
end


# implementation of the TrackSegment and ActiveTrackSegment interfaces

"""
    activate(spd::ConstantSpeed, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat}

Return a description of the segment starting from a given tram motion state.
"""
activate(spd::ConstantSpeed, time::NumT, pos, speed, accel) where {NumT <: AbstractFloat} =
    ConstantSpeedState{NumT}(spd.speed, pos, pos + spd.distance)

"""
    drive(spd::ConstantSpeedState{NumT}, time, pos, speed, accel) where {NumT}

Return the speed and acceleration at the given point on the
speed profile. The `position` and `time` values are **not**
relative wrt. the segment start, they are relative to the start
of the entire journey.

Return `nothing` when this segment has ended.
"""
function drive(spd::ConstantSpeedState{NumT}, time, pos, speed, accel) where {NumT}
    pos < spd.to_point && return end_state(spd)
    return nothing
end

distance(spd::ConstantSpeedState) = spd.to_point - spd.from_point
duration(spd::ConstantSpeedState) = abs(distance(spd) / spd.speed)
end_state(spd::ConstantSpeedState{NumT}) where {NumT} = TrajectoryDrive{NumT}(
    speed = spd.speed,
    accel = zero(NumT),
    jerk = zero(NumT),
)
