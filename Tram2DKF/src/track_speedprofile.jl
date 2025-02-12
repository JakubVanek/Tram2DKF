abstract type TrajectorySegment end
# activate(segment::TrajectorySegment, time, pos, speed, accel)::ActiveTrajectorySegment

TrajectoryDrive = @NamedTuple{speed::Float64, accel::Float64}
abstract type ActiveTrajectorySegment end
# drive(segment::ActiveTrajectorySegment, time, pos, speed, accel), returns Union{TrajectoryDrive, Nothing}

# TRAM STOP

@kwdef struct Stop <: TrajectorySegment
    duration::Float64
end
struct StopState <: ActiveTrajectorySegment
    template::Stop
    started_at::Float64
end
activate(stop::Stop, time, pos, speed, accel) = StopState(stop, time)
function drive(stop::StopState, time, pos, speed, accel)::Union{TrajectoryDrive, Nothing}
    time < stop.started_at + stop.template.duration && return (speed = 0.0, accel = 0.0)
    return nothing
end


# TRAM ACCELERATION

@kwdef struct Accelerate <: TrajectorySegment
    to_speed::Float64
    acceleration::Float64
end
struct AccelerateState <: ActiveTrajectorySegment
    from_time::Float64
    to_time::Float64
    from_speed::Float64
    to_speed::Float64
    acceleration::Float64
end
activate(acc::Accelerate, time, pos, speed, accel) = AccelerateState(
    time,
    time + abs((acc.to_speed - speed) / acc.acceleration),
    speed,
    acc.to_speed,
    copysign(acc.acceleration, acc.to_speed - speed)
)
function drive(acc::AccelerateState, time, pos, speed, accel)::Union{TrajectoryDrive, Nothing}
    time < acc.from_time && return (
        speed = acc.from_speed,
        accel = 0.0
    )
    time < acc.to_time && return (
        speed = lerp(acc.from_time, acc.from_speed, acc.to_time, acc.to_speed, time),
        accel = acc.acceleration
    )
    return nothing
end

# TRAM COASTING

@kwdef struct ConstantSpeed <: TrajectorySegment
    speed::Float64
    distance::Float64
end
struct ConstantSpeedState <: ActiveTrajectorySegment
    template::ConstantSpeed
    from_point::Float64
end
activate(spd::ConstantSpeed, time, pos, speed, accel) = ConstantSpeedState(spd, pos)
function drive(spd::ConstantSpeedState, time, pos, speed, accel)::Union{TrajectoryDrive, Nothing}
    pos < spd.from_point + spd.template.distance && return (
        speed = spd.template.speed,
        accel = 0.0
    )
    return nothing
end
