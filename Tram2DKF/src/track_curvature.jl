abstract type TrackSegment end
# activate(segment::TrackSegment, pos)::ActiveTrackSegment

TrackCurvature = @NamedTuple{curvature::Float64, dcurvature::Float64}
abstract type ActiveTrackSegment end
# curvature(segment::ActiveTrackSegment, pos)::Union{TrackCurvature, Nothing}

@kwdef struct StraightTrack <: TrackSegment
    distance::Float64
end
struct StraightTrackState <: ActiveTrackSegment
    template::StraightTrack
    from_point::Float64
end
activate(seg::StraightTrack, pos) = StraightTrackState(seg, pos)
function curvature(seg::StraightTrackState, pos)::Union{TrackCurvature, Nothing}
    (pos - seg.from_point) < seg.template.distance && return (curvature = 0.0, dcurvature = 0.0)
    return nothing
end


@kwdef struct TrackTurn <: TrackSegment
    angle::Float64
    radius::Float64
    transition_curve_length::Float64 = 0.0
end
struct TrackTurnState <: ActiveTrackSegment
    max_curvature::Float64
    transition_in_start::Float64
    arc_start::Float64
    transition_out_start::Float64
    turn_end::Float64
end
function activate(seg::TrackTurn, pos)::TrackTurnState
    seg.radius > 0                   || error("turn radius must be positive")
    seg.transition_curve_length >= 0 || error("track transition curve length must be nonnegative")

    max_curvature = 1/seg.radius
    transition_angles = seg.transition_curve_length * max_curvature

    if transition_angles <= abs(seg.angle)
        circular_arc_length = abs(seg.angle - copysign(transition_angles, seg.angle)) / max_curvature

        TrackTurnState(
            copysign(max_curvature, seg.angle),
            pos,
            pos + seg.transition_curve_length,
            pos + seg.transition_curve_length + circular_arc_length,
            pos + 2*seg.transition_curve_length + circular_arc_length
        )
    else
        transition_curve_length = sqrt(abs(seg.angle) * seg.radius * seg.transition_curve_length)
        max_curvature = seg.angle / transition_curve_length

        TrackTurnState(
            max_curvature,
            pos,
            pos + transition_curve_length,
            pos + transition_curve_length,
            pos + 2*transition_curve_length
        )
    end
end
function curvature(seg::TrackTurnState, pos)::Union{TrackCurvature, Nothing}
    pos < seg.transition_in_start && return (
        curvature = 0.0,
        dcurvature = 0.0
    )
    pos < seg.arc_start && return (
        curvature = lerp(seg.transition_in_start, 0.0, seg.arc_start, seg.max_curvature, pos),
        dcurvature = dlerp(seg.transition_in_start, 0.0, seg.arc_start, seg.max_curvature)
    )
    pos < seg.transition_out_start && return (
        curvature = seg.max_curvature,
        dcurvature = 0.0
    )
    pos < seg.turn_end && return (
        curvature = lerp(seg.transition_out_start, seg.max_curvature, seg.turn_end, 0.0, pos),
        dcurvature = dlerp(seg.transition_out_start, seg.max_curvature, seg.turn_end, 0.0)
    )
    return nothing
end
