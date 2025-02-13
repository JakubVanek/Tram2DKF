"""
    TrackSegment

Specifies a part of a tram track geometry.

# Interface

    activate(segment::TrackSegment, position)::ActiveTrackSegment

Returns a description of the segment given its starting position in metres.
"""
abstract type TrackSegment end

"""
    TrackCurvature{NumT}

Represents the local geometry of a tram track.
"""
@kwdef struct TrackCurvature{NumT <: AbstractFloat}
    "Local track curvature, in 1/meter units."
    curvature::NumT
    "Local curvature derivative, in 1/meter^2 units."
    dcurvature::NumT
end

"""
    ActiveTrackSegment

Generates curvature information for a specific segment of a track.

# Interface

    curvature(segment::ActiveTrackSegment, position)::Union{TrackCurvature, Nothing}

Returns the local curvature at `position` metres from track origin.
The `position` value is **not** relative wrt. the segment start.

The method needs to return `nothing` when `position` is after the end of the track segment.
However, it should return zero `position` when the `position` is before the start of the segment.
"""
abstract type ActiveTrackSegment end






# STRAIGHT TRACK

"""
    StraightTrack{NumT}

Describes a `distance` metres long segment of a straight track
(i.e. zero curvature).
"""
@kwdef struct StraightTrack{NumT <: Real} <: TrackSegment
    distance::NumT
end

"""
    StraightTrackState{NumT}

Generates zero curvature between `from_point` and `to_point` track positions.
"""
struct StraightTrackState{NumT <: AbstractFloat} <: ActiveTrackSegment
    from_point::NumT
    to_point::NumT
end

# implementation of the TrackSegment and ActiveTrackSegment interfaces

activate(seg::StraightTrack, pos::NumT) where {NumT <: AbstractFloat} =
    StraightTrackState{NumT}(pos, pos + seg.distance)

function curvature(seg::StraightTrackState{NumT}, pos) where {NumT}
    pos < seg.to_point && return TrackCurvature{NumT}(curvature = zero(NumT), dcurvature = zero(NumT))
    return nothing
end







# TRACK TURN

"""
    TrackTurn{NumT <: Real}

Describes a track curve, optionally including clothoid transition segments.
"""
@kwdef struct TrackTurn{NumT <: Real} <: TrackSegment
    """
    Total change of track heading between the beginning and end of the turn, in radians.

    Use positive values for left turns and negative values for right turns.
    """
    angle::NumT

    "Smallest radius of the turn segment, in metres."
    radius::NumT

    "Length of the clothoid segment at each end, in metres. Can be zero to omit the transition segments entirely."
    transition_curve_length::NumT
end

"""
    TrackTurnState

Precomputed trapezoidal curvature profile for a track turn.

* `max_curvature` is the maximum curvature encountered.
* between `transition_in_start` and `arc_start` the curvature will be linearly increased
* between `arc_start` and `transition_out_start` the curvature will be constant (`max_curvature`)
* between `transition_out_start` and `turn_end` the curvature will be linearly decreased
"""
struct TrackTurnState{NumT <: AbstractFloat} <: ActiveTrackSegment
    max_curvature::NumT
    transition_in_start::NumT
    arc_start::NumT
    transition_out_start::NumT
    turn_end::NumT
end

# implementation of the TrackSegment and ActiveTrackSegment interfaces

function activate(seg::TrackTurn, pos::NumT) where {NumT <: AbstractFloat}
    seg.radius > 0                   || error("turn radius must be positive")
    seg.transition_curve_length >= 0 || error("track transition curve length must be nonnegative")

    max_curvature = 1/seg.radius
    transition_angles = seg.transition_curve_length * max_curvature

    if transition_angles <= abs(seg.angle)
        # there will be some circular arc segment
        circular_arc_length = abs(seg.angle - copysign(transition_angles, seg.angle)) / max_curvature

        return TrackTurnState{NumT}(
            copysign(max_curvature, seg.angle),
            pos,
            pos + seg.transition_curve_length,
            pos + seg.transition_curve_length + circular_arc_length,
            pos + 2*seg.transition_curve_length + circular_arc_length
        )
    else
        # no circular arc segment, just clothoids
        transition_curve_length = sqrt(abs(seg.angle) * seg.radius * seg.transition_curve_length)
        max_curvature = seg.angle / transition_curve_length

        return TrackTurnState{NumT}(
            max_curvature,
            pos,
            pos + transition_curve_length,
            pos + transition_curve_length,
            pos + 2*transition_curve_length
        )
    end
end

function curvature(seg::TrackTurnState{NumT}, pos) where {NumT}
    pos < seg.transition_in_start && return TrackCurvature{NumT}(
        curvature = zero(NumT),
        dcurvature = zero(NumT)
    )
    pos < seg.arc_start && return TrackCurvature{NumT}(
        curvature = lerp(seg.transition_in_start, zero(NumT), seg.arc_start, seg.max_curvature, pos),
        dcurvature = dlerp(seg.transition_in_start, zero(NumT), seg.arc_start, seg.max_curvature)
    )
    pos < seg.transition_out_start && return TrackCurvature{NumT}(
        curvature = seg.max_curvature,
        dcurvature = zero(NumT)
    )
    pos < seg.turn_end && return TrackCurvature{NumT}(
        curvature = lerp(seg.transition_out_start, seg.max_curvature, seg.turn_end, zero(NumT), pos),
        dcurvature = dlerp(seg.transition_out_start, seg.max_curvature, seg.turn_end, zero(NumT))
    )
    return nothing
end
