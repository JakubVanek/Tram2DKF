# Most of these come from
# - R. Schubert, E. Richter, a G. Wanielik, „Comparison and Evaluation of Advanced Motion Models for Vehicle Tracking".
# - X. Rong Li a V. P. Jilkov, „Survey of Maneuvering Target Tracking—Part 1: Dynamic models", IEEE Trans. Aerosp. Electron. Syst., roč. 39, č. 4, s. 1333–1364, říj. 2003, doi: 10.1109/TAES.2003.1261132.

"""
    PolarCV # polar, constant velocity

Represents a point mass moving at a constant velocity.
Velocity is parameterized in polar coordinates.

# State vector layout
* `x[1]` ... X position
* `x[2]` ... Y position
* `x[3]` ... forward velocity
* `x[4]` ... heading
"""
struct PolarCV <: StateEquation{ContinuousTime} end
ninputs(::PolarCV) = 0
nstates(::PolarCV) = 4
function (::PolarCV)(x, u)
    dx = [
        x[3] * cos(x[4]),
        x[3] * sin(x[4]),
        0,
        0
    ]
    return dx
end

"""
    PolarCA # polar, constant acceleration

Represents an accelerating point mass.
Velocity is parameterized in polar coordinates;
acceleration is assumed to only act in the
forward direction.

# State vector layout
* `x[1]` ... X position
* `x[2]` ... Y position
* `x[3]` ... forward velocity
* `x[4]` ... forward acceleration
* `x[5]` ... heading
"""
struct PolarCA <: StateEquation{ContinuousTime} end
ninputs(::PolarCA) = 0
nstates(::PolarCA) = 5
function (::PolarCA)(x, u)
    dx = [
        x[3] * cos(x[5]),
        x[3] * sin(x[5]),
        x[4],
        0,
        0
    ]
    return dx
end



"""
    PolarCTRV # polar, constant turn rate, constant velocity

Represents a 2D rigid body moving along a circular path along a constant speed.
Velocity is parameterized in polar coordinates.

# State vector layout
* `x[1]` ... X position
* `x[2]` ... Y position
* `x[3]` ... forward velocity
* `x[4]` ... heading
* `x[5]` ... yaw rate
"""
struct PolarCTRV <: StateEquation{ContinuousTime} end
ninputs(::PolarCTRV) = 0
nstates(::PolarCTRV) = 5
function (::PolarCTRV)(x, u)
    # x[1] ... X position
    # x[2] ... Y position
    # x[3] ... forward velocity
    # x[4] ... heading
    # x[5] ... yaw rate
    dx = [
        x[3] * cos(x[4]),
        x[3] * sin(x[4]),
        0,
        x[5],
        0
    ]
    return dx
end

"""
    PolarCTRA # polar, constant turn rate, constant acceleration

Represents an accelerating 2D rigid body moving along a clothoidal path.
Velocity is parameterized in polar coordinates;
acceleration is assumed to only act in the
forward direction.

# State vector layout
* `x[1]` ... X position
* `x[2]` ... Y position
* `x[3]` ... forward velocity
* `x[4]` ... forward acceleration
* `x[5]` ... heading
* `x[6]` ... yaw rate
"""
struct PolarCTRA <: StateEquation{ContinuousTime} end
ninputs(::PolarCTRA) = 0
nstates(::PolarCTRA) = 6
function (::PolarCTRA)(x, u)
    dx = [
        x[3] * cos(x[5]),
        x[3] * sin(x[5]),
        x[4],
        0,
        x[6],
        0
    ]
    return dx
end


"""
    PolarCCA # polar, constant curvature, constant acceleration

Represents an accelerating rigid body moving along a circular curve.
Velocity is parameterized in polar coordinates;
acceleration is assumed to only act in the
forward direction.

# State vector layout
* `x[1]` ... X position
* `x[2]` ... Y position
* `x[3]` ... forward velocity
* `x[4]` ... forward acceleration
* `x[5]` ... heading
* `x[6]` ... track curvature
"""
struct PolarCCA <: StateEquation{ContinuousTime} end
ninputs(::PolarCCA) = 0
nstates(::PolarCCA) = 6
function (::PolarCCA)(x, u)
    dx = [
        x[3] * cos(x[5]),
        x[3] * sin(x[5]),
        x[4],
        0,
        x[3] * x[6],
        0
    ]
    return dx
end



"""
    CartesianCV # cartesian, constant velocity

Represents an point mass moving at a constant speed.
Velocity is parameterized in cartesian coordinates.

# State vector layout
* `x[1]` ... X position
* `x[2]` ... Y position
* `x[3]` ... X speed
* `x[4]` ... Y speed
"""
struct CartesianCV <: StateEquation{ContinuousTime} end
ninputs(::CartesianCV) = 0
nstates(::CartesianCV) = 4
function (::CartesianCV)(x, u)
    dx = [
        x[3],
        x[4],
        0,
        0
    ]
    return dx
end

"""
    CartesianCA # cartesian, constant acceleration

Represents an accelerating point mass.
Velocity and acceleration are parameterized in cartesian coordinates.

# State vector layout
* `x[1]` ... X position
* `x[2]` ... Y position
* `x[3]` ... X speed
* `x[4]` ... Y speed
* `x[5]` ... X acceleration
* `x[6]` ... Y acceleration
"""
struct CartesianCA <: StateEquation{ContinuousTime} end
ninputs(::CartesianCA) = 0
nstates(::CartesianCA) = 6
function (::CartesianCA)(x, u)
    # x[1] ... X position
    # x[2] ... Y position
    # x[3] ... X speed
    # x[4] ... Y speed
    # x[5] ... X acceleration
    # x[6] ... Y acceleration
    dx = [
        x[3],
        x[4],
        x[5],
        x[6],
        0,
        0
    ]
    return dx
end


"""
    CartesianCTRV # cartesian, constant turn rate, constant velocity

Represents an rotating rigid body moving at a constant speed magnitude.
Velocity is parameterized in cartesian coordinates and is
continuously rotating.

# State vector layout
* `x[1]` ... X position
* `x[2]` ... Y position
* `x[3]` ... X speed
* `x[4]` ... Y speed
* `x[5]` ... yaw rate
"""
struct CartesianCTRV <: StateEquation{ContinuousTime} end
ninputs(::CartesianCTRV) = 0
nstates(::CartesianCTRV) = 5
function (::CartesianCTRV)(x, u)
    dx = [
        x[3],
        x[4],
        -x[5] * x[4],
        +x[5] * x[3],
        0
    ]
    return dx
end

