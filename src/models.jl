using ForwardDiff

"""
    Time

Marker type for discriminating between continuous-time and
discrete-time models. Typically one of [`DiscreteTime`](@ref)
and [`ContinuousTime`](@ref).

See also [`StateEquation`](@ref).
"""
abstract type Time end

"""
    DiscreteTime

Marker indicating that the [`StateEquation`](@ref) operates
in discrete time.
"""
abstract type DiscreteTime <: Time end

"""
    ContinuousTime

Marker indicating that the [`StateEquation`](@ref) operates
in continuous time.
"""
abstract type ContinuousTime <: Time end

"""
    StateEquation{T <: Time}

State-space dynamics model for an abstract system.

The model is represented by a first-order
explicit vector ODE. DAE models are not supported.

Whether the system is evolving in continuous or discrete
time is determined by the T type argument. See
[`DiscreteTime`](@ref) and [`ContinuousTime`](@ref).

# Interface

Users can expect values of this type to have the following methods:

    (eq::StateEquation)(x, u)

Advance the state of the system:
- continuous-time systems return the derivative of the state at the current time
- discrete-time systems return the new state at the next time instant

Dependence on time is not explicitly modelled. If needed,
the time can be obtained by introducing an additional state
with derivative equal to 1.

    ninputs(eq::StateEquation)

Expected length of the control vector `u`.

    nstates(eq::StateEquation)

Expected length of the state vector `x`.
"""
abstract type StateEquation{T <: Time} end

"""
    MeasurementEquation

Model of a system state measurement.

The model is represented as an explicit function.
Implicit measurement equations are not supported.

# Interface
Users can expect values of this type to have the following methods:

    y = (eq::MeasurementEquation)(x, u)

Predict the measurement value based on the current state and return
it as a vector.

    ninputs(eq::StateEquation)

Expected length of the control vector `u`.

    nstates(eq::StateEquation)

Expected length of the state vector `x`.

    noutputs(eq::StateEquation)

Expected length of the measurement vector `y`.

"""
abstract type MeasurementEquation end

"""
    CompositeMeasurement

Glues multiple independent measurements models into one measurement vector.
"""
struct CompositeMeasurement{MultipleMeasurements} <: MeasurementEquation
    "Sub-measurements whose results to concatenate."
    pieces::MultipleMeasurements
end

"""
    (msr::CompositeMeasurement)(x, u)

Evaluate specified sub-measurements and concatenate
their results into one large measurement vector.
"""
(msr::CompositeMeasurement)(x, u) = reduce(vcat, msr.pieces .|> piece -> piece(x,u))
ninputs(msr::CompositeMeasurement) = ninputs(msr.pieces[1])
nstates(msr::CompositeMeasurement) = nstates(msr.pieces[1])
noutputs(msr::CompositeMeasurement) = sum(msr.pieces .|> noutputs)

"""
    LTIStateEquation{T <: Time}

Linear time-invariant state-space model.
"""
struct LTIStateEquation{T <: Time, SM <: AbstractMatrix{<:Real}, CM <: AbstractMatrix{<:Real}} <: StateEquation{T}
    "System matrix"
    A::SM

    "Control matrix"
    B::CM

    """
        LTIStateEquation{T}(A, B) where {T <: Time}

    Create a new LTI model with `A` and `B` matrices.
    """
    function LTIStateEquation{T}(A::SM, B::CM) where {T <: Time, SM <: AbstractMatrix{<:Real}, CM <: AbstractMatrix{<:Real}}
        size(A, 1) == 0          && error("Matrix A must have at least one row")
        size(A, 1) != size(A, 2) && error("Matrix A must be rectangular")
        size(A, 1) != size(B, 1) && !isempty(B) && error("Matrix B must be as tall as matrix A is")
        return new{T, SM, CM}(A, B)
    end
end
(model::LTIStateEquation)(x,u) = ninputs(model) == 0 ? model.A*x : model.A*x + model.B*u
nstates(model::LTIStateEquation) = size(model.B, 1)
ninputs(model::LTIStateEquation) = size(model.B, 2)


"""
    LTIMeasurementEquation

Linear time-invariant measurement model.
"""
struct LTIMeasurementEquation{MM <: AbstractMatrix{<:Real}, FM <: AbstractMatrix{<:Real}} <: MeasurementEquation
    "Measurement matrix"
    C::MM

    "Feed-through matrix"
    D::FM

    """
        LTIMeasurementEquation(C, D)

    Create a new LTI measurement model with `C` and `D` matrices.
    """
    function LTIMeasurementEquation(C::MM, D::FM) where {MM <: AbstractMatrix{<:Real}, FM <: AbstractMatrix{<:Real}}
        size(C, 1) == 0          && error("Matrix C must have at least one row")
        size(C, 1) != size(D, 1) && !isempty(D) && error("Matrix D must be as tall as matrix C is")
        return new{MM, FM}(C, D)
    end
end
(model::LTIMeasurementEquation)(x,u) = ninputs(model) == 0 ? model.C*x : model.C*x + model.D*u
noutputs(model::LTIMeasurementEquation) = size(model.C, 1)
nstates(model::LTIMeasurementEquation)  = size(model.C, 2)
ninputs(model::LTIMeasurementEquation) = size(model.D, 2)


"""
    Euler(f,x,u,dt)

Run a single step of the Euler discretization method.

Return the state at which the system specified by the ODE `f`
will be after `dt` seconds if the current state is `x` and
control is `u`.
"""
Euler(f,x,u,dt) = x + f(x,u)*dt

"""
    RK4(f,x,u,dt)

Run a single step of the Runge-Kutta order 4 discretization method.

Return the state at which the system specified by the ODE `f`
will be after `dt` seconds if the current state is `x` and
control is `u`.
"""
function RK4(f,x,u,dt)
    # https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    k1 = f(x,           u)
    k2 = f(x + k1*dt/2, u)
    k3 = f(x + k2*dt/2, u)
    k4 = f(x + k3*dt,   u)
    return x + (k1 + 2*k2 + 2*k3 + k4)/6 * dt
end

"""
    DiscretizedStateEquation{SubmodelT <: StateEquation{ContinuousTime}, DiscretizationMethodT, TimeT}

Helper object that wraps a continuous-time system and behaves
like a discrete-time system.
"""
struct DiscretizedStateEquation{SubmodelT <: StateEquation{ContinuousTime}, DiscretizationMethodT, TimeT} <: StateEquation{DiscreteTime}
    "Continuous-time system to discretize."
    f::SubmodelT

    """
    ODE discretization method. The value must be callable as

        method(f,x,u,dt)

    where `f` is the ODE, `x` is system state, `u` is control and `dt`
    is the discretization time step. The function must return the
    state after time `dt`.
    """
    method::DiscretizationMethodT

    """
    Length of the main discretization time step.

    This determines how the discretized system will appear to the outside world.
    """
    Ts::TimeT

    """
    Split the main time step into this many finer intervals.

    This allows for more precise discretization: this way, the discretization
    method can be invoked `subsamples`-times over a `Ts/subsamples` interval.
    """
    subsamples::Int
end


"""
    discretize(model::StateEquation{ContinuousTime}, method, sampling_time, subsamples::Int = 1)

Discretize the given `model` using the discretization `method` with given `sampling_time`.

Additionally, each `sampling_time` interval can be internally split into `subsamples`
sub-intervals over which will the discretization `method`` be repeatedly invoked.
"""
function discretize(model::StateEquation{ContinuousTime}, method, sampling_time, subsamples::Int = 1)
    subsamples <= 0 && error("Number of subsamples must be at least one")
    !isfinite(sampling_time) && error("Sampling time has to be a real finite value")
    sampling_time <= 0.0 && error("Sampling time has to be a positive value")

    return DiscretizedStateEquation(model, method, sampling_time, subsamples)
end

# implementation of the StateEquation{DiscreteTime} interface

function (discretizer::DiscretizedStateEquation)(x, u)
    dt = discretizer.Ts / discretizer.subsamples
    x_now = x
    for _ = 1:discretizer.subsamples
        x_now = discretizer.method(discretizer.f, x_now, u, dt)
    end
    return x_now
end
ninputs(model::DiscretizedStateEquation)  = ninputs(model.f)
nstates(model::DiscretizedStateEquation)  = nstates(model.f)


"""
    linearize(f::StateEquation{TimeT}, x, u) where {TimeT <: Time}

Linearize the given nonlinear dynamic model `f`
at the operating point specified by state `x` and control `u`.
"""
function linearize(f::StateEquation{TimeT}, x, u) where {TimeT <: Time}
    A = ForwardDiff.jacobian((x_) -> f(x_, u), x)::Matrix{eltype(x)}
    if ninputs(f) == 0
        B = zeros(eltype(x), nstates(f), 0)::Matrix{eltype(x)}
    else
        B = ForwardDiff.jacobian((u_) -> f(x, u_), u)::Matrix{eltype(x)}
    end
    return LTIStateEquation{TimeT}(A, B)
end


"""
    linearize(g::MeasurementEquation, x, u)

Linearize the given nonlinear measurement model `g`
at the operating point specified by state `x` and control `u`.
"""
function linearize(g::MeasurementEquation, x, u)
    C = ForwardDiff.jacobian((x_) -> g(x_, u), x)::Matrix{eltype(x)}
    if ninputs(g) == 0
        D = zeros(eltype(x), noutputs(g), 0)::Matrix{eltype(x)}
    else
        D = ForwardDiff.jacobian((u_) -> g(x, u_), u)::Matrix{eltype(x)}
    end
    return LTIMeasurementEquation(C, D)
end
