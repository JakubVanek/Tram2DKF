using Zygote

abstract type Time end
abstract type DiscreteTime <: Time end
abstract type ContinuousTime <: Time end

abstract type StateEquation{T <: Time} end
# (fn)(x,u,t)
# ninputs(fn)
# nstates(fn)

abstract type MeasurementEquation{T <: Time} end
# (fn)(x,u,t)
# ninputs(fn)
# nstates(fn)
# noutputs(fn)

struct LTIStateEquation{T <: Time} <: StateEquation{T}
    A::Matrix{Float64}
    B::Matrix{Float64}
    function LTIStateEquation{T}(A, B) where {T <: Time}
        size(A, 1) == 0          && error("Matrix A must have at least one row")
        size(A, 1) != size(A, 2) && error("Matrix A must be rectangular")
        size(A, 1) != size(B, 1) && error("Matrix B must be as tall as matrix A is")
        new{T}(A, B)
    end
end
(model::LTIStateEquation)(x,u,t) = ninputs(model) == 0 ? model.A*x : model.A*x + model.B*u
nstates(model::LTIStateEquation) = size(model.B, 1)
ninputs(model::LTIStateEquation) = size(model.B, 2)




struct LTIMeasurementEquation{T <: Time} <: MeasurementEquation{T}
    C::AbstractMatrix{Float64}
    D::AbstractMatrix{Float64}
    function LTIMeasurementEquation{T}(C, D) where {T <: Time}
        size(C, 1) == 0          && error("Matrix C must have at least one row")
        size(C, 1) != size(D, 1) && error("Matrix D must be as tall as matrix C is")
        new{T}(C, D)
    end
end
(model::LTIMeasurementEquation)(x,u,t) = ninputs(model) == 0 ? model.C*x : model.C*x + model.D*u
noutputs(model::LTIMeasurementEquation) = size(model.C, 1)
nstates(model::LTIMeasurementEquation)  = size(model.C, 2)
ninputs(model::LTIMeasurementEquation) = size(model.D, 2)


abstract type DiscretizationAlgorithm end
# (algo)(model,x,u,t,dt)

struct Euler <: DiscretizationAlgorithm end
(::Euler)(f,x,u,t,dt) = f(x,u,t)

struct RK4 <: DiscretizationAlgorithm end
function (::RK4)(f,x,u,t,dt)
    # https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    k1 = f(x,           u, t)
    k2 = f(x + k1*dt/2, u, t + dt/2)
    k3 = f(x + k2*dt/2, u, t + dt/2)
    k4 = f(x + k3*dt,   u, t + dt)
    (k1 + 2*k2 + 3*k3 + k4)/6
end

struct DiscretizedStateEquation{SubmodelT <: StateEquation{ContinuousTime}, MethodT <: DiscretizationAlgorithm} <: StateEquation{DiscreteTime}
    f::SubmodelT
    method::MethodT
    Ts::Float64
    subsamples::Int
end

function discretize(model::StateEquation{ContinuousTime}, method::DiscretizationAlgorithm, sampling_time::Float64, subsamples::Int = 1)
    subsamples <= 0 && error("Number of subsamples must be at least one")
    !isfinite(sampling_time) && error("Sampling time has to be a real finite value")
    sampling_time <= 0.0 && error("Sampling time has to be a positive value")

    DiscretizedStateEquation{typeof(model), typeof(method)}(model, method, sampling_time, subsamples)
end

function (discretizer::DiscretizedStateEquation)(x, u, k)
    dt = discretizer.Ts / discretizer.subsamples
    x_now = x
    t_now = k*discretizer.Ts
    for i = 1:discretizer.subsamples
        x_now += discretizer.method(discretizer.f, x_now, u, t_now, dt) * dt
        t_now += dt
    end

    x_now
end
ninputs(model::DiscretizedStateEquation)  = ninputs(model.f)
nstates(model::DiscretizedStateEquation)  = nstates(model.f)


struct DiscretizedMeasurementEquation{SubmodelT <: MeasurementEquation{ContinuousTime}} <: MeasurementEquation{DiscreteTime}
    g::SubmodelT
    Ts::Float64
end

function discretize(model::MeasurementEquation{ContinuousTime}, sampling_time::Float64)
    !isfinite(sampling_time) && error("Sampling time has to be a real finite value")
    sampling_time <= 0.0 && error("Sampling time has to be a positive value")

    DiscretizedMeasurementEquation{typeof(model)}(model, sampling_time)
end

(discretizer::DiscretizedMeasurementEquation)(x, u, k) = discretizer.g(x, u, k*discretizer.Ts)
nstates(model::DiscretizedMeasurementEquation)  = nstates(model.g)
noutputs(model::DiscretizedMeasurementEquation) = noutputs(model.g)
ninputs(model::DiscretizedMeasurementEquation) = ninputs(model.g)


function linearize(f::StateEquation{TimeT}, x, u, t) where {TimeT <: Time}
    A = jacobian((new_x) -> f(new_x, u, t), x)[1]
    if ninputs(f) > 0
        B = jacobian((new_u) -> f(x, new_u, t), x)[1]
    else
        B = zeros(nstates(f), 0)
    end
    LTIStateEquation{TimeT}(A, B)
end

function linearize(g::MeasurementEquation{TimeT}, x, u, t) where {TimeT <: Time}
    C = jacobian((new_x) -> g(new_x, u, t), x)[1]
    if ninputs(g) > 0
        D = jacobian((new_u) -> g(x, new_u, t), x)[1]
    else
        D = zeros(noutputs(g), 0)
    end
    LTIMeasurementEquation{TimeT}(C, D)
end
