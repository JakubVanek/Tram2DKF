using Zygote

abstract type ContinuousStateSpaceModel{NumberT <: Real} end
# fc(mod,x,u,t)
# g(mod,x,u,t)
# nstates(mod)
# ninputs(mod)
# noutputs(mod)

abstract type DiscreteStateSpaceModel{NumberT <: Real} end
# fd(mod,x,u,k)
# g(mod,x,u,k)
# nstates(mod)
# ninputs(mod)
# noutputs(mod)

abstract type LinearDiscreteStateSpaceModel{NumberT <: Real} <: DiscreteStateSpaceModel{NumberT} end
# A(mod,x,u,t)
# B(mod,x,u,t)
# C(mod,x,u,t)
# D(mod,x,u,t)
# nstates(mod)
# ninputs(mod)
# noutputs(mod)

struct EulerDiscretized{NumberT <: Real,SubmodelT <: ContinuousStateSpaceModel{<:NumberT}} <: DiscreteStateSpaceModel{NumberT}
    continuous::SubmodelT
    sampling_time::NumberT
    subsamples::Int

    function EulerDiscretized(continuous_model::ContinuousStateSpaceModel{<:NumberT}, sampling_time::NumberT, subsamples::Int = 1) where NumberT <: Real
        subsamples <= 0 && error("Number of subsamples must be at least one")
        sampling_time <= zero(NumberT) && error("Sampling time has to be a positive value")
        new{NumberT, typeof(continuous_model)}(continuous_model, sampling_time, subsamples)
    end
end

function fd(model::EulerDiscretized, x, u, k)
    t_step = model.sampling_time / model.subsamples
    x_now = x
    t_now = k*model.sampling_time
    for i = 1:model.subsamples
        x_now = fc(model.continuous, x_now, u, t_now) * t_step
        t_now += t_step
    end

    x_now
end
g(model::EulerDiscretized, x, u, k) = g(model.continuous, x, u, k*model.sampling_time)
nstates(model::EulerDiscretized)  = nstates(model.continuous)
ninputs(model::EulerDiscretized)  = ninputs(model.continuous)
noutputs(model::EulerDiscretized) = noutputs(model.continuous)


struct Linearized{NumberT <: Real, SubmodelT <: DiscreteStateSpaceModel{<:NumberT}} <: LinearDiscreteStateSpaceModel{NumberT}
    nonlinear::SubmodelT
end

A(mod::Linearized,x,u,k) = jacobian((new_x) -> f(mod, new_x, u, k), x)[0]
B(mod::Linearized,x,u,k) = jacobian((new_u) -> f(mod, x, new_u, k), x)[0]
C(mod::Linearized,x,u,k) = jacobian((new_x) -> g(mod, new_x, u, k), x)[0]
D(mod::Linearized,x,u,k) = jacobian((new_u) -> g(mod, x, new_u, k), x)[0]
nstates(mod::Linearized) = nstates(mod.nonlinear)
ninputs(mod::Linearized) = ninputs(mod.nonlinear)
noutputs(mod::Linearized) = noutputs(mod.nonlinear)
