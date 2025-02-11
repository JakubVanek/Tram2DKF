abstract type StepSizeControl end
# (iterate::StepSizeControl)(V, x0, xstep0), returns xstep

struct IdentityStepping <: StepSizeControl end
struct BacktrackingLineSearch <: StepSizeControl
    strictness::Float64
    reduction::Float64
    max_iters::Int
end

function (::IdentityStepping)(V, x0, xstep0)
    return xstep0
end

function (bls::BacktrackingLineSearch)(V, x0, xstep0)
    V0 = V(x0)
    dV0 = V'(x0)

    multiplier = 1.0

    for i in 1:bls.max_iters
        step = xstep0 * multiplier
        x = x0 + step

        obtained_decrease = V0 - V(x)
        wanted_decrease = -dV0 * norm(step) * bls.strictness

        if obtained_decrease > wanted_decrease
            return step
        else
            multiplier = multiplier * bls.reduction
        end
    end
    return zero(xstep0)
end
