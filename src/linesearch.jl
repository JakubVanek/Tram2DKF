"""
    IdentityStepping(V, x0, xstep0)

Noop step size control: pass through the initial step size `xstep0`.
"""
IdentityStepping(V, x0, xstep0) = xstep0

"""
    AlwaysFeasible(x)

Noop feasibility check - all X are feasible.
"""
AlwaysFeasible(x) = true

"""
    BacktrackingLineSearch{T <: Real}

Optimize step length by iteratively shortening the step and
checking if the Armijo condition holds.

The Armijo condition holds if the function has decreased
enough compared to its derivative at the origin point.

Additionally, the function can check if the resulting vector
falls into a feasible region. The check is provided by
a user-defined function.
"""
@kwdef struct BacktrackingLineSearch{T <: Real, FeasibilityCheck}
    """
    The function has to decrease at least this value times
    the decrease that a linear approximation would achieve at the
    new point.

    Acceptable values are between 0-1.
    """
    strictness::T = 0.1

    """
    When a step needs to be shortened, multiply its length by this factor.

    Acceptable values are between 0-1; good values might be less than 0.5
    (see <https://math.stackexchange.com/a/3932268>).
    """
    reduction::T = 0.5

    "Iterate at most this many times."
    max_iters::Int = 20

    "Function for checking if given X is feasible."
    isfeasible::FeasibilityCheck = AlwaysFeasible
end


"""
    (bls::BacktrackingLineSearch)(V, x0, xstep0)

Optimize step length via backtracking line search.

The function will find such a step to satisfy the Armijo
condition when minimizing function `V` in
the direction `xstep0` when coming from point `x0`.

This function assumes that the `x0`` is feasible
(the `bls.isfeasible` function shall return true at `x0`).

The new step vector is returned.
"""
function (bls::BacktrackingLineSearch)(V, x0, xstep0)
    V0 = V(x0)
    if x0 isa AbstractVector{<:Real}
        dV0 = ForwardDiff.gradient(V, x0)
    elseif x0 isa Number
        dV0 = ForwardDiff.derivative(V, x0)
    else
        error("x0 has unknown data type")
    end
    full_step_threshold = -(dV0 ⋅ xstep0) * bls.strictness

    multiplier = 1.0

    for _ in 1:bls.max_iters
        step = xstep0 * multiplier
        x = x0 + step

        if bls.isfeasible(x)
            obtained_decrease = V0 - V(x)
            wanted_decrease = full_step_threshold * multiplier

            if obtained_decrease > wanted_decrease
                return step
            end
        end

        multiplier = multiplier * bls.reduction
    end

    step = xstep0 * multiplier
    x = x0 + step
    if bls.isfeasible(x) && V(x) < V0
        return step
    else
        return zero(step)
    end
end
