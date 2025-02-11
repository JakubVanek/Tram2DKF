@testset "Identity stepper" begin
    linear_fn(x) = 2*x

    stepper = IdentityStepping()
    # just passthrough the step
    @test stepper(linear_fn, 2, +10) == +10
    @test stepper(linear_fn, 2, -10) == -10
end


@testset "Backtracking stepper" begin
    linear_fn(x) = 2*x
    quadratic_fn(x) = x^2

    stepper = BacktrackingLineSearch(0.1, 0.5, 20)

    # minimizing direction -> use directly
    @test stepper(linear_fn, 0, -1) == -1
    # maximizing direction -> runs out of iterations and returns zero
    @test stepper(linear_fn, 0, +1) == 0
    # prevent overshoot in quadratic minimization
    @test stepper(quadratic_fn, 1, -3) > -2 # this would move the point to -1
    @test stepper(quadratic_fn, 1, -3) < 0  # this would not move the point at all
end
