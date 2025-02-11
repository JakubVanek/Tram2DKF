# struct SpinnyIntegrator reused from EKF tests
# struct PowerMeasurement reused from EKF tests

IEKF = IteratedExtendedKalmanFilter(BacktrackingLineSearch(0.1, 0.5, 20), 1e-6, 20)

@testset "IEKF time step" begin
    # the same as for EKF
    model = SpinnyIntegrator()

    # standard prediction
    prev = Gaussian([0.0], [1.0;;])
    noise = Gaussian([0.0], [1.0;;])
    next = forward_step(IEKF, model, prev, [], noise)
    @test mean(next) == [1.0]
    @test covariance(next) == [2.0;;]

    # prediction with LLT-factorized Gaussians
    prev_sqrt = SqrtGaussian([0.0], LowerTriangular([1.0;;]))
    noise_sqrt = SqrtGaussian([0.0], LowerTriangular([1.0;;]))
    next_sqrt = forward_step(IEKF, model, prev_sqrt, [], noise_sqrt)
    @test mean(next_sqrt) ≈ [1.0]
    @test covariance(next_sqrt) ≈ [2.0;;]
end


@testset "IEKF data step" begin
    model = PowerMeasurement()
    prior = Gaussian([1.0], [1.0;;])
    measurement = Gaussian([4.0], [1e-10;;])

    # standard filtering
    posterior = data_step(IEKF, model, prior, [], measurement)
    @test mean(posterior) ≈ [2.0]
    @test covariance(posterior)[1,1] <= 1e-10

    # filtering with LLT-factorized Gaussians
    prior_sqrt = SqrtGaussian([1.0], LowerTriangular([1.0;;]))
    measurement_sqrt = SqrtGaussian([4.0], LowerTriangular([1e-5;;]))
    posterior_sqrt = data_step(IEKF, model, prior_sqrt, [], measurement_sqrt)
    @test mean(posterior_sqrt) ≈ [2.0]
    @test covariance(posterior_sqrt)[1,1] <= 1e-10
end
