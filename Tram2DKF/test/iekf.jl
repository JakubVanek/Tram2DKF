# struct SpinnyIntegrator reused from EKF tests
# struct PowerMeasurement reused from EKF tests

IEKF = IteratedExtendedKalmanFilter(BacktrackingLineSearch(0.1, 0.5, 20), 1e-6, 20)

@testset "IEKF time step" begin
    # the same as for EKF
    model = SpinnyIntegrator()
    prev = Gaussian([0.0], [1.0;;])
    noise = Gaussian([0.0], [1.0;;])
    next = forward_step(IEKF, model, prev, [], noise)

    @test mean(next) == [1.0]
    @test covariance(next) == [2.0;;]
end


@testset "IEKF data step" begin
    model = PowerMeasurement()
    prior = Gaussian([1.0], [1.0;;])
    measurement = Gaussian([4.0], [1e-9;;])
    posterior = data_step(IEKF, model, prior, [], measurement)

    @test mean(posterior) ≈ [2.0]
    @test covariance(posterior)[1,1] <= 1e-9
end
