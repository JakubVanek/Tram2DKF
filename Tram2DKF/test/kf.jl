DtIntegrator = LTIStateEquation{DiscreteTime}([1.0;;], [1.0;;])
JustState    = LTIMeasurementEquation([1.0;;], [0.0;;])
LKF          = LinearKalmanFilter()

@testset "LKF time step" begin
    prev = Gaussian([0.0], [1.0;;])
    noise = Gaussian([0.0], [1.0;;])
    control = [1.0]
    next = forward_step(LKF, DtIntegrator, prev, control, noise)

    @test mean(next) == [1.0]
    @test covariance(next) == [2.0;;]
end


@testset "LKF data step" begin
    prior = Gaussian([0.0], [1.0;;])
    measurement = Gaussian([1.0], [1.0;;])
    posterior = data_step(LKF, JustState, prior, measurement)

    @test mean(posterior) == [0.5]
    @test covariance(posterior) == [0.5;;]
end


@testset "SQRT LKF time step" begin
    prev = SqrtGaussian(Gaussian([0.0], [1.0;;]))
    noise = SqrtGaussian(Gaussian([0.0], [1.0;;]))
    control = [1.0]
    next = forward_step(LKF, DtIntegrator, prev, control, noise)

    @test typeof(next) == SqrtGaussian
    @test mean(next) ≈ [1.0]
    @test covariance(next) ≈ [2.0;;]
end


@testset "SQRT LKF data step" begin
    prior = SqrtGaussian(Gaussian([0.0], [1.0;;]))
    measurement = SqrtGaussian(Gaussian([1.0], [1.0;;]))
    posterior = data_step(LKF, JustState, prior, measurement)

    @test typeof(posterior) == SqrtGaussian
    @test mean(posterior) ≈ [0.5]
    @test covariance(posterior) ≈ [0.5;;]
end
