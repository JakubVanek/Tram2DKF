DtIntegrator     = LTIStateEquation{DiscreteTime}([1.0;;], [1.0;;])
DtIntegratorNoIn = LTIStateEquation{DiscreteTime}([1.0;;], zeros(1,0))
JustState        = LTIMeasurementEquation([1.0;;], [0.0;;])
JustStateNoIn    = LTIMeasurementEquation([1.0;;], [;;])
LKF              = LinearKalmanFilter()

@testset "LKF time step" begin
    @testset "With input" begin
        prev = Gaussian([0.0], [1.0;;])
        noise = Gaussian([0.0], [1.0;;])
        next = forward_step(LKF, DtIntegrator, prev, [1.0], noise)

        @test mean(next) == [1.0]
        @test covariance(next) == [2.0;;]
    end
    @testset "No input" begin
        prev = Gaussian([0.0], [1.0;;])
        noise = Gaussian([0.0], [1.0;;])
        next = forward_step(LKF, DtIntegratorNoIn, prev, [], noise)

        @test mean(next) == [0.0]
        @test covariance(next) == [2.0;;]
    end
end


@testset "LKF data step" begin
    @testset "With input" begin
        prior = Gaussian([0.0], [1.0;;])
        measurement = Gaussian([1.0], [1.0;;])
        posterior = data_step(LKF, JustState, prior, [0.0], measurement)

        @test mean(posterior) == [0.5]
        @test covariance(posterior) == [0.5;;]
    end
    @testset "No input" begin
        prior = Gaussian([0.0], [1.0;;])
        measurement = Gaussian([1.0], [1.0;;])
        posterior = data_step(LKF, JustStateNoIn, prior, [], measurement)

        @test mean(posterior) == [0.5]
        @test covariance(posterior) == [0.5;;]
    end
end


@testset "SQRT LKF time step" begin
    @testset "With input" begin
        prev = SqrtGaussian(Gaussian([0.0], [1.0;;]))
        noise = SqrtGaussian(Gaussian([0.0], [1.0;;]))
        next = forward_step(LKF, DtIntegrator, prev, [1.0], noise)

        @test typeof(next) == SqrtGaussian
        @test mean(next) ≈ [1.0]
        @test covariance(next) ≈ [2.0;;]
    end
    @testset "No input" begin
        prev = SqrtGaussian(Gaussian([0.0], [1.0;;]))
        noise = SqrtGaussian(Gaussian([0.0], [1.0;;]))
        next = forward_step(LKF, DtIntegratorNoIn, prev, [], noise)

        @test typeof(next) == SqrtGaussian
        @test mean(next) ≈ [0.0]
        @test covariance(next) ≈ [2.0;;]
    end
end


@testset "SQRT LKF data step" begin
    @testset "With input" begin
        prior = SqrtGaussian(Gaussian([0.0], [1.0;;]))
        measurement = SqrtGaussian(Gaussian([1.0], [1.0;;]))
        posterior = data_step(LKF, JustState, prior, [0.0], measurement)

        @test typeof(posterior) == SqrtGaussian
        @test mean(posterior) ≈ [0.5]
        @test covariance(posterior) ≈ [0.5;;]
    end
    @testset "No input" begin
        prior = SqrtGaussian(Gaussian([0.0], [1.0;;]))
        measurement = SqrtGaussian(Gaussian([1.0], [1.0;;]))
        posterior = data_step(LKF, JustStateNoIn, prior, [], measurement)

        @test typeof(posterior) == SqrtGaussian
        @test mean(posterior) ≈ [0.5]
        @test covariance(posterior) ≈ [0.5;;]
    end
end
