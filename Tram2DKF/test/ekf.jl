struct SpinnyIntegrator <: StateEquation{DiscreteTime} end
(::SpinnyIntegrator)(x, u) = [1 + sin(x[1])]
nstates(::SpinnyIntegrator) = 1
ninputs(::SpinnyIntegrator) = 0

struct PowerMeasurement <: MeasurementEquation end
(::PowerMeasurement)(x, u) = [x[1]^2]
ninputs(::PowerMeasurement) = 0
nstates(::PowerMeasurement) = 1
noutputs(::PowerMeasurement) = 1

EKF = ExtendedKalmanFilter()

@testset "EKF time step" begin
    model = SpinnyIntegrator()
    prev = Gaussian([0.0], [1.0;;])
    noise = Gaussian([0.0], [1.0;;])
    next = forward_step(EKF, model, prev, [], noise)

    @test mean(next) == [1.0]
    @test covariance(next) == [2.0;;]
end


@testset "EKF data step" begin
    model = PowerMeasurement()
    prior = Gaussian([1.0], [1.0;;])
    measurement = Gaussian([1.0], [1.0;;])
    posterior = data_step(EKF, model, prior, [], measurement)

    @test mean(posterior) ≈ [1.0]
    @test covariance(posterior) ≈ [0.2;;]
end
