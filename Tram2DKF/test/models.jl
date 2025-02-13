import Tram2DKF: nstates, ninputs, noutputs

struct SimpleCTEquation <: StateEquation{ContinuousTime} end
(::SimpleCTEquation)(x,u) = [x[1] + u[1]]
ninputs(::SimpleCTEquation) = 1
nstates(::SimpleCTEquation) = 1

@testset "System linearization" begin
    linmodel = linearize(SimpleCTEquation(), [0], [0])
    @test linmodel.A == [1;;]
    @test linmodel.B == [1;;]
    @test ninputs(linmodel) == 1
    @test nstates(linmodel) == 1
end


struct NoInputCTEquation <: StateEquation{ContinuousTime} end
(::NoInputCTEquation)(x,u) = [x[1]]
ninputs(::NoInputCTEquation) = 0
nstates(::NoInputCTEquation) = 1

@testset "System linearization with no input" begin
    linmodel = linearize(NoInputCTEquation(), [0], [0])
    @test linmodel.A == [1;;]
    @test isempty(linmodel.B)
    @test ninputs(linmodel) == 0
    @test nstates(linmodel) == 1
end


struct PowerMsrEquation <: MeasurementEquation end
(::PowerMsrEquation)(x,u) = [x[1]^2]
ninputs(::PowerMsrEquation) = 0
nstates(::PowerMsrEquation) = 1
noutputs(::PowerMsrEquation) = 1

@testset "Measurement linearization" begin
    linmodel = linearize(PowerMsrEquation(), [1], [0])
    @test linmodel.C == [2;;]
    @test isempty(linmodel.D)
    @test ninputs(linmodel) == 0
    @test nstates(linmodel) == 1
    @test noutputs(linmodel) == 1
end


struct Integrator <: StateEquation{ContinuousTime} end
(::Integrator)(x,u) = [u[1]]
ninputs(::Integrator) = 1
nstates(::Integrator) = 1

@testset "Euler discretization" begin
    ct_model = Integrator()
    dt_model = discretize(ct_model, Euler(), 2.0)

    @test ninputs(dt_model) == 1
    @test nstates(dt_model) == 1
    @test dt_model([0], [1]) == [2]
    @test dt_model([0], [2]) == [4]
end


struct MeasA <: MeasurementEquation end
(::MeasA)(x,u) = [x[1]]
ninputs(::MeasA) = 0
nstates(::MeasA) = 1
noutputs(::MeasA) = 1

struct MeasB <: MeasurementEquation end
(::MeasB)(x,u) = [x[1]^2]
ninputs(::MeasB) = 0
nstates(::MeasB) = 1
noutputs(::MeasB) = 1

@testset "Composite measurement" begin
    composite = CompositeMeasurement([MeasA(), MeasB()])
    @test ninputs(composite) == 0
    @test noutputs(composite) == 2
    @test nstates(composite) == 1
    @test composite([0], []) == [0, 0]
    @test composite([1], []) == [1, 1]
    @test composite([2], []) == [2, 4]
end
