using Tram2DKF
using Test
using JET

@testset "Tram2DKF.jl" begin
    @testset "Code linting (JET.jl)" begin
        JET.test_package(Tram2DKF; target_defined_modules = true)
    end
    include("utils.jl")
    include("models.jl")
    include("kf.jl")
    include("distributions.jl")
    include("ekf.jl")
    include("linesearch.jl")
    include("iekf.jl")
    include("track_speedprofile.jl")
    include("track_curvature.jl")
    include("track_generator.jl")
end
