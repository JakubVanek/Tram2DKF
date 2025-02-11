using Tram2DKF: lerp, dlerp

@testset "Linear interpolation tests" begin
    # rising slope, interpolation
    @test lerp(0, 0, 1, 2, 0.5) == 1.0
    # rising slope, extrapolation
    @test lerp(0, 0, 1, 2, 1.5) == 3.0
    # rising slope, derivative
    @test dlerp(0, 0, 1, 2) == 2.0

    # falling slope, interpolation
    @test lerp(-1, +2, 1, -2, 0.5) == -1.0
    # falling slope, extrapolation
    @test lerp(-1, +2, 1, -2, 1.5) == -3.0
    # falling slope, derivative
    @test dlerp(-1, +2, 1, -2) == -2.0
end
