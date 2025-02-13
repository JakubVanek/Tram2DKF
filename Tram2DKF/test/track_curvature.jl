using Tram2DKF: StraightTrack, TrackCurvature, TrackTurn, activate, curvature
import Base.isapprox

isapprox(a::TrackCurvature, b::TrackCurvature) = (a.curvature ≈ b.curvature) && (a.dcurvature ≈ b.dcurvature)

@testset "Track curvature helpers" begin

    @testset "Straight track" begin
        segment = activate(StraightTrack(100.0), 10.0)
        @test curvature(segment, 10.0) == TrackCurvature(curvature = 0.0, dcurvature = 0.0)
        @test curvature(segment, 60.0) == TrackCurvature(curvature = 0.0, dcurvature = 0.0)
        @test curvature(segment, 109.0) == TrackCurvature(curvature = 0.0, dcurvature = 0.0)
        @test curvature(segment, 110.0) === nothing
        @test curvature(segment, 200.0) === nothing
    end

    @testset "Sharp left turn" begin
        angle = pi/2
        radius = 10.0
        segment = activate(TrackTurn(angle, radius, 0.0), 0.0)
        @test curvature(segment, 0.0)            == TrackCurvature(curvature = 1/radius, dcurvature = 0.0)
        @test curvature(segment, angle/2*radius) == TrackCurvature(curvature = 1/radius, dcurvature = 0.0)
        @test curvature(segment, angle*radius)   === nothing
    end

    @testset "Sharp right turn" begin
        angle = -pi/2
        radius = 10.0
        segment = activate(TrackTurn(angle, radius, 0.0), 0.0)
        @test curvature(segment, 0.0)                 == TrackCurvature(curvature = -1/radius, dcurvature = 0.0)
        @test curvature(segment, abs(angle/2)*radius) == TrackCurvature(curvature = -1/radius, dcurvature = 0.0)
        @test curvature(segment, abs(angle)*radius)   === nothing
    end


    @testset "Left turn with transitions" begin
        angle = pi/2
        radius = 10.0
        segment = activate(TrackTurn(angle, radius, 1.0), 0.0)
        @test curvature(segment, 0.0)            == TrackCurvature(curvature = 0.0, dcurvature = 0.1)
        @test curvature(segment, angle/2*radius) == TrackCurvature(curvature = 1/radius, dcurvature = 0.0) # hopefully
        @test curvature(segment, 100.0)          === nothing
    end


    @testset "Transition-only left turn" begin
        angle = pi/2
        radius = 10.0
        segment = activate(TrackTurn(angle, radius, 100.0), 0.0)
        @test curvature(segment, 0.0)            ≈ TrackCurvature(curvature = 0.0, dcurvature = 0.001)
        @test curvature(segment, 1000.0)         === nothing # hopefully
    end
end

