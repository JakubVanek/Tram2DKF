@testset "Track generation" begin
    @testset "Short drive forward" begin
        using Tram2DKF: IDX_ACCELERATION, IDX_SPEED

        track::Vector{<:TrackSegment} = [
            StraightTrack(distance=100.0), # test that jumping to next segment works
            StraightTrack(distance=900.0)
        ]

        trip::Vector{<:TrajectorySegment} = [
            Stop(duration=1.0),
            Accelerate(to_speed=10.0, acceleration=1.0),
            ConstantSpeed(speed=10.0, distance=100.0),
            Accelerate(to_speed=0.0, acceleration=1.0),
            Stop(duration=10.0)
        ]

        states = render_trip(track, trip, 0.1)

        # initial state
        @test states[1] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # acceleration probes
        @test states[50][IDX_ACCELERATION]  == 1.0
        @test states[150][IDX_ACCELERATION] == 0.0
        @test states[250][IDX_ACCELERATION] == -1.0

        # speed probes
        @test states[50][IDX_SPEED]  > 0.0
        @test states[50][IDX_SPEED]  < 10.0
        @test states[150][IDX_SPEED] == 10.0
        @test states[250][IDX_SPEED] < 10.0
        @test states[250][IDX_SPEED] > 0.0
    end
end
