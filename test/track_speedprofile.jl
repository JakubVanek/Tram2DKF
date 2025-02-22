@testset "Speed profile helpers" begin
    using Tram2DKF: Stop, Accelerate, ConstantSpeed, activate, drive, TrajectoryDrive, distance, duration, end_state

    @testset "Stop" begin
        segment = activate(Stop(10.0), 0.0, 0.0, 0.0, 0.0)
        @test drive(segment, 0.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 0.0, accel = 0.0, jerk = 0.0)
        @test drive(segment, 5.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 0.0, accel = 0.0, jerk = 0.0)
        @test drive(segment, 9.9, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 0.0, accel = 0.0, jerk = 0.0)
        @test drive(segment, 10.0, 0.0, 0.0, 0.0) === nothing
        @test drive(segment, 20.0, 0.0, 0.0, 0.0) === nothing
        @test duration(segment) == 10.0
        @test distance(segment) == 0.0
        @test end_state(segment) == TrajectoryDrive(speed = 0.0, accel = 0.0, jerk = 0.0)
    end
    @testset "Acceleration" begin
        segment = activate(Accelerate(10.0, 1.0), 0.0, 0.0, 0.0, 0.0)
        @test drive(segment, 0.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 0.0, accel = 1.0, jerk = 0.0)
        @test drive(segment, 5.0, 12.5, 5.0, 1.0) == TrajectoryDrive(speed = 5.0, accel = 1.0, jerk = 0.0)
        @test drive(segment, 10.0, 50.0, 10.0, 1.0) === nothing
        @test drive(segment, 20.0, 100.0, 10.0, 0.0) === nothing
        @test duration(segment) == 10.0
        @test distance(segment) == 50.0
        @test end_state(segment) == TrajectoryDrive(speed = 10.0, accel = 0.0, jerk = 0.0)
    end
    @testset "Smooth acceleration" begin
        segment = activate(SmoothlyAccelerate(10.0, 1.0, 1.0), 0.0, 0.0, 0.0, 0.0)
        @test drive(segment, 0.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 0.0, accel = 0.0, jerk = 1.0)
        @test drive(segment, 1.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 0.5, accel = 1.0, jerk = 0.0)
        @test drive(segment, 2.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 1.5, accel = 1.0, jerk = 0.0)
        @test drive(segment, 9.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 8.5, accel = 1.0, jerk = 0.0)
        @test drive(segment, 10.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 9.5, accel = 1.0, jerk = -1.0)
        @test drive(segment, 11.0, 0.0, 0.0, 0.0) === nothing
        @test duration(segment) == 11.0
        @test distance(segment) == 55.0
        @test end_state(segment) == TrajectoryDrive(speed = 10.0, accel = 0.0, jerk = 0.0)
    end
    @testset "Constant speed" begin
        segment = activate(ConstantSpeed(10.0, 100.0), 0.0, 0.0, 0.0, 0.0)
        @test drive(segment, 0.0, 0.0, 10.0, 0.0) == TrajectoryDrive(speed = 10.0, accel = 0.0, jerk = 0.0)
        @test drive(segment, 5.0, 50.0, 10.0, 0.0) == TrajectoryDrive(speed = 10.0, accel = 0.0, jerk = 0.0)
        @test drive(segment, 9.9, 99.0, 10.0, 0.0) == TrajectoryDrive(speed = 10.0, accel = 0.0, jerk = 0.0)
        @test drive(segment, 10, 100.0, 10.0, 0.0) === nothing
        @test duration(segment) == 10.0
        @test distance(segment) == 100.0
        @test end_state(segment) == TrajectoryDrive(speed = 10.0, accel = 0.0, jerk = 0.0)
    end
    @testset "Deceleration" begin
        segment = activate(Accelerate(0.0, -1.0), 0.0, 0.0, 10.0, 0.0)
        @test drive(segment, 0.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 10.0, accel = -1.0, jerk = 0.0)
        @test drive(segment, 5.0, 37.5, 5.0, -1.0) == TrajectoryDrive(speed = 5.0, accel = -1.0, jerk = 0.0)
        @test drive(segment, 10.0, 50.0, 0.0, -1.0) === nothing
        @test drive(segment, 20.0, 0.0, 0.0, 0.0) === nothing
        @test duration(segment) == 10.0
        @test distance(segment) == 50.0
        @test end_state(segment) == TrajectoryDrive(speed = 0.0, accel = 0.0, jerk = 0.0)
    end
    @testset "Smooth deceleration" begin
        segment = activate(SmoothlyAccelerate(0.0, 1.0, 1.0), 0.0, 0.0, 10.0, 0.0)
        @test drive(segment, 0.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 10.0, accel = 0.0, jerk = -1.0)
        @test drive(segment, 1.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 9.5, accel = -1.0, jerk = 0.0)
        @test drive(segment, 2.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 8.5, accel = -1.0, jerk = 0.0)
        @test drive(segment, 9.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 1.5, accel = -1.0, jerk = 0.0)
        @test drive(segment, 10.0, 0.0, 0.0, 0.0) == TrajectoryDrive(speed = 0.5, accel = -1.0, jerk = +1.0)
        @test drive(segment, 11.0, 0.0, 0.0, 0.0) === nothing
        @test duration(segment) == 11.0
        @test distance(segment) == 55.0
        @test end_state(segment) == TrajectoryDrive(speed = 0.0, accel = 0.0, jerk = 0.0)
    end
end
