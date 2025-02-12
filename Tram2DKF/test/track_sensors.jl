@testset "Sensor simulation" begin
    using LinearAlgebra

    @testset "GNSS" begin
        #        t    s    x    y     v    a    φ    c    dc
        state = [0.0, 0.0, 5.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        altitude = 200.0
        cov_matrix = I(3)
        sample = simulate_gnss([state], altitude, cov_matrix)[1]

        # zero noise is not possible because simulate_gnss() runs Cholesky factorization inside
        @test  -1.0 <= sample[1] <=  11.0 # ± 6 sigma
        @test  19.0 <= sample[2] <=  31.0 # ± 6 sigma
        @test 194.0 <= sample[3] <= 206.0 # ± 6 sigma
    end
    @testset "Accelerometer" begin
        #        t    s    x    y    v    a    φ    c    dc
        state = [0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 0.0]

        acc = simulate_accelerometer([state], 0.0)[1]
        @test acc[1] == 1.0 # acceleration
        @test acc[2] == 12.0 # velocity^2 * curvature
        @test 9 <= acc[3] <= 10
    end
    @testset "Gyroscope" begin
        #        t    s    x    y    v    a    φ    c    dc
        state = [0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0, 3.0, 0.0]

        gyro = simulate_gyro([state], 0.0)[1]
        @test gyro[1] == 0.0
        @test gyro[2] == 0.0
        @test gyro[3] == 6.0 # velocity * curvature
    end
    @testset "Odometry" begin
        #        t    s     x    y     v    a    φ    c    dc
        state = [0.0, 10.4, 5.0, 25.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        odo = simulate_odometry([state], 1.0)[1]
        @test odo == 10.0
    end
end
