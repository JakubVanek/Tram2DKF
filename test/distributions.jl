using LinearAlgebra

@testset "Standard Gaussian" begin
    g = Gaussian([0.0], [1.0;;])

    @test mean(g) == [0.0]
    @test covariance(g) == [1.0;;]
    @test pdf(g, [1.0])    ≈ 1/sqrt(2*pi) * exp(-1/2)
    @test logpdf(g, [1.0]) ≈ -1/2*log(2*pi) - 1/2
end

@testset "Square root Gaussian" begin
    g = SqrtGaussian([0.0], LowerTriangular([1.0;;]))

    @test mean(g) == [0.0]
    @test covariance(g) == [1.0;;]
    @test pdf(g, [1.0])    ≈ 1/sqrt(2*pi) * exp(-1/2)
    @test logpdf(g, [1.0]) ≈ -1/2*log(2*pi) - 1/2
end
