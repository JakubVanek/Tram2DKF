module Tram2DKF

# Write your package code here.

include("models.jl")
include("kf_base.jl")
include("kf.jl")

export DiscreteTime, ContinuousTime
export StateEquation, MeasurementEquation
export LTIStateEquation, LTIMeasurementEquation
export DiscretizationAlgorithm, RK4, Euler
export linearize, discretize
export nstates, ninputs, noutputs
export Gaussian, SqrtGaussian, mean, covariance
export KalmanFilter, LinearKalmanFilter
export forward_step, backward_step, data_step

end
