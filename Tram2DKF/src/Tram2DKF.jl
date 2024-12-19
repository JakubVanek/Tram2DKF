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

end
