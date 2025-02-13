module Tram2DKF

include("utils.jl")
include("models.jl")
include("kf_base.jl")
include("kf.jl")
include("ekf.jl")
include("linesearch.jl")
include("iekf.jl")
include("motion_models.jl")
include("track_speedprofile.jl")
include("track_curvature.jl")
include("track_generator.jl")
include("track_sensors.jl")

export DiscreteTime, ContinuousTime
export StateEquation, MeasurementEquation, CompositeMeasurement
export LTIStateEquation, LTIMeasurementEquation
export DiscretizationAlgorithm, RK4, Euler
export linearize, discretize
export nstates, ninputs, noutputs
export Gaussian, SqrtGaussian, mean, covariance, pdf, logpdf
export KalmanFilter, LinearKalmanFilter, ExtendedKalmanFilter, IteratedExtendedKalmanFilter
export forward_step, backward_step, data_step
export IdentityStepping, BacktrackingLineSearch
export TrackSegment, StraightTrack, TrackTurn
export SpeedProfileSegment, Stop, Accelerate, ConstantSpeed
export render_trip
export simulate_accelerometer, simulate_gyro, simulate_gnss, simulate_odometry

end
