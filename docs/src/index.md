```@meta
CurrentModule = Tram2DKF
```

# Tram2DKF

A set of utilities for testing tram localization algorithms.

The package can be roughly split in three parts:

* Framework for defining linear and nonlinear state space models
* Implementation of Kalman filters:
  * Linear Kalman filter
  * Extended Kalman filter
  * Iterated Extended Kalman filter
* Generator of tram trajectories based on simple track geometry description

## State space models

To estimate the state of something using Kalman filters, you need to have
a model of it. This package accepts linear and nonlinear explicit state-space models.

Nonlinear models are defined by creating a new structure that implements a
[`StateEquation{Time}`](@ref) abstract type. The `Time` parameter determines
the kind of the system: either [`DiscreteTime`](@ref) or [`ContinuousTime`](@ref).

You could define a continuous-time model this way:

```julia
using Tram2DKF
import Tram2DKF.ninputs, Tram2DKF.noutputs, Tram2DKF.nstates

# define new struct
struct FancyIntegrator <: StateEquation{ContinuousTime} end

# compute state derivative: just the input
(::FancyIntegrator)(x, u) = [u[1]]

# add marker functions
ninputs(::FancyIntegrator) = 1
nstates(::FancyIntegrator) = 1
```

You can then [`discretize`](@ref) such a model into a discrete-time model.
Alternatively, you may directly define a struct subclassing [`StateEquation{DiscreteTime}`](@ref).

Linear models are represented using [`LTIStateEquation`](@ref) and [`LTIMeasurementEquation`](@ref)
types. These structs directly accept the state-space matrices as their fields.

You may also [`linearize`](@ref) a nonlinear model in some operating point
into a linear model.

## Kalman filtering

Kalman filters, in a nutshell, allow you to fuse information from multiple sources.
This allows you to reduce the effect of measurement noise on your estimates.

The package currently implements a [`LinearKalmanFilter`](@ref),
an [`ExtendedKalmanFilter`](@ref) and
an [`IteratedExtendedKalmanFilter`](@ref).

The structs themselves carry only the parameters of the respective algorithms.
To invoke the filters, you can use:
* the [`forward_step`](@ref) function for the time update step (= prediction of new state),
* the [`data_step`](@ref) function for the data update step (= fusion of sensor measurements),
* the [`backward_step`](@ref) function for a backward smoothing step (= smoothing of old states based on new measurements).

These filters assume that measurements and states are Gaussian random variables.
To represent such variables, the [`Gaussian`](@ref) and [`SqrtGaussian`](@ref)
types is used. The latter is used to implement more numerically stable updates.
Additionally, there is a [`UncertainValue`](@ref) abstract type that unifies the
two representations behind two methods: [`mean`](@ref) and [`covariance`](@ref).

## Tram trajectories

The ultimate goal of this package is to be useful in developing tram
localization algorithms. To compare different algorithms, it is necessary
to obtain some dataset on which they could be tested. This is somewhat
nontrivial.

This package allows you to generate synthetic data based on a simple description.
The description is composed of two parts:
* Description of the tram track. This basically determines where the track turns
  straight track segments are, and what are their parameters.
* Description of the tram speed profile. This determines when the tram accelerates
  or decelerates.

This is illustrated by a simple example:

```julia
using Tram2DKF
using LinearAlgebra

# Define track profile
track = [
    StraightTrack(distance=100.0),
    TrackTurn(angle=deg2rad(+90), radius=20.0, transition_curve_length=0.0),
    StraightTrack(distance=100.0),
]

# Define speed profile
trip = [
    Stop(duration=1.0),
    Accelerate(to_speed=10.0, acceleration=1.0),
    ConstantSpeed(speed=10.0, distance=100.0),
    Accelerate(to_speed=0.0, acceleration=1.0),
    Stop(duration=10.0),
]

# Generate ground truth data
states = render_trip(track, trip, Ts, 10)
# ^ `states` is a vector of state vectors at different times

# Simulate sensor measurements corrupted by noise
gyro = simulate_gyro(states, 0.02)
acc  = simulate_accelerometer(states, 0.1)
odo  = simulate_odometry(states, 0.01)
gnss = simulate_gnss(states, 200, I(3))
```

## Demo scripts

There are three example scripts available:
* `scripts/plot_curved_track.jl` shows how to generate samples of the track position without concern for any tram dynamics.
* `scripts/plot_speed_profile.jl` shows how to generate trajectories of trams when considering changes in speed.
* `scripts/estimation_test.jl` is an all-in-one example:
  * first, the test data is generated
  * then, a model is defined (or, rather, a predefined [`PolarCTRA`](@ref) model is reused)
  * finally, a Kalman filter is run to try to undo the effect of noise in measurements
