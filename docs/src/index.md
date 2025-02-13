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
the kind of the system: either `DiscreteTime` or `ContinuousTime`.

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
Alternatively, you may directly define a struct subclassing `StateEquation{DiscreteTime}`.

Linear models are represented using [`LTIStateEquation`](@ref) and [`LTIMeasurementEquation`](@ref)
types. These structs directly accept the state-space matrices as their fields.

You may also [`linearize`](@ref) a nonlinear model in some operating point
into a linear model.

## Kalman filtering



## Tram trajectories


## Demo scripts
