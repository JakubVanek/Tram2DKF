# Tram2DKF

[![Build Status](https://github.com/JakubVanek/Tram2DKF/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/JakubVanek/Tram2DKF/actions/workflows/CI.yml?query=branch%3Amain)
[![Documentation](https://img.shields.io/badge/docs-dev-blue.svg)](https://jakubvanek.github.io/Tram2DKF/)

A set of utilities for testing tram localization algorithms.

The package can be roughly split in three parts:

* Framework for defining linear and nonlinear state space models
* Implementation of Kalman filters:
  * Linear Kalman filter
  * Extended Kalman filter
  * Iterated Extended Kalman filter
* Generator of tram trajectories based on simple track geometry description

More information can be found in the [online documentation](https://jakubvanek.github.io/Tram2DKF).

To run some examples:
1. Create a new environment and enter it:
```bash
$ cd <some folder>
$ julia --project=.
```

2. Install example file requirements from within Julia:
```julia
] activate
] add https://github.com/JakubVanek/Tram2DKF
] add Plots
```
3. Download an example script (e.g. [plot_curved_track.jl](https://raw.githubusercontent.com/JakubVanek/Tram2DKF/refs/heads/main/scripts/plot_curved_track.jl)) to your computer
4. Run it:
```bash
$ julia --project=. plot_curved_track.jl
```
5. Open the newly generated `curved_track.png` file.
