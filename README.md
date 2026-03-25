# Simplified Orbit and Attitude Simulator

A MATLAB/Simulink-based simulator for spacecraft orbit propagation, attitude determination, and control. Built around a 6DOF model of a rigid body in space (a little 1U CubeSat).

## Overview

The simulator models a 1U CubeSat in low Earth orbit with the following capabilities:

- **Orbital Dynamics**: Spherical gravity + J2 perturbation
- **Attitude Dynamics**: Rigid body dynamics with quaternion kinematics
- **Attitude Determination**: MEKF (gyro + magnetometer + sun sensor + star tracker), TRIAD, QUEST
- **Control**: Quaternion-feedback PD controller, B-dot detumbling
- **Trajectory Generation**: Inertial pointing, nadir pointing, target tracking
- **Actuators**: Reaction wheels (DC motor model with PI speed loop), magnetorquers
- **Sensors**: Gyroscope, magnetometer, sun sensor, star tracker
- **Environment**: NRLMSISE-00 (atmosphere), IGRF-14 (geomagnetic field), IAU-76 (ECI ↔ LLA), cylindrical eclipse model
- **Perturbations**: Gravity gradient, aerodynamic drag, solar radiation pressure, residual magnetic dipole

Default scenario: 1U CubeSat in a 500 km circular sun-synchronous orbit (i ≈ 98.4°) with a ground target at São Paulo.

## Requirements

- MATLAB R2025a or later
- Simulink
- Aerospace Toolbox

## Usage

```matlab
run main.m
```

`main.m` executes the setup scripts (`constants` → `scenario` → `params` → `ic`) and runs the Simulink model.

## Structure (outdated)

| File / Folder | Description |
|---|---|
| `main.m` | Entry point |
| `constants.m` | Physical and Earth parameters (WGS-84, EGM96 harmonics) |
| `scenario.m` | Orbit definition (KOE), ground target, simulation epoch |
| `params.m` | Satellite, actuator, sensor, and controller parameters |
| `ic.m` | Initial conditions (attitude quaternion, orbital state) |
| `model.slx` | Simulink model |
| `utils/` | Shared functions (coordinate transforms, quaternion math, time) |
| `fsw/` | Flight software (estimation, control, guidance) |

## To Do

- Fix sun pointing mode
- Documentation
- Update readme

## Notes

Work in progress, and mostly personal/academic/educational. Hardware parameters are sourced from component datasheets where available (see comments in `params.m`).
