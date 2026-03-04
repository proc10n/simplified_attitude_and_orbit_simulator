# Simplified Orbit and Attitude Simulator

A MATLAB/Simulink-based simulator for spacecraft orbit propagation and attitude dynamics. Built around a 6DOF model with coupled orbital and attitude subsystems.

## Overview

The simulator models a satellite in low Earth orbit with the following capabilities:

- **Orbital Dynamics**: Spherical gravity + J2 perturbation, Keplerian elements to ECI conversion
- **Attitude Dynamics**: Rigid body dynamics with quaternion kinematics
- **Control**: Quaternion-feedback PD controller
- **Trajectory Generation**: Inertial pointing, nadir pointing, target tracking
- **Environmental Models**: NRLMSISE-00 (atmosphere), IGRF-14 (geomagnetic field), IAU-76 (ECI ↔ LLA)
- **Estimation** *(implemented, not yet integrated)*: TRIAD, QUEST (static), MEKF (dynamic)
- **Sensors**: Basic gyroscope model (bias + noise)

Default scenario: 500 km circular, sun-synchronous orbit (i ≈ 98.4°) with a ground target at São Paulo.

## Requirements

- MATLAB (R2023b or later recommended)
- Simulink
- Aerospace Toolbox *(if using built-in environment blocks)*

## Usage

```matlab
run main.m
```

`main.m` executes the setup scripts in order (`constants` → `params` → `scenario` → `ic`) and then runs the Simulink model.

## Structure

| File / Folder | Description |
|---|---|
| `main.m` | Entry point |
| `constants.m` | Earth parameters (WGS-84, EGM96 harmonics) |
| `params.m` | Satellite inertia, actuator parameters, controller gains |
| `scenario.m` | Orbit definition (KOE), ground target, epoch |
| `ic.m` | Initial conditions (attitude quaternion, orbital state in ECI) |
| `model.slx` | Simulink model |
| `utils/` | Auxiliary functions (time, coordinate transforms, GMST, etc.) |

## To Do

- Magnetorquer model
- Additional perturbations (drag, SRP, gravity gradient torque)
- Sensor models (sun sensor, magnetometer, star tracker)
- Integration of estimation algorithms into the control loop

## Notes

This is a work-in-progress, personal/academic project. Parameters are mostly placeholder values.
