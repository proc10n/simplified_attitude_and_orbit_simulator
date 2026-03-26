# SOAD — Simplified Orbit and Attitude Simulator

MATLAB/Simulink 6DOF simulation of a 1U CubeSat ADCS. Models the full loop from orbit propagation through sensors, estimation, control, and actuation.

Default scenario: 500 km sun-synchronous orbit, ground target at São Paulo.

## What's in it

- **Orbit:** Two-body + J2
- **Attitude dynamics:** Rigid body + quaternion kinematics. Perturbation torques: gravity gradient, aero drag, SRP, residual magnetic dipole
- **Environment:** IGRF-14, NRLMSISE-00, IAU-76 frame conversions, cylindrical eclipse, solar ephemeris
- **Sensors:** Gyro (ADXRS453), magnetometer (PNI RM3100), sun sensor (NSS AQUILA-D02), star tracker (arcsec Sagitta) — all parameterized from datasheets
- **Estimation:** MEKF (gyro + mag + sun + star tracker), TRIAD, QUEST
- **Trajectory generation:** Inertial, nadir, sun pointing, ground target tracking
- **Control:** Quaternion-feedback PD, B-dot detumbling
- **Actuators:** Reaction wheels (DC motor model, Portescap 20ECF14, PI speed loop) + magnetorquers (NSS NCTR-M003)
- **Visualization:** VTS export (CIC OEM/AEM)

## Requirements

MATLAB R2025a+, Simulink, Aerospace Toolbox

## Usage

```matlab
run main.m
```

## Structure

```
soad/
├── main.m                 Entry point
├── export_vts.m           CIC OEM/AEM export for VTS
├── model.slx              Simulink model
├── config/
│   ├── load_constants.m   Physical constants (WGS-84, EGM96)
│   ├── load_scenario.m    Epoch, orbit elements, ground target
│   ├── load_hardware.m    Satellite body, sensors, actuators
│   ├── load_fsw.m         Controller gains, MEKF config, sim settings
│   └── load_ic.m          Initial conditions from scenario
├── fsw/
│   ├── mekf.m             Multiplicative Extended Kalman Filter
│   ├── triad.m            TRIAD determination
│   ├── nadir_pointing.m   Nadir (LVLH) guidance
│   ├── target_tracking.m  Ground target tracking guidance
│   └── sun_pointing.m     Sun pointing (eigen-axis)
└── utils/
    ├── quat_mul.m, quat_err.m, quat2dcm.m, dcm2quat.m, skew_mat.m
    ├── R1.m, R3.m
    ├── koe2eci.m, ecef2eci.m
    └── greg2jd.m, gmst.m
```

Config files are functions returning structs with explicit dependencies. Hardware params come from datasheets; FSW params are tuning knobs. See comments in each file for sources and units.


## Notes

Work in progress — personal/academic project.
