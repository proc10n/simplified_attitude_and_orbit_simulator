# Simplified Orbit and Attitude Simulator

**Disclaimer (26/03/26):** I think the sim is in a good enough spot at the moment. It still has some rough edges, some stuff I vibe-coded with Claude because it was too boring, some new stuff too add (new controller, new estimators, orbit determination, etc.) but nothing really seems to be all that valueable at the moment.

I initially made this sim simply because the one I used for university was tied to the CubeSat club I used to participate, so I wished to have something of my own (for projects, maybe papers eventually). They ended up being quite similar, though I think mine is more organized and runs better.

Thing is, I'm at a point where I'm bottlenecked by my own knowledge of what an AOCS sim even needs to have. I can, for example, add J3+ purturbations or third body effects, but these would be so miniscule for the time horizon I'm using my sim for that it's pointless to do so. Should I change the sun ephemeris block for a hand coded algorithm taken from a book such as Meeus or Vallado? Maybe. But I'm not sure it is worthwhile. Likewise, I could add some new controller to the sim, like LQR or a Sliding Mode Controller, but my understanding is that, even though they do see some use, the quaternion feedback "PD" law is more ubiquitous.

Also, I'm not sure wheter "huge simulink block diagram" is the way to go anymore. It's nice to show to other people, but I keep using more and more MATLAB Function blocks.

So I think I'll leave it like this for now. When I have a better idea of what to do next, I'll come back.

---

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
│   ├── nadir_pointing.m   Nadir pointing
│   ├── target_tracking.m  Ground target tracking
│   └── sun_pointing.m     Sun pointing
└── utils/
    ├── quat_mul.m, quat_err.m, quat2dcm.m, dcm2quat.m, skew_mat.m
    ├── R1.m, R3.m
    ├── koe2eci.m, ecef2eci.m
    └── greg2jd.m, gmst.m
```
## Examples

1. Ground Target Tracking

https://github.com/user-attachments/assets/17e2450d-faf1-47ba-bc4c-3cb7422f293d

2. Nadir Pointing

https://github.com/user-attachments/assets/ab5ae276-bb23-49c4-9e57-d8ea0db3ccd7

3. Sun Pointing

https://github.com/user-attachments/assets/fbc9a979-26e9-4426-9b4b-0f0f46871f7b

## Notes

Work in progress. Personal/academic project.
