# Quadplane Condor UAV Aircraft Model & Packaging Guide

This directory contains the complete simulation model assets, 3D CAD/visual geometries, PX4 Autopilot airframe mixer configurations, and Gazebo world environments for the **Quadplane Condor** Hybrid VTOL aircraft.

---

## 1. Technical Aircraft Specifications

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Aircraft Layout** | **4 + 1 Hybrid VTOL** | 4 vertical lift rotors + 1 forward puller/tractor motor |
| **Empennage** | **V-Tail (Ruddervators)** | Combined elevator pitch & rudder yaw control |
| **Wingspan ($b$)** | **2.40 m** | High-aspect-ratio rectangular fixed wing |
| **Mean Aerodynamic Chord ($\bar{c}$)** | **0.30 m** | Wing chord length |
| **Wing Area ($S$)** | **0.72 m²** | Total wing reference lifting area |
| **Maximum Takeoff Weight (MTOW)** | **7.80 kg** (76.5 N) | Full payload and avionics operational weight |
| **Cruise Speed ($V_{\text{cruise}}$)** | **18.0 m/s** (64.8 km/h) | Optimal fixed-wing cruise speed |
| **Transition Airspeed ($v_{\text{trans}}$)** | **15.0 m/s** (54.0 km/h) | Forward wing-borne transition threshold |
| **Endurance** | **3 – 5 hours** | Long-range aerial surveillance and mapping |

---

## 2. Directory Structure

```
models/
├── README.md                           # This document
├── airframes/
│   └── 4030_gz_quadplane_condor        # PX4 Autopilot airframe definition & controller parameters
├── quadplane_condor/                   # Gazebo Harmonic 3D SDF Model
│   ├── model.config                    # Gazebo model metadata
│   ├── model.sdf                       # Physics, aerodynamics, sensors, and actuator plugin definition
│   └── meshes/                         # 3D Visual & CAD Geometries
│       ├── condor_cad.stl              # High-fidelity CAD geometry (STL)
│       ├── quadplane_condor.dae        # Visual Collada 3D mesh
│       ├── iris_prop_cw.dae            # Clockwise rotor propeller
│       └── iris_prop_ccw.dae           # Counter-clockwise rotor propeller
└── worlds/
    └── condor_figure8.sdf              # 3D Airfield simulation world with Figure-8 flight gates & Helipad
```

---

## 3. Motor & Actuator Mapping

```
                 ▲ North (+X)
                 │
              [Motor 4] (Nose Tractor Puller)
                 │
   [Motor 2] (FL, CW)    [Motor 0] (FR, CCW)
            \            /
             \  [FUSE]  /
              \        /
   [Motor 1] (RL, CCW)   [Motor 3] (RR, CW)
                 │
            /---------\ (V-Tail Ruddervators: CS0 / CS1)
```

- **Motor 0 (Front-Right)**: $x = +0.4416\,\text{m}$, $y = +0.4236\,\text{m}$ (CCW)
- **Motor 1 (Rear-Left)**: $x = -0.4428\,\text{m}$, $y = -0.4192\,\text{m}$ (CCW)
- **Motor 2 (Front-Left)**: $x = +0.4416\,\text{m}$, $y = -0.4236\,\text{m}$ (CW)
- **Motor 3 (Rear-Right)**: $x = -0.4428\,\text{m}$, $y = +0.4192\,\text{m}$ (CW)
- **Motor 4 (Nose Puller)**: $x = +0.5256\,\text{m}$, Axis: $+X$ forward thrust
- **V-Tail Ruddervators**:
  - `CS0`: Left ruddervator ($\delta_e + \delta_r$)
  - `CS1`: Right ruddervator ($\delta_e - \delta_r$)

---

## 4. How to Install & Package for External Parties

### A. Automatic Installation on Target Machine
When deploying this repository onto another machine (with PX4-Autopilot and Gazebo installed):

```bash
# 1. Run the automated model installer & setup
./manage.sh setup

# 2. Launch the complete simulation stack
./manage.sh all
```

### B. Packaging the Project for Distribution
To package the complete self-contained project into a redistributable archive:

```bash
# Create a clean tar.gz package automatically
./manage.sh package
```
