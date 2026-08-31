# Motion Planning: LIPM Trajectory & Inverse Kinematics Pipeline

This directory contains the MATLAB workflow for generating balanced bipedal walking trajectories using the Linear Inverted Pendulum Model (LIPM), solving joint kinematics, and exporting trajectory matrices for Simulink.

---

## Workflow Overview

```text
[ applicationLIPM.mlapp ] ──> Tune & verify balance parameters
          │
          ▼
[ animateLIPM.m ] ──> Plot leg/COM trajectories ──> Generates: defaultfootinfos.mat
          │
          ▼
[ InverseKinematics_local.m ] ──> Calculate joint positions ──> Generates: IK trajectory data
          │
          ▼
[ Test_gen_mat.m ] ──> Format joint time-series ──> Generates: jointAngs_simulink_ready.mat
          │
          ▼
[ Simulink Model ] ──> Low-level robot execution
```

---

## Prerequisites

- **MATLAB** (R2023a or newer recommended)
- **Toolboxes:** Simulink, Simscape Multibody, Optimization Toolbox

---

## Step-by-Step Execution Guide

### 1. Balance Parameter Configuration

Launch the App Designer interface to configure and verify LIPM balance parameters:

```matlab
applicationLIPM
```

**Objective:** Adjust pendulum height ($z_0$), step time ($T_s$), and step length parameters to ensure zero-moment point (ZMP) stability for the physical robot specifications.

### 2. Trajectory Generation & Visualization

Apply the verified parameters to `animateLIPM.m` to generate and preview leg and Center of Mass (COM) trajectories:

```matlab
run('animateLIPM.m')
```

- **Output:** Displays animated robot walking gait and foot trajectory plots.
- **Generated Artifact:** `defaultfootinfos.mat` — stores target foot placements and time vectors.

### 3. Inverse Kinematics Computation

Compute the exact joint angles required to track the generated foot placements:

```matlab
run('InverseKinematics_local.m')
```

- **Input:** Automatically loads `defaultfootinfos.mat`.
- **Output:** Calculates leg joint positions across the stride cycle using the local-coordinate-frame IK solver.

### 4. Format Trajectories for Simulation

Convert raw joint trajectories into a time-series format optimized for Simulink import:

```matlab
run('Test_gen_mat.m')
```

- **Generated Artifact:** `jointAngs_simulink_ready.mat`

### 5. Load into Simulink

Move `jointAngs_simulink_ready.mat` to your main simulation workspace or load it before launching the Simulink environment:

```matlab
load('jointAngs_simulink_ready.mat');
```

---

## Data Artifacts Summary

| File Name | Producer Script | Description |
| --- | --- | --- |
| `defaultfootinfos.mat` | `animateLIPM.m` | Foot placement coordinates, step timing, and COM trajectories |
| `jointAngs_simulink_ready.mat` | `Test_gen_mat.m` | Time-series joint angle matrix formatted for Simulink block inputs |