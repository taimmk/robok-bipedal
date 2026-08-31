# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for Robok robot."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##

ROBOK_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"E:/Project/ISAACSIM/robot/robok/urdf/robok_3/robok.usda",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,  # Set to False initially to prevent self-collision glitches
            solver_position_iteration_count=4, 
            solver_velocity_iteration_count=0
        ),
        joint_drive_props=sim_utils.JointDrivePropertiesCfg(ensure_drives_exist=True),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.55),  # Spawns base link slightly above ground plane (0.4m)
        joint_pos={
            # Left leg joints
            "hipyawl": 0.0,
            "hiprolll": 0.0,
            "hippitchl": 0.0,
            "anklepitchl": 0.0,
            "footl": 0.0,
            # Right leg joints
            "hipyawr": 0.0,
            "hiprollr": 0.0,
            "hippitchr": 0.0,
            "anklepitchr": 0.0,
            "footr": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.6,
    actuators={
        "all_legs": ImplicitActuatorCfg(
            joint_names_expr=[".*"],  # Regex pattern matching all 10 joints on robok
            effort_limit_sim=10.0,    # Max torque limit matching small servo actuators (10 N·m)
            stiffness={
                ".*": 20.0,           # Proportional Gain (Kp) for PD joint drives
            },
            damping={
                ".*": 5.0,            # Derivative Gain (Kd) for PD joint drives
            },
        ),
    },
)