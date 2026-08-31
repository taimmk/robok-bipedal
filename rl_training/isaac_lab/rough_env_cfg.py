# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as TermCfg  # <-- تم إضافة هذا السطر
from isaaclab.utils.configclass import configclass
from isaaclab_physx.sensors import ContactSensorCfg as PhysXContactSensorCfg

import isaaclab.envs.mdp as env_mdp  # <-- تم إضافة هذا السطر لاستدعاء دوال الارتفاع
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)
from isaaclab_tasks.utils import preset

##
# Pre-defined configs
##
# Ensure this import points to where you saved your ROBOK_CFG in the previous step
from .Robok import ROBOK_CFG  # isort: skip


@configclass
class RobokRewardsCfg(RewardsCfg):
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=1.5,
        params={
            # UPDATED: broadened body_names to ensure it matches the child collision node
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*foot.*"),
            "command_name": "base_velocity",
            "threshold": 0.35, 
        },
    )
    
    # FIX 2 FOR JUMPING: Heavily penalize vertical (Z-axis) velocity
    # ---------------------------------------------------------
    #lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    #ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    
    # 3. FORCE TORSO FLAT (Stops robot from leaning sideways over one leg)
    #flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-2.5)
    
    # 1. STRICT TORSO POSTURE CONSTRAINT (Prevents tilting and rolling)
    # flat_orientation_l2 = RewTerm(
    #     func=mdp.flat_orientation_l2, 
    #     weight=-5.0  # Massive penalty for leaning away from perfectly flat
    # )

   
    
    
    # 2. ANTI-WOBBLE CONSTRAINT (Prevents angular velocity in Pitch/Roll)
    # ang_vel_xy_l2 = RewTerm(
    #     func=mdp.ang_vel_xy_l2, 
    #     weight=-0.5  # Punishes the torso for swaying side-to-side or front-to-back
    # )
    
    # 4. CRITICAL FIX: PENALIZE JOINT DEVIATION ON ALL JOINTS (Hips, Knees, Ankles)
    # This prevents the robot from tucking one leg up continuously
    joint_deviation_all_joints = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )
    # ---------------------------------------------------------
    
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["hipyaw.*", "hiproll.*"])},
    )
    joint_deviation_ankles = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["anklepitch.*"])},
    )
    # penalize ankle joint limits
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names="anklepitch.*")},
    )
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names="hiproll.*")},
    )

    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*foot.*"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*foot.*"),
        },
    )
    
    
    
    
    
    

@configclass
class RobokRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Robok rough environment configuration."""

    rewards: RobokRewardsCfg = RobokRewardsCfg()

    def __post_init__(self):
        super().__post_init__()

        
        self.scene.terrain.pos = (0.0, 0.0, -0.4)
        # biped yaw control is harder than quadruped — relax the per-episode-mean yaw
        # threshold to 0.8 rad/s (defaults work for quadrupeds).
        self.commands.base_velocity.vel_yaw_success_threshold = 0.8
        # scene
        self.scene.robot = ROBOK_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        
        # ADD THIS LINE: Force contact sensor to scan the entire robot tree for bodies
        #self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/Robot/.*"
        
        # CHANGED: Reverted to a single string to fix the AttributeError
        # self.scene.left_foot_contact = PhysXContactSensorCfg(
        #     prim_path="{ENV_REGEX_NS}/Robot/Geometry/base_link/.*_hipyaw/.*_hiproll/.*_hippitch/.*_anklepitch/.*_foot",
        #     history_length=3,
        #     track_air_time=True,
        #     update_period=self.sim.dt,
        # )
        
        #----------------------------
        # Command velocity ranges
        self.commands.base_velocity.ranges.lin_vel_x = (0.2, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        #----------------------------
        
        
        
        self.scene.contact_forces = PhysXContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/.*foot.*", 
            history_length=3,
            track_air_time=True,
            update_period=self.sim.dt,
        )

        # self.scene.right_foot_contact = PhysXContactSensorCfg(
        #     prim_path="{ENV_REGEX_NS}/Robot/Geometry/base_link/right_hipyaw/right_hiproll/right_hippitch/right_anklepitch/right_foot/right_foot",
        #     history_length=3,
        #     track_air_time=True,
        #     update_period=self.sim.dt,
        # )
        
        # self.scene.contact_forces = PhysXContactSensorCfg(
        #     prim_path="{ENV_REGEX_NS}/Robot/base_link/right_hipyaw/right_hiproll/right_hippitch/right_anklepitch/right_foot/right_foot", 
        #     history_length=3,
        #     track_air_time=True,
        #     update_period=self.sim.dt
        # )
        #self.rewards.feet_air_time = None        # Turns off air-time reward
        
        # Apply armature to the custom "all_legs" actuator group
        #self.scene.robot.actuators["all_legs"].armature = preset(default=0.0, newton_mjwarp=0.02)

    # Apply armature to the custom "all_legs" actuator group
        self.scene.robot.actuators["all_legs"].armature = 0.0

        # Height scanner follows the base_link
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot"

        # Add mass to base_link instead of pelvis
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_link"
        self.events.add_base_mass.params["mass_distribution_params"] = (1.0, 1.25)
        self.events.base_com = None
        self.events.base_external_force_torque.params["asset_cfg"].body_names = ".*base_link"
        
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)

        # actions
        self.actions.joint_pos.scale = 0.5

        # ---------------------------------------------------------
        # تم التعديل هنا: إيقاف حساس التلامس القديم واستخدام حساس الارتفاع
        # ---------------------------------------------------------
        self.terminations.base_contact = None  # تعطيل شرط التلامس القديم الذي كان يسبب الخطأ
        
        self.terminations.base_height = TermCfg(
            func=env_mdp.root_height_below_minimum,
            params={
                "minimum_height": 0.1,  # التوقف إذا قل ارتفاع المركز عن 0.1
                "asset_cfg": SceneEntityCfg("robot")
            }
        )
        
        
         # NEW: Hard constraint to reset if the torso tilts too far
        self.terminations.bad_orientation = TermCfg(
            func=env_mdp.bad_orientation,
            params={
                # Limit sets how far the projected gravity vector can deviate.
                # A value of 0.5 allows roughly a 30-degree tilt before resetting.
                # Lower the limit (e.g., 0.2) to make the constraint even stricter.
                "limit_angle": 0.5, 
                "asset_cfg": SceneEntityCfg("robot")
            }
        )
        # ---------------------------------------------------------

        # rewards
        self.rewards.undesired_contacts = None
        self.rewards.dof_torques_l2.weight = -5.0e-6
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 1.0
        self.rewards.action_rate_l2.weight = -0.05
        self.rewards.dof_acc_l2.weight = -1.0e-6


@configclass
class RobokRoughEnvCfg_PLAY(RobokRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (0.2, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (0, 0)
        self.commands.base_velocity.ranges.heading = (0, 0)
        # disable randomization for play
        self.observations.policy.enable_corruption = False