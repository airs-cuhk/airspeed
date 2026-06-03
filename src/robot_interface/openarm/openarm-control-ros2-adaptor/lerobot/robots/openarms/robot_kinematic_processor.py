#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Kinematic processor steps for bimanual OpenArms robot.

Provides FK/IK conversions for both left and right arms.
"""

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from lerobot.configs.types import FeatureType, PipelineFeatureType, PolicyFeature
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import (
    ProcessorStep,
    ProcessorStepRegistry,
    RobotAction,
    RobotActionProcessorStep,
    TransitionKey,
)
from lerobot.utils.rotation import Rotation


def compute_bimanual_fk(
    joints: dict[str, Any],
    left_kinematics: RobotKinematics,
    right_kinematics: RobotKinematics,
    motor_names: list[str],
) -> dict[str, Any]:
    """Compute FK for both arms, converting joint positions to EE poses."""
    result = {}
    
    for prefix, kinematics in [("left", left_kinematics), ("right", right_kinematics)]:
        motor_joint_values = []
        for name in motor_names:
            key = f"{prefix}_{name}.pos"
            if key in joints:
                motor_joint_values.append(joints[key])
        
        if not motor_joint_values:
            continue
            
        q = np.array(motor_joint_values, dtype=float)
        t = kinematics.forward_kinematics(q)
        pos = t[:3, 3]
        tw = Rotation.from_matrix(t[:3, :3]).as_rotvec()
        
        gripper_key = f"{prefix}_gripper.pos"
        gripper_pos = joints.get(gripper_key, 0.0)
        
        result[f"{prefix}_ee.x"] = float(pos[0])
        result[f"{prefix}_ee.y"] = float(pos[1])
        result[f"{prefix}_ee.z"] = float(pos[2])
        result[f"{prefix}_ee.wx"] = float(tw[0])
        result[f"{prefix}_ee.wy"] = float(tw[1])
        result[f"{prefix}_ee.wz"] = float(tw[2])
        result[f"{prefix}_ee.gripper_pos"] = float(gripper_pos)
    
    return result


@ProcessorStepRegistry.register("bimanual_forward_kinematics_joints_to_ee")
@dataclass
class BimanualForwardKinematicsJointsToEE(RobotActionProcessorStep):
    """
    Converts joint positions to end-effector poses for both arms using FK.
    
    Input action keys: {prefix}_{motor}.pos (e.g., "right_joint_1.pos", "left_joint_2.pos")
    Output action keys: {prefix}_ee.{x,y,z,wx,wy,wz,gripper_pos}
    """
    left_kinematics: RobotKinematics
    right_kinematics: RobotKinematics
    motor_names: list[str]  # e.g., ["joint_1", "joint_2", ..., "joint_7", "gripper"]

    def action(self, action: RobotAction) -> RobotAction:
        ee_poses = compute_bimanual_fk(
            action, self.left_kinematics, self.right_kinematics, self.motor_names
        )
        
        # Remove joint positions, keep EE poses
        for prefix in ["left", "right"]:
            for name in self.motor_names:
                action.pop(f"{prefix}_{name}.pos", None)
        
        action.update(ee_poses)
        return action

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        for prefix in ["left", "right"]:
            for name in self.motor_names:
                features[PipelineFeatureType.ACTION].pop(f"{prefix}_{name}.pos", None)
            for k in ["x", "y", "z", "wx", "wy", "wz", "gripper_pos"]:
                features[PipelineFeatureType.ACTION][f"{prefix}_ee.{k}"] = PolicyFeature(
                    type=FeatureType.ACTION, shape=(1,)
                )
        return features


@ProcessorStepRegistry.register("bimanual_inverse_kinematics_ee_to_joints")
@dataclass
class BimanualInverseKinematicsEEToJoints(RobotActionProcessorStep):
    """
    Converts EE poses to joint positions for both arms using IK.
    
    Input action keys: {prefix}_ee.{x,y,z,wx,wy,wz,gripper_pos}
    Output action keys: {prefix}_{motor}.pos
    """
    left_kinematics: RobotKinematics
    right_kinematics: RobotKinematics
    motor_names: list[str]
    initial_guess_current_joints: bool = True
    
    q_curr_left: np.ndarray | None = field(default=None, init=False, repr=False)
    q_curr_right: np.ndarray | None = field(default=None, init=False, repr=False)

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION)
        if observation is None:
            raise ValueError("Observation required for IK")
        observation = observation.copy()
        
        for prefix, kinematics in [("left", self.left_kinematics), ("right", self.right_kinematics)]:
            x = action.pop(f"{prefix}_ee.x", None)
            y = action.pop(f"{prefix}_ee.y", None)
            z = action.pop(f"{prefix}_ee.z", None)
            wx = action.pop(f"{prefix}_ee.wx", None)
            wy = action.pop(f"{prefix}_ee.wy", None)
            wz = action.pop(f"{prefix}_ee.wz", None)
            gripper_pos = action.pop(f"{prefix}_ee.gripper_pos", None)
            
            if None in (x, y, z, wx, wy, wz, gripper_pos):
                continue
            
            # Get current joint positions from observation (in degrees from real robot)
            # NOTE: Observations from OpenArmsFollower are in degrees, not radians
            q_raw_deg = np.array([
                float(observation.get(f"{prefix}_{name}.pos", 0.0))
                for name in self.motor_names if name != "gripper"
            ], dtype=float)
            
            # q_raw_deg is already in degrees (no conversion needed)
            
            q_curr_attr = f"q_curr_{prefix}"
            if self.initial_guess_current_joints:
                # Store in degrees for IK solver
                setattr(self, q_curr_attr, q_raw_deg)
            else:
                if getattr(self, q_curr_attr) is None:
                    setattr(self, q_curr_attr, q_raw_deg)
            
            q_initial_deg = getattr(self, q_curr_attr)
            
            t_des = np.eye(4, dtype=float)
            t_des[:3, :3] = Rotation.from_rotvec([wx, wy, wz]).as_matrix()
            t_des[:3, 3] = [x, y, z]
            
            # Debug: print target pose and initial guess
            import logging
            logger = logging.getLogger(__name__)
            logger.debug(
                f"[{prefix} arm IK] Target EE: pos=({x:.3f}, {y:.3f}, {z:.3f}), "
                f"rotvec=({wx:.3f}, {wy:.3f}, {wz:.3f}), "
                f"initial guess (deg)={q_initial_deg}"
            )
            
            # IK solver expects degrees, returns degrees
            q_target_deg = kinematics.inverse_kinematics(q_initial_deg, t_des)
            
            # Verify IK solution by forward kinematics
            t_actual = kinematics.forward_kinematics(q_target_deg)
            pos_actual = t_actual[:3, 3]
            pos_error = np.linalg.norm(pos_actual - np.array([x, y, z]))
            
            if pos_error > 0.01:  # 1cm tolerance
                logger.warning(
                    f"[{prefix} arm IK] Large position error: {pos_error:.4f}m. "
                    f"Target=({x:.3f}, {y:.3f}, {z:.3f}), "
                    f"Actual=({pos_actual[0]:.3f}, {pos_actual[1]:.3f}, {pos_actual[2]:.3f})"
                )
            
            # Store result in degrees for next iteration
            setattr(self, q_curr_attr, q_target_deg)
            
            # Convert back to radians for action (motor commands are in radians)
            q_target_rad = np.deg2rad(q_target_deg)
            
            for i, name in enumerate(self.motor_names):
                if name != "gripper":
                    action[f"{prefix}_{name}.pos"] = float(q_target_rad[i])
                else:
                    action[f"{prefix}_gripper.pos"] = float(gripper_pos)
        
        return action

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        for prefix in ["left", "right"]:
            for k in ["x", "y", "z", "wx", "wy", "wz", "gripper_pos"]:
                features[PipelineFeatureType.ACTION].pop(f"{prefix}_ee.{k}", None)
            for name in self.motor_names:
                features[PipelineFeatureType.ACTION][f"{prefix}_{name}.pos"] = PolicyFeature(
                    type=FeatureType.ACTION, shape=(1,)
                )
        return features

    def reset(self):
        self.q_curr_left = None
        self.q_curr_right = None


@ProcessorStepRegistry.register("bimanual_ee_bounds_and_safety")
@dataclass
class BimanualEEBoundsAndSafety(RobotActionProcessorStep):
    """
    Clips EE poses to bounds and limits step size for both arms.
    """
    end_effector_bounds: dict  # {"min": [x,y,z], "max": [x,y,z]}
    max_ee_step_m: float = 0.05
    
    _last_pos_left: np.ndarray | None = field(default=None, init=False, repr=False)
    _last_pos_right: np.ndarray | None = field(default=None, init=False, repr=False)

    def action(self, action: RobotAction) -> RobotAction:
        import logging
        logger = logging.getLogger(__name__)
        
        for prefix in ["left", "right"]:
            x = action.get(f"{prefix}_ee.x")
            y = action.get(f"{prefix}_ee.y")
            z = action.get(f"{prefix}_ee.z")
            
            if None in (x, y, z):
                continue
            
            pos_original = np.array([x, y, z], dtype=float)
            pos = pos_original.copy()
            
            # Check bounds clipping
            pos_clipped = np.clip(pos, self.end_effector_bounds["min"], self.end_effector_bounds["max"])
            if not np.allclose(pos, pos_clipped):
                logger.warning(
                    f"[{prefix} arm] EE position clipped to bounds: "
                    f"original=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) -> "
                    f"clipped=({pos_clipped[0]:.3f}, {pos_clipped[1]:.3f}, {pos_clipped[2]:.3f})"
                )
            pos = pos_clipped
            
            last_pos_attr = f"_last_pos_{prefix}"
            last_pos = getattr(self, last_pos_attr)
            if last_pos is not None:
                dpos = pos - last_pos
                n = float(np.linalg.norm(dpos))
                if n > self.max_ee_step_m and n > 0:
                    pos_limited = last_pos + dpos * (self.max_ee_step_m / n)
                    logger.warning(
                        f"[{prefix} arm] EE step size limited: "
                        f"requested step={n:.3f}m > max={self.max_ee_step_m:.3f}m. "
                        f"Original target=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) -> "
                        f"Limited to=({pos_limited[0]:.3f}, {pos_limited[1]:.3f}, {pos_limited[2]:.3f})"
                    )
                    pos = pos_limited
            
            setattr(self, last_pos_attr, pos)
            
            action[f"{prefix}_ee.x"] = float(pos[0])
            action[f"{prefix}_ee.y"] = float(pos[1])
            action[f"{prefix}_ee.z"] = float(pos[2])
        
        return action

    def reset(self):
        self._last_pos_left = None
        self._last_pos_right = None

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        return features

