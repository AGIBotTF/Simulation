from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.tasks as tasks
from typing import Optional
import numpy as np


# Inheriting from the base class Follow Target
class FollowTarget(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "Arm_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        #TODO: change this to the robot usd file.
        # asset_path = "C:/Users/krasi/OneDrive/Documents/TONI_AI_NN_training/isaac/ArmTuned.usd"
        asset_path = "C:/Users/krasi/OneDrive/Documents/TONI_AI_NN_training/isaac/ignored_mimic_upside_down.usd"
        # asset_path = "C:/IsaacSim/MyScripts/Cubic.usd"
        # asset_path = "/home/skyentific/Documents/YellowArm/URDF/yellowarm_description/robots/yellow_arm/yellow_arm.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/ArmTuned")
        # add_reference_to_stage(usd_path=asset_path, prim_path="/World/yellowarm")
        #define the gripper
        # gripper = ParallelGripper(
        #     #We chose the following values while inspecting the articulation
        #     end_effector_prim_path="/World/yellowarm/yellowarm_link7",
        #     joint_prim_names=["yellowarm_joint8", "yellowarm_joint9"],
        #     joint_opened_positions=np.array([0.022, 0.022]),
        #     joint_closed_positions=np.array([-0.015, -0.015]),
        #     action_deltas=np.array([0.037, 0.037]),
        # )
        gripper = ParallelGripper(
            end_effector_prim_path="/World/ArmTuned/my_robot_arm/part6",
            joint_prim_names=["base_to_left_fingertip", "base_to_right_fingertip"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([0.3, -0.3]),
            action_deltas=np.array([-0.3, 0.3]),
        )
        #define the manipulator
        # manipulator = SingleManipulator(prim_path="/World/yellowarm", name="yellowarm_robot",
        #                                                 end_effector_prim_path="/World/yellowarm/yellowarm_link7",
        #                                                 gripper=gripper)
        
        manipulator = SingleManipulator(prim_path="/World/ArmTuned", name="ArmTuned",
                                                end_effector_prim_path="/World/ArmTuned/my_robot_arm/part6",
                                                gripper=gripper)
        #set the default positions of the other gripper joints to be opened so
        #that its out of the way of the joints we want to control when gripping an object for instance.
        # joints_default_positions = np.zeros(9)
        # joints_default_positions[7] = -0.022
        # joints_default_positions[8] =  0.022

        joints_default_positions = np.zeros(8)
        joints_default_positions[6] =  0
        joints_default_positions[7] =  0

        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator