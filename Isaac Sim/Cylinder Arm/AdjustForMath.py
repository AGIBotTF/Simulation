from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
import math
from omni.isaac.core import World
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# asset_path = "C:/Users/krasi/OneDrive/Documents/TONI_AI_NN_training/isaac/ArmTuned.usd"
# asset_path = "C:/Users/krasi/OneDrive/Documents/TONI_AI_NN_training/isaac/ignored_mimic_upside_down.usd"
asset_path = "C:/Users/krasi/OneDrive/Documents/TONI_AI_NN_training/isaac/EndEff.usd"

add_reference_to_stage(usd_path=asset_path, prim_path="/World/ArmTuned")


gripper = ParallelGripper(
    end_effector_prim_path="/World/ArmTuned/my_robot_arm/end_effector",
    joint_prim_names=["base_to_left_fingertip", "base_to_right_fingertip"],
    joint_opened_positions=np.array([0, 0]),
    joint_closed_positions=np.array([0.3, -0.3]),
    action_deltas=np.array([-0.3, 0.3]),
)
robot_arm = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/ArmTuned",
        name="ArmTuned",
        end_effector_prim_path="/World/ArmTuned/my_robot_arm/end_effector",

                                                gripper=gripper,
                                                position=np.array([0.0, 0.0, 0])
                                                ))

num_joints = 7
joints_default_positions = np.zeros(num_joints)
robot_arm.set_joints_default_state(positions=joints_default_positions)

my_world.reset()

i = 0
g = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        

        i += 0.01

        gripper_positions = robot_arm.gripper.get_joint_positions()

        new_positions = np.copy(joints_default_positions)
        for joint in range(6):
            new_positions[joint] = i % (math.pi*2)

        set_positions = np.copy(new_positions)
        set_positions[4] = gripper_positions[0]
        set_positions[5] = gripper_positions[1]
        robot_arm.set_joint_positions(set_positions)
        # robot_arm.apply_action(ArticulationAction(joint_positions=new_positions))

        g += 1
        if g < 300:
            robot_arm.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] + 0.1, gripper_positions[1] - 0.1]))
        if g > 300:
            robot_arm.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] - 0.1, gripper_positions[1] + 0.1]))
        if g == 600:
            g = 0
        
        
        
        
simulation_app.close()




