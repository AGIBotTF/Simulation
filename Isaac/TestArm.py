import os
import psutil

p = psutil.Process(os.getpid())
# p.cpu_affinity([0, 1, 2, 3, 4, 5])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

my_world = World(stage_units_in_meters=1.0)
asset_path = "C:/Users/krasi/OneDrive/Documents/TONI_AI_NN_training/isaac/ArmTuned.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/ArmTuned")

gripper = ParallelGripper(
    end_effector_prim_path="/World/ArmTuned/my_robot_arm/part6",
    joint_prim_names=["base_to_left_fingertip", "base_to_right_fingertip"],
    joint_opened_positions=np.array([0, 0]),
    joint_closed_positions=np.array([0.3, -0.3]),
    action_deltas=np.array([-0.3, 0.3]),
)

my_yellowarm = my_world.scene.add(SingleManipulator(prim_path="/World/ArmTuned", name="ArmTuned",
                                                end_effector_prim_path="/World/ArmTuned/my_robot_arm/part6",
                                                gripper=gripper))

joints_default_positions = np.zeros(8)
joints_default_positions[6] =  0
joints_default_positions[7] =  0
my_yellowarm.set_joints_default_state(positions=joints_default_positions)
my_world.scene.add_default_ground_plane()

my_world.reset()


i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        i += 1
        gripper_positions = my_yellowarm.gripper.get_joint_positions()
        if i < 300:
            my_yellowarm.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] + 0.1, gripper_positions[1] - 0.1]))
        if i > 300:
            my_yellowarm.gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] - 0.1, gripper_positions[1] + 0.1]))
        if i == 600:
            i = 0

simulation_app.close()
