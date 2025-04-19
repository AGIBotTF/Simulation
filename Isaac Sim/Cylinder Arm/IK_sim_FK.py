import numpy as np
from scipy.optimize import minimize

# --- Isaac Sim / Omniverse Setup ---
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.dynamic_control import _dynamic_control

# Create the simulation world and add a ground plane.
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# Load the robot arm asset (update the usd_path as needed).
asset_path = "C:/Users/krasi/OneDrive/Documents/TONI_AI_NN_training/isaac/EndEff.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/ArmTuned")

# Setup gripper and robot arm (we assume the first 4 joints are controlled by IK).
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
        position=np.array([0.0, 0.0, -0.1]),
    )
)

# Define the number of joints: we assume 4 joints for the arm IK and a couple for the gripper.
num_modules = 4  
num_joints = 7
joints_default_positions = np.zeros(num_joints)
robot_arm.set_joints_default_state(positions=joints_default_positions)

# Create a target object (a small red cuboid) for the IK to reach.
target = my_world.scene.add(
    VisualCuboid(
        prim_path="/World/target",
        name="target",
        position=np.array([1.0, 2, 1.1]),
        scale=np.array([0.1, 0.1, 0.1]),
        color=np.array([1.0, 0.0, 0.0]),
    )
)

# Acquire the dynamic control interface to query rigid-body poses.
dc = _dynamic_control.acquire_dynamic_control_interface()

my_world.reset()

# --- Using Simulation as Forward Kinematics ---

def forward_kinematics_sim(angles):
    """
    Use the simulation as a forward kinematics oracle:
      1. Update the robot arm joints with the candidate angles (only the first num_modules joints).
      2. Step the simulation to update the physical state.
      3. Return the end-effector's position from the dynamic control interface.
    """
    # Set only the IK-controlled joints (first num_modules joints).
    set_positions = np.copy(joints_default_positions)
    set_positions[:num_modules] = angles
    robot_arm.set_joint_positions(set_positions)
    
    # Step the simulation so that the new joint configuration is processed.
    my_world.step(render=True)
    
    # Query the end-effector pose.
    ee_body = dc.get_rigid_body("/World/ArmTuned/my_robot_arm/end_effector")
    ee_pose = dc.get_rigid_body_pose(ee_body)
    return np.array(ee_pose.p)

def solve_ik_sim(target_pos, initial_guess=None, max_iter=50):
    """
    Solve inverse kinematics using simulation-based forward kinematics.
    The objective is to minimize the squared distance between the target position
    and the end-effector position as obtained from simulation.
    """
    if initial_guess is None:
        initial_guess = np.zeros(num_modules)
    else:
        initial_guess = np.asarray(initial_guess, dtype=float)

    def objective(angles):
        eff_pos = forward_kinematics_sim(angles)
        return np.sum((eff_pos - target_pos)**2)

    bounds = [(-180, 180)] * num_modules
    result = minimize(objective, initial_guess, method="L-BFGS-B", bounds=bounds, options={"maxiter": max_iter})
    return result.x

# --- Main Simulation Loop ---
i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        i += 0.01

        # Get the current target world position.
        target_pos, target_rot = target.get_world_pose()

        # Solve IK using the simulation-based forward kinematics.
        ik_angles = solve_ik_sim(np.array(target_pos))
        print("IK angles:", ik_angles)

        # Update the robot arm: apply the IK solution for the first num_modules joints,
        # and leave the remaining joints (e.g. gripper) as they are.
        set_positions = np.copy(joints_default_positions)
        set_positions[:num_modules] = ik_angles
        # For example, if the gripper joints are at indices 4 and 5, keep them unchanged.
        set_positions[4:6] = robot_arm.gripper.get_joint_positions()
        robot_arm.set_joint_positions(set_positions)

        # Print the simulation’s end-effector position for comparison.
        ee_body = dc.get_rigid_body("/World/ArmTuned/my_robot_arm/end_effector")
        ee_pose = dc.get_rigid_body_pose(ee_body)
        print("Simulation End-Effector Position:", ee_pose.p)
        print("\n")

simulation_app.close()