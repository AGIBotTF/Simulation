import numpy as np
from numpy.linalg import solve
from scipy.optimize import minimize

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix for counterclockwise rotation by theta radians about a given axis.
    """
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    a = np.cos(theta / 2.0)
    b, c, d = -axis * np.sin(theta / 2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([
        [aa + bb - cc - dd, 2*(bc + ad), 2*(bd - ac)],
        [2*(bc - ad), aa + cc - bb - dd, 2*(cd + ab)],
        [2*(bd + ac), 2*(cd - ab), aa + dd - bb - cc]
    ])

def rpy_to_rot(roll, pitch, yaw):
    """
    Convert roll, pitch, yaw angles (in radians) to a rotation matrix.
    URDF convention: R = R_z(yaw) * R_y(pitch) * R_x(roll)
    """
    c_r, s_r = np.cos(roll), np.sin(roll)
    c_p, s_p = np.cos(pitch), np.sin(pitch)
    c_y, s_y = np.cos(yaw), np.sin(yaw)
    
    R_x = np.array([
        [1,    0,     0],
        [0,  c_r, -s_r],
        [0,  s_r,  c_r]
    ])
    
    R_y = np.array([
        [ c_p, 0, s_p],
        [   0, 1,   0],
        [-s_p, 0, c_p]
    ])
    
    R_z = np.array([
        [c_y, -s_y, 0],
        [s_y,  c_y, 0],
        [  0,    0, 1]
    ])
    
    return R_z @ R_y @ R_x

# ---------------------------
# Compute Transforms for Each Module
# ---------------------------
def compute_module_transforms(angles):
    """
    Given a set of joint angles (in degrees) for a num_modules arm,
    compute a list of (R, t) transforms for each module (link) and
    an additional one for the end effector.
    
    Transforms based on URDF parameters:
      - Joint1: origin at [0,0,0], rotates about z.
      - Joints 2-4: fixed transform with:
           translation: [0, -cos(pi/3)+0.1, sin(pi/3)-0.155]
           fixed rotation: rpy = (pi/3, 0, 0),
           then a rotation about z by the joint angle.
      - End-effector: fixed transform:
           translation: [0, -cos(pi/3)-0.4, sin(pi/3)]
           rotation: rpy = (-pi/6, 0, 0)
    """
    transforms = []  # list of (R, t) tuples
    R_total = np.eye(3)
    t_total = np.zeros(3)
    
    # Joint 1 (base_link -> part1)
    R_joint1 = rotation_matrix([0, 0, 1], np.radians(angles[0]))
    R_total = R_total @ R_joint1
    transforms.append((R_total.copy(), t_total.copy()))
    
    # Fixed transform for joints 2 to 4
    R_origin = rpy_to_rot(np.pi/3, 0, 0)
    t_origin = np.array([0, -np.cos(np.pi/3) + 0.1, np.sin(np.pi/3) - 0.155])
    
    for i in range(1, len(angles)):
        R_joint = rotation_matrix([0, 0, 1], np.radians(angles[i]))
        R_module = R_origin @ R_joint
        t_total = t_total + R_total @ t_origin
        R_total = R_total @ R_module
        transforms.append((R_total.copy(), t_total.copy()))
    
    # End-effector transform (attached to the last module)
    R_end = rpy_to_rot(-np.pi/6, 0, 0)
    t_end = np.array([0, -np.cos(np.pi/3) - 0.4, np.sin(np.pi/3)])
    t_total = t_total + R_total @ t_end
    R_total = R_total @ R_end
    transforms.append((R_total.copy(), t_total.copy()))
    
    return transforms

def forward_kinematics_urdf(angles):
    """
    Compute and return the end-effector position from the given joint angles.
    """
    transforms = compute_module_transforms(angles)
    # The end effector transform is the last element in the list.
    return transforms[-1][1]

def transform_vertices(vertices, R, t):
    """
    Apply a rotation R and translation t to an array of vertices.
    vertices: array of shape (n, 3)
    """
    return (R @ vertices.T).T + t

# ---------------------------
# Inverse Kinematics Solver
# ---------------------------
num_modules = 4  # number of revolute joints

def solve_ik(target, initial_guess=None, max_iter=200):
    """
    Solve inverse kinematics to find a set of joint angles (in degrees)
    that minimizes the squared distance between the end effector and target.
    """
    if initial_guess is None:
        initial_guess = np.zeros(num_modules)
    else:
        initial_guess = np.asarray(initial_guess, dtype=float)

    def objective(angles):
        eff = forward_kinematics_urdf(angles)
        return np.sum((eff - target)**2)

    bounds = [(-180, 180)] * num_modules

    result = minimize(objective, initial_guess, method="L-BFGS-B", bounds=bounds, options={"maxiter": max_iter})
    return result.x


# Example usage:
# my_target = np.array([1.0, 2.0, 1.1])
# ik_solution = solve_ik(my_target, use_urdf=True)
# effector_pos = forward_kinematics_urdf(ik_solution)
# print("IK solution angles (degrees):", ik_solution)
# print("End effector position:", effector_pos)
# print("Target position:", my_target)
# print("Distance to target:", np.linalg.norm(my_target - effector_pos))
# ---------------------
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
# simulation_app = SimulationApp({"headless": True})

import numpy as np
import math
from omni.isaac.core import World
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.objects import VisualCuboid

from omni.usd import get_context
from pxr import UsdGeom

from omni.isaac.dynamic_control import _dynamic_control


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
        position=np.array([0.0, 0.0, -0.1]),
    )
)

num_joints = 7
joints_default_positions = np.zeros(num_joints)
robot_arm.set_joints_default_state(positions=joints_default_positions)

target = my_world.scene.add(
    VisualCuboid(
        prim_path="/World/target",
        name="target",
        position=np.array([1.0, 2, 1.1]),
        scale=np.array([0.1, 0.1, 0.1]),
        color=np.array([1.0, 0.0, 0.0]),
    )
)
visulas =[]
for i in range(4):
    visulas.append(my_world.scene.add(
        VisualCuboid(
            prim_path=f"/World/target{i}",
            name=f"target{i}",
            position=np.array([0.0, 0, 0]),
            scale=np.array([0.1, 0.1, 0.1]),
            color=np.array([0.0, 0.0, 1.0]),
        )
    ))

target_pose = my_world.scene.add(
    VisualCuboid(
        prim_path="/World/target_pose",
        name="target_pose",
        position=np.array([1.0, 2, 1.1]),
        scale=np.array([0.2, 0.09, 0.09]),
        color=np.array([0.0, 1.0, 0.0]),
    )
)

dc=_dynamic_control.acquire_dynamic_control_interface()

my_world.reset()

# Constants and state variables
TARGET_GRIP_THRESHOLD = 300  # Number of simulation steps to consider target "gripped"
DISTANCE_THRESHOLD = 0.1     # Distance below which we consider the target reached
target_gripped_counter = 0

while simulation_app.is_running():
    my_world.step(render=True)
    if not my_world.is_playing():
        continue

    # Reset world at the beginning of an episode if needed.
    if my_world.current_time_step_index == 0:
        my_world.reset()

    # Get the current gripper joint positions
    gripper_positions = robot_arm.gripper.get_joint_positions()

    # Get the current target pose
    target_pos, target_rot = target.get_world_pose()

    # Solve IK for the current target position (returns angles in degrees)
    ik_solution_deg = solve_ik(target_pos)
    # Convert IK solution to radians (simulation expects radians)
    ik_solution_rad = np.radians(ik_solution_deg)
    
    # Update the robot arm joints:
    # - The first 4 joints are from our IK solution.
    # - The gripper joints (positions 4 and 5) are kept as-is.
    set_positions = np.copy(joints_default_positions)
    set_positions[0:4] = ik_solution_rad
    set_positions[4] = gripper_positions[0]
    set_positions[5] = gripper_positions[1]
    robot_arm.set_joint_positions(set_positions)

    # Compute the end-effector position using forward kinematics
    effector_pos = forward_kinematics_urdf(ik_solution_deg)
    # Update the target_pose visual (e.g., for debugging) to follow the effector position
    target_pose.set_world_pose(effector_pos, target_rot)

    # Update visual blue cubes along the arm's segments for debugging:
    module_transforms = compute_module_transforms(ik_solution_deg)
    for idx in range(4):
        # You can choose to update the orientation as needed; here we keep the current one.
        _, current_vis_rot = visulas[idx].get_world_pose()
        new_vis_pos = module_transforms[idx][1]
        visulas[idx].set_world_pose(new_vis_pos, current_vis_rot)

    # Check the distance between the end effector and target
    distance = np.linalg.norm(effector_pos - target_pos)
    if distance < DISTANCE_THRESHOLD:
        # When close, apply a closing action to the gripper.
        robot_arm.gripper.apply_action(
            ArticulationAction(joint_positions=[gripper_positions[0] + 0.1,
                                                 gripper_positions[1] - 0.1])
        )
        target_gripped_counter += 1
    else:
        # Otherwise, open the gripper and reset the counter.
        robot_arm.gripper.apply_action(
            ArticulationAction(joint_positions=[gripper_positions[0] - 0.1,
                                                 gripper_positions[1] + 0.1])
        )
        target_gripped_counter = 0

    # After the target has been gripped for enough frames, generate a new target.
    if target_gripped_counter > TARGET_GRIP_THRESHOLD:
        # Generate a new random target position within specified bounds.
        new_target_pos = np.array([2 * np.random.rand() - 1,
                                   2 * np.random.rand() - 1,
                                   2 * np.random.rand()])
        target.set_world_pose(new_target_pos, target_rot)
        # Immediately open the gripper after moving the target.
        robot_arm.gripper.apply_action(
            ArticulationAction(joint_positions=[gripper_positions[0] - 0.1,
                                                 gripper_positions[1] + 0.1])
        )
        target_gripped_counter = 0

simulation_app.close()


# i = 0
# catched = 0
# while simulation_app.is_running():
#     my_world.step(render=True)
#     if my_world.is_playing():
#         if my_world.current_time_step_index == 0:
#             my_world.reset()
#         i += 0.01

#         gripper_positions = robot_arm.gripper.get_joint_positions()

#         set_positions = np.copy(joints_default_positions)

#         pos, rot = target.get_world_pose()


#         # print("Angles: ", solve_ik(pos))
#         effector_pos = forward_kinematics_urdf(solve_ik(pos))
#         # print("IK pos: ", pos, effector_pos)


#         # object=dc.get_rigid_body("/World/ArmTuned/my_robot_arm/end_effector")
#         # object_pose=dc.get_rigid_body_pose(object)
#         # print("Position of EE: ", object_pose.p)
#         # print("\n\n")

#         ik_solution = solve_ik(pos)
#         set_positions[0:4] = np.radians(ik_solution)


#         set_positions[4] = gripper_positions[0]
#         set_positions[5] = gripper_positions[1]
#         robot_arm.set_joint_positions(set_positions)

#         pos_pose, rot_pose = target_pose.get_world_pose()
#         pos_pose = effector_pos
#         target_pose.set_world_pose(pos_pose, rot_pose)


#         for i in range(4):
#             pos_pose, rot_pose = visulas[i].get_world_pose()
            
#             pos_pose = compute_module_transforms(solve_ik(pos))[i][1]

#             visulas[i].set_world_pose(pos_pose, rot_pose)

#         if True:  # make it do that if distance is close to target from end effector
#             if np.linalg.norm(effector_pos - pos) < 0.1:
#                 robot_arm.gripper.apply_action(
#                 ArticulationAction(joint_positions=[gripper_positions[0] + 0.1, gripper_positions[1] - 0.1]))
        
#                 catched += 1
#             else:
#                 robot_arm.gripper.apply_action(
#                 ArticulationAction(joint_positions=[gripper_positions[0] - 0.1, gripper_positions[1] + 0.1]))
        
#                 catched = 0
#             # ArticulationAction(joint_positions=[gripper_positions[0] - 0.1, gripper_positions[1] + 0.1]))
#         if True and catched>300:  # randomise possition after it has reached target
#             # target.set_world_pose(pos, rot)
#             pos, rot = target.get_world_pose()
#             pos[0] = 2 * np.random.rand() - 1
#             pos[1] = 2 * np.random.rand() - 1
#             pos[2] = 2 * np.random.rand()
#             target.set_world_pose(pos, rot)


# simulation_app.close()
