import numpy as np
from numpy.linalg import solve
from scipy.optimize import minimize

# num_modules = 4  # number of revolute joints


# def rotation_matrix(axis, theta):
#     """
#     Return the rotation matrix for counterclockwise rotation by theta radians about a given axis.
#     """
#     axis = np.asarray(axis, dtype=float)
#     axis = axis / np.linalg.norm(axis)
#     a = np.cos(theta / 2.0)
#     b, c, d = -axis * np.sin(theta / 2.0)
#     aa, bb, cc, dd = a * a, b * b, c * c, d * d
#     bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
#     return np.array(
#         [
#             [aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
#             [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
#             [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc],
#         ]
#     )


# def rpy_to_rot(roll, pitch, yaw):
#     """
#     Convert roll, pitch, yaw angles (in radians) to a rotation matrix.
#     URDF uses the convention: R = R_z(yaw) * R_y(pitch) * R_x(roll).
#     """
#     c_r, s_r = np.cos(roll), np.sin(roll)
#     c_p, s_p = np.cos(pitch), np.sin(pitch)
#     c_y, s_y = np.cos(yaw), np.sin(yaw)

#     R_x = np.array([[1, 0, 0], [0, c_r, -s_r], [0, s_r, c_r]])

#     R_y = np.array([[c_p, 0, s_p], [0, 1, 0], [-s_p, 0, c_p]])

#     R_z = np.array([[c_y, -s_y, 0], [s_y, c_y, 0], [0, 0, 1]])

#     return R_z @ R_y @ R_x


# def forward_kinematics_urdf(angles):
#     """
#     Compute the end-effector position for your URDF-based robot arm.

#     The first joint (joint1) has no offset (origin at [0,0,0]),
#     while joints 2, 3, and 4 use the same origin:
#        - translation: [0, -cos(pi/3)+0.1, sin(pi/3)-0.155]
#        - fixed rotation: rpy = (pi/3, 0, 0)
#     Finally, the end effector is attached with a fixed joint transform:
#        - translation: [0, -cos(pi/3)-0.4, sin(pi/3)]
#        - rotation: rpy = (-pi/6, 0, 0)
#     """
#     assert len(angles) == num_modules, "angles must have length num_modules"

#     # Start with identity transformation (base_link at origin)
#     R_total = np.eye(3)
#     t_total = np.zeros(3)

#     # Joint 1 (from base_link to part1):
#     # URDF for joint1: origin at (0,0,0), axis [0,0,1]
#     R_joint1 = rotation_matrix([0, 0, 1], np.radians(angles[0]))
#     R_total = R_total @ R_joint1  # update rotation (translation remains zero)

#     # For joints 2 to 4, the URDF defines a similar transform:
#     # Joint origin translation and rotation (from part1 to part2, etc.)
#     R_origin = rpy_to_rot(np.pi / 3, 0, 0)
#     t_origin = np.array([0, -np.cos(np.pi / 3) + 0.1, np.sin(np.pi / 3) - 0.155])

#     # Loop over joints 2, 3, and 4
#     for i in range(1, num_modules):
#         # Joint rotation about the z-axis (axis defined in URDF as [0,0,1])
#         R_joint = rotation_matrix([0, 0, 1], np.radians(angles[i]))
#         # Combined module rotation: fixed origin rotation then joint rotation.
#         R_module = R_origin @ R_joint

#         # Update the total translation by adding the offset transformed by the current rotation
#         t_total = t_total + R_total @ t_origin
#         # Update the total rotation
#         R_total = R_total @ R_module

#     # Apply the fixed end-effector transform
#     # End-effector joint: origin: xyz = [0, -cos(pi/3)-0.4, sin(pi/3)] and rpy = (-pi/6, 0, 0)
#     R_end = rpy_to_rot(-np.pi / 6, 0, 0)
#     t_end = np.array([0, -np.cos(np.pi / 3) - 0.4, np.sin(np.pi / 3)])

#     t_total = t_total + R_total @ t_end
#     R_total = R_total @ R_end  # final rotation (if needed)

#     return t_total


# def forward_kinematics_simple(angles):
#     """
#     The original forward kinematics using the simple cylinder modules.
#     (This is left for reference.)
#     """
#     assert len(angles) == num_modules, "angles must have length num_modules"
#     T_total = np.eye(3)
#     translation = np.zeros(3)
#     height = 1.4

#     for i in range(num_modules):
#         bottom_angle = -30 if i % 2 == 0 else 30
#         tilt_rad = np.radians(-bottom_angle)
#         rot_axis = [np.sin(tilt_rad), 0, np.cos(tilt_rad)]
#         R_slider = rotation_matrix(rot_axis, np.radians(angles[i]))
#         T_module = T_total @ R_slider
#         top_center_local = np.array([0, 0, height])
#         top_center_world = T_module @ top_center_local + translation
#         translation = top_center_world
#         T_total = T_module
#     return translation


# def solve_ik(target, initial_guess=None, max_iter=200, use_urdf=True):
#     """
#     Solve inverse kinematics to find joint angles (in degrees) that bring the end effector
#     as close as possible to 'target'. Choose the forward kinematics model via use_urdf.
#     """
#     if initial_guess is None:
#         initial_guess = np.zeros(num_modules)
#     else:
#         initial_guess = np.asarray(initial_guess, dtype=float)

#     def objective(angles_deg):
#         if use_urdf:
#             eff_pos = forward_kinematics_urdf(angles_deg)
#         else:
#             eff_pos = forward_kinematics_simple(angles_deg)
#         return np.sum((eff_pos - target) ** 2)

#     bounds = [(-180, 180)] * num_modules

#     result = minimize(
#         objective,
#         initial_guess,
#         method="L-BFGS-B",
#         bounds=bounds,
#         options={"maxiter": max_iter},
#     )
#     return result.x

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
i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
        i += 0.01

        gripper_positions = robot_arm.gripper.get_joint_positions()

        set_positions = np.copy(joints_default_positions)

        pos, rot = target.get_world_pose()

        # print("\n\n\n\n")
        # print(solve_ik_discrete(pos))
        # simulation_app.close()

        # set_positions[0], set_positions[1], set_positions[2], set_positions[3] = solve_ik_discrete(pos)
        set_positions[0], set_positions[1], set_positions[2], set_positions[3] = (
            # solve_ik(pos, use_urdf=True)
            solve_ik(pos)
        )

        # print("Angles: ", solve_ik(pos, use_urdf=True))
        print("Angles: ", solve_ik(pos))
        effector_pos = forward_kinematics_urdf(solve_ik(pos))
        # effector_pos = forward_kinematics_urdf(solve_ik(pos, use_urdf=True))
        print("IK pos: ", pos, effector_pos)


        object=dc.get_rigid_body("/World/ArmTuned/my_robot_arm/end_effector")
        object_pose=dc.get_rigid_body_pose(object)
        print("Position of EE: ", object_pose.p)
        print("\n\n")

        ik_solution = solve_ik(pos)
        set_positions[0:4] = np.radians(ik_solution)


        set_positions[4] = gripper_positions[0]
        set_positions[5] = gripper_positions[1]
        robot_arm.set_joint_positions(set_positions)

        pos_pose, rot_pose = target_pose.get_world_pose()
        pos_pose = effector_pos
        target_pose.set_world_pose(pos_pose, rot_pose)

        # print(compute_module_transforms(solve_ik(pos))[1])

        for i in range(4):
            pos_pose, rot_pose = visulas[i].get_world_pose()
            
            pos_pose = compute_module_transforms(solve_ik(pos))[i][1]

            visulas[i].set_world_pose(pos_pose, rot_pose)

        if False:  # make it do that if distance is close to target from end effector
            robot_arm.gripper.apply_action(
                ArticulationAction(
                    joint_positions=[
                        gripper_positions[0] + 0.1,
                        gripper_positions[1] - 0.1,
                    ]
                )
            )
            # ArticulationAction(joint_positions=[gripper_positions[0] - 0.1, gripper_positions[1] + 0.1]))
        if False:  # randomise possition after it has reached target
            target.set_world_pose(pos, rot)


simulation_app.close()
