import numpy as np
from numpy.linalg import solve
from scipy.optimize import minimize
import math

def rotation_matrix(axis, theta):
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

def compute_module_transforms(angles):
    transforms = []  
    R_total = np.eye(3)
    t_total = np.zeros(3)

    R_joint1 = rotation_matrix([0, 0, 1], np.radians(angles[0]))
    R_total = R_total @ R_joint1
    transforms.append((R_total.copy(), t_total.copy()))

    R_origin = rpy_to_rot(np.pi/3, 0, 0)
    t_origin = np.array([0, -np.cos(np.pi/3) + 0.1, np.sin(np.pi/3) - 0.155])

    for i in range(1, len(angles)):
        R_joint = rotation_matrix([0, 0, 1], np.radians(angles[i]))
        R_module = R_origin @ R_joint
        t_total = t_total + R_total @ t_origin
        R_total = R_total @ R_module
        transforms.append((R_total.copy(), t_total.copy()))

    R_end = rpy_to_rot(-np.pi/6, 0, 0)
    t_end = np.array([0, -np.cos(np.pi/3) - 0.4, np.sin(np.pi/3)])
    t_total = t_total + R_total @ t_end
    R_total = R_total @ R_end
    transforms.append((R_total.copy(), t_total.copy()))

    return transforms

def forward_kinematics_urdf(angles):
    transforms = compute_module_transforms(angles)
    return transforms[-1][1]

def transform_vertices(vertices, R, t):
    return (R @ vertices.T).T + t

num_modules = 4  
def solve_ik(target, initial_guess=None, max_iter=200):
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

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

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
visulas = []
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

dc = _dynamic_control.acquire_dynamic_control_interface()
my_world.reset()

TARGET_GRIP_THRESHOLD = 300  
DISTANCE_THRESHOLD = 0.1     
target_gripped_counter = 0

SMOOTHING_MODE = "manual"

current_joint_angles = np.copy(joints_default_positions)

while simulation_app.is_running():
    my_world.step(render=True)
    if not my_world.is_playing():
        continue

    if my_world.current_time_step_index == 0:
        my_world.reset()

    gripper_positions = robot_arm.gripper.get_joint_positions()
    target_pos, target_rot = target.get_world_pose()

    ik_solution_deg = solve_ik(target_pos)
    ik_solution_rad = np.radians(ik_solution_deg)

    if SMOOTHING_MODE == "drive":

        set_positions = np.copy(joints_default_positions)
        set_positions[0:4] = ik_solution_rad        
        set_positions[4] = gripper_positions[0]
        set_positions[5] = gripper_positions[1]

        robot_arm.apply_action(ArticulationAction(joint_positions=set_positions))

        current_joint_angles = set_positions.copy()

    elif SMOOTHING_MODE == "manual":

        target_angles = ik_solution_rad
        max_step = 0.01  
        new_angles = []
        for curr, target_angle in zip(current_joint_angles[0:4], target_angles):
            diff = target_angle - curr
            if abs(diff) > max_step:
                step = max_step if diff > 0 else -max_step
                new_angles.append(curr + step)
            else:
                new_angles.append(target_angle)
        new_angles = np.array(new_angles)
        current_joint_angles[0:4] = new_angles

        current_joint_angles[4] = gripper_positions[0]
        current_joint_angles[5] = gripper_positions[1]

        robot_arm.set_joint_positions(current_joint_angles)

    effector_pos = forward_kinematics_urdf(np.degrees(current_joint_angles[0:4]))
    target_pose.set_world_pose(effector_pos, target_rot)

    module_transforms = compute_module_transforms(np.degrees(current_joint_angles[0:4]))
    for idx in range(4):
        _, current_vis_rot = visulas[idx].get_world_pose()
        new_vis_pos = module_transforms[idx][1]
        visulas[idx].set_world_pose(new_vis_pos, current_vis_rot)

    distance = np.linalg.norm(effector_pos - target_pos)
    if distance < DISTANCE_THRESHOLD:
        robot_arm.gripper.apply_action(
            ArticulationAction(joint_positions=[gripper_positions[0] + 0.1,
                                                 gripper_positions[1] - 0.1])
        )
        target_gripped_counter += 1
    else:
        robot_arm.gripper.apply_action(
            ArticulationAction(joint_positions=[gripper_positions[0] - 0.1,
                                                 gripper_positions[1] + 0.1])
        )
        target_gripped_counter = 0

    if target_gripped_counter > TARGET_GRIP_THRESHOLD:
        new_target_pos = np.array([2 * np.random.rand() - 1,
                                   2 * np.random.rand() - 1,
                                   2 * np.random.rand()])
        target.set_world_pose(new_target_pos, target_rot)
        robot_arm.gripper.apply_action(
            ArticulationAction(joint_positions=[gripper_positions[0] - 0.1,
                                                 gripper_positions[1] + 0.1])
        )
        target_gripped_counter = 0

simulation_app.close()