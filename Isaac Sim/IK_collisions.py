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
    """
    Compute and return the end-effector position from the given joint angles.
    angles are expected in degrees.
    """
    transforms = compute_module_transforms(angles)
    return transforms[-1][1]

def transform_vertices(vertices, R, t):
    """
    Apply a rotation R and translation t to an array of vertices.
    vertices: array of shape (n, 3)
    """
    return (R @ vertices.T).T + t

num_modules = 4  

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

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

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
import numpy as np
from omni.isaac.core.utils.stage import get_current_stage

# Define the USD paths for your arm parts and the cube.
ARM_PART_PATHS = [
    "/World/ArmTuned/my_robot_arm/part1/visuals/cylinder/mesh",
    "/World/ArmTuned/my_robot_arm/part2/visuals/cylinder/mesh",
    "/World/ArmTuned/my_robot_arm/part3/visuals/cylinder/mesh",
    "/World/ArmTuned/my_robot_arm/part4/visuals/cylinder/mesh"
]
CUBE_PATH = "/World/target"

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

initial_target_pos, _ = target.get_world_pose()
initial_ik_deg = solve_ik(initial_target_pos)

current_arm_angles = initial_ik_deg.copy()
desired_arm_angles = current_arm_angles.copy()
angle_step_deg = np.degrees(np.pi/100)  

dc = _dynamic_control.acquire_dynamic_control_interface()
# Subscribe to contact report events with the callback.
dc.get_contact_report(contact_report_callback)

while simulation_app.is_running():
    # Step the simulation and render.
    my_world.step(render=True)
    if not my_world.is_playing():
        continue

    # (Example logic to control the robot arm; replace with your code.)
    target_pos, target_rot = target.get_world_pose()
    ik_solution_deg = solve_ik(target_pos)
    ik_solution_rad = np.radians(ik_solution_deg)
    # Initialize joint positions array.
    set_positions = np.copy(joints_default_positions)
    # Assign the first 4 joint positions from the IK solution.
    set_positions[0:4] = ik_solution_rad
    # The last two positions control the gripper (unchanged here).
    set_positions[4] = gripper_positions[0]
    set_positions[5] = gripper_positions[1]
    robot_arm.set_joint_positions(set_positions)

    # Update the end-effector pose marker (or target pose) if needed.
    effector_pos = forward_kinematics_urdf(ik_solution_deg)
    target_pose.set_world_pose(effector_pos, target_rot)

    if my_world.current_time_step_index == 0:
        my_world.reset()

    gripper_positions = robot_arm.gripper.get_joint_positions()

    target_pos, target_rot = target.get_world_pose()

    if np.allclose(current_arm_angles, desired_arm_angles, atol=0.5):

        desired_arm_angles = solve_ik(target_pos)

    effector_pos = forward_kinematics_urdf(current_arm_angles)
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

        desired_arm_angles = solve_ik(new_target_pos)
        target_gripped_counter = 0

    for i in range(len(current_arm_angles)):
        diff = desired_arm_angles[i] - current_arm_angles[i]
        if abs(diff) > angle_step_deg:
            current_arm_angles[i] += np.sign(diff) * angle_step_deg
        else:
            current_arm_angles[i] = desired_arm_angles[i]

    set_positions = np.copy(joints_default_positions)
    set_positions[0:4] = np.radians(current_arm_angles)

    set_positions[4] = gripper_positions[0]
    set_positions[5] = gripper_positions[1]
    robot_arm.set_joint_positions(set_positions)

    effector_pos = forward_kinematics_urdf(current_arm_angles)
    target_pose.set_world_pose(effector_pos, target_rot)

    module_transforms = compute_module_transforms(current_arm_angles)
    for idx in range(4):
        _, current_vis_rot = visulas[idx].get_world_pose()
        new_vis_pos = module_transforms[idx][1]
        visulas[idx].set_world_pose(new_vis_pos, current_vis_rot)

simulation_app.close()


# Helper function to change the display color of a prim.
def change_prim_color(prim_path: str, color: np.ndarray):
    """
    Changes the prim's display color.
    color should be a 3-element numpy array [R, G, B], with values in [0.0, 1.0].
    """
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if prim:
        attr = prim.GetAttribute("primvars:displayColor")
        if attr:
            # Set color; note that it expects a list of colors (one per vertex or for the prim)
            attr.Set([list(color)])
        else:
            print(f"Warning: 'primvars:displayColor' attribute not found on {prim_path}")
    else:
        print(f"Warning: Prim not found at {prim_path}")

# Collision (contact) callback function.
def contact_report_callback(contact_report):
    """
    This callback is invoked after each physics step.
    It iterates over all contacts in the current report.
    For each contact, if one of the contacts involves both the cube and any arm part,
    the corresponding arm part's color is set to red.
    Otherwise, the arm parts are reset to blue.
    """
    # Use a dictionary to keep track of arm parts that encountered collisions.
    collided = {part: False for part in ARM_PART_PATHS}

    for contact in contact_report.contacts:
        # Adjust these attribute names if your contact report uses different field names.
        prim0 = contact.prim0   # e.g. a string with a prim path
        prim1 = contact.prim1

        # Check each arm part for a collision with the cube.
        for arm_part in ARM_PART_PATHS:
            if ((arm_part in prim0 and CUBE_PATH in prim1) or 
                (arm_part in prim1 and CUBE_PATH in prim0)):
                collided[arm_part] = True
                # Change the color to red immediately.
                change_prim_color(arm_part, np.array([1.0, 0.0, 0.0]))
                print(f"Collision detected between {arm_part} and {CUBE_PATH}")

    # For any arm part that did not encounter a collision, reset its color to blue.
    for arm_part, hit in collided.items():
        if not hit:
            change_prim_color(arm_part, np.array([0.0, 0.0, 1.0]))
