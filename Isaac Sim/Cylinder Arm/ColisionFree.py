import numpy as np
from numpy.linalg import norm
from scipy.optimize import minimize
import math

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.kit.commands import execute
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.core.objects import VisualCuboid
from omni.isaac.sensor import ContactSensor
from omni.isaac.core.utils.types import ArticulationAction

def rotation_matrix(axis, theta):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    a = math.cos(theta/2)
    b, c, d = -axis * math.sin(theta/2)
    return np.array([
        [a*a + b*b - c*c - d*d,   2*(b*c + a*d),       2*(b*d - a*c)],
        [2*(b*c - a*d),           a*a + c*c - b*b - d*d, 2*(c*d + a*b)],
        [2*(b*d + a*c),           2*(c*d - a*b),       a*a + d*d - b*b - c*c]
    ])

def rpy_to_rot(roll, pitch, yaw):
    c_r, s_r = math.cos(roll), math.sin(roll)
    c_p, s_p = math.cos(pitch), math.sin(pitch)
    c_y, s_y = math.cos(yaw), math.sin(yaw)
    R_x = np.array([[1,0,0],[0,c_r,-s_r],[0,s_r,c_r]])
    R_y = np.array([[c_p,0,s_p],[0,1,0],[-s_p,0,c_p]])
    R_z = np.array([[c_y,-s_y,0],[s_y,c_y,0],[0,0,1]])
    return R_z @ R_y @ R_x

def compute_module_transforms(angles_deg):
    angles = np.radians(angles_deg)
    transforms = []
    R_total = np.eye(3)
    t_total = np.zeros(3)

    R0 = rotation_matrix([0,0,1], angles[0])
    R_total = R_total @ R0
    transforms.append((R_total.copy(), t_total.copy()))

    R_origin = rpy_to_rot(np.pi/3, 0, 0)
    t_origin = np.array([0, -math.cos(np.pi/3) + 0.1, math.sin(np.pi/3) - 0.155])
    for i in range(1, 4):
        Rj = rotation_matrix([0,0,1], angles[i])
        R_module = R_origin @ Rj
        t_total = t_total + R_total @ t_origin
        R_total = R_total @ R_module
        transforms.append((R_total.copy(), t_total.copy()))

    R_end = rpy_to_rot(-np.pi/6, 0, 0)
    t_end = np.array([0, -math.cos(np.pi/3) - 0.4, math.sin(np.pi/3)])
    t_total = t_total + R_total @ t_end
    R_total = R_total @ R_end
    transforms.append((R_total.copy(), t_total.copy()))
    return transforms

def forward_kinematics(angles_deg):
    return compute_module_transforms(angles_deg)[-1][1]

def solve_ik(target, initial_guess=None, max_iter=200):
    if initial_guess is None:
        x0 = np.zeros(4)
    else:
        x0 = np.array(initial_guess, dtype=float)
    def obj(a):
        return np.sum((forward_kinematics(a) - target)**2)
    bounds = [(-180,180)]*4
    res = minimize(obj, x0, method="L-BFGS-B", bounds=bounds, options={"maxiter":max_iter})
    return res.x

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

asset_path = "C:/Users/krasi/OneDrive/Documents/TONI_AI_NN_training/isaac/EndEff.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/ArmTuned")

gripper = ParallelGripper(
    end_effector_prim_path="/World/ArmTuned/my_robot_arm/end_effector",
    joint_prim_names=["base_to_left_fingertip","base_to_right_fingertip"],
    joint_opened_positions=np.array([0,0]),
    joint_closed_positions=np.array([0.3,-0.3]),
    action_deltas=np.array([-0.3,0.3]),
)
robot_arm = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/ArmTuned",
        name="ArmTuned",
        end_effector_prim_path="/World/ArmTuned/my_robot_arm/end_effector",
        gripper=gripper,
        position=np.array([0,0,-0.1]),
    )
)
num_joints = 7
joints_default = np.zeros(num_joints)
robot_arm.set_joints_default_state(positions=joints_default)

target = my_world.scene.add(
    VisualCuboid(
        prim_path="/World/target",
        name="target",
        position=np.array([1.0, 2.0, 1.1]),
        scale=np.array([0.1,0.1,0.1]),
        color=np.array([1,0,0]),
    )
)

debug_vis = []
for i in range(4):
    debug_vis.append(
        my_world.scene.add(
            VisualCuboid(
                prim_path=f"/World/vis_mod{i}",
                name=f"vis_mod{i}",
                position=np.zeros(3),
                scale=np.array([0.05,0.05,0.05]),
                color=np.array([0,0,1]),
            )
        )
    )

target_pose = my_world.scene.add(
    VisualCuboid(
        prim_path="/World/target_pose",
        name="target_pose",
        position=np.array([1.0,2.0,1.1]),
        scale=np.array([0.2,0.09,0.09]),
        color=np.array([0,1,0]),
    )
)

my_world.reset()

COLLISION_THRESH = 1e-3  
link_sensors = []
for idx in range(4):
    link_prim = f"/World/ArmTuned/my_robot_arm/module{idx}_link"
    sensor_name = f"module{idx}_contact"
    execute(
        "IsaacSensorCreateContactSensor",
        path=sensor_name,
        parent=link_prim,
        sensor_period=1,
        min_threshold=0.0,
        max_threshold=1e6,
        translation=[0,0,0],
    )
    cs = ContactSensor(
        prim_path=f"{link_prim}/{sensor_name}",
        name=sensor_name,
        frequency=60,
        translation=np.array([0,0,0]),
        min_threshold=0.0,
        max_threshold=1e6,
        radius=-1,
    )
    my_world.scene.add(cs)
    link_sensors.append(cs)

def plan_lift_and_go(start_pos, goal_pos, lift_height=0.5):
    mid = np.array([start_pos[0], start_pos[1], max(start_pos[2], goal_pos[2]) + lift_height])
    via = [mid, goal_pos]
    sols = []
    guess = None
    for pt in via:
        sol = solve_ik(pt, initial_guess=guess)

        rad = np.radians(sol)
        trial = np.zeros(num_joints); trial[0:4] = rad
        robot_arm.set_joint_positions(trial)
        my_world.step(render=False)

        for s in link_sensors:
            frame = s.get_current_frame()
            if frame["in_contact"] or norm(frame["force"]) > COLLISION_THRESH:
                raise RuntimeError(f"Collision at waypoint {pt}")
        sols.append(sol)
        guess = sol
    return sols

SMOOTH_STEP = 0.01
waypoints = []; way_idx = 0
current_angles = joints_default.copy()

while simulation_app.is_running():
    my_world.step(render=True)
    if not my_world.is_playing(): continue
    if my_world.current_time_step_index == 0:
        my_world.reset()

    grip = robot_arm.gripper.get_joint_positions()
    targ_pos, targ_rot = target.get_world_pose()

    if way_idx >= len(waypoints):
        try:
            eff_pos = forward_kinematics(np.degrees(current_angles[:4]))
            plan = plan_lift_and_go(eff_pos, targ_pos)
            waypoints = []
            for deg in plan:
                rad = np.radians(deg)
                arr = np.zeros(num_joints)
                arr[0:4] = rad; arr[4:6] = grip
                waypoints.append(arr)
            way_idx = 0
        except RuntimeError as e:
            print("Plan failed:", e)
            waypoints, way_idx = [], 0

    if way_idx < len(waypoints):
        tgt = waypoints[way_idx]
        new = current_angles.copy()
        for i in range(4):
            d = tgt[i] - current_angles[i]
            new[i] += np.sign(d)*min(abs(d), SMOOTH_STEP)
        new[4:6] = grip
        robot_arm.set_joint_positions(new)

        my_world.step(render=False)
        collided = False
        for s in link_sensors:
            f = s.get_current_frame()
            if f["in_contact"] or norm(f["force"]) > COLLISION_THRESH:
                collided = True; break
        if not collided:
            current_angles = new
            if norm(current_angles[:4] - tgt[:4]) < 1e-3:
                way_idx += 1
        else:
            print("Collision mid‑move; stopping.")

    eff = forward_kinematics(np.degrees(current_angles[:4]))
    target_pose.set_world_pose(eff, targ_rot)
    for i, vis in enumerate(debug_vis):
        pos = compute_module_transforms(np.degrees(current_angles[:4]))[i][1]
        _, rot = vis.get_world_pose()
        vis.set_world_pose(pos, rot)

simulation_app.close()