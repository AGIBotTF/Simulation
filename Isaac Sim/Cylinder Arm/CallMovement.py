# | - така ще индикирам ако нещо трябва да е там
from isaacsim import SimulationApp                   # |
simulation_app = SimulationApp({"headless": False})  # |
# ^^^ с тези 2 реда пускаме симулацията и винаги седят ПРЕДИ други импорти

# vvv зареждаме нещата, които ще ни трябват (omni.isaac се лоадва динамично след като се е пуснала симулацията и съответно нито Copilot нито Editora ти знае какво има вътре)
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere

world = World(stage_units_in_meters=1.0) # |
world.scene.add_default_ground_plane()

red_cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/RedCube",
        name="red_cube",
        position=np.array([0.0, 0.0, 1.0]),
        scale=np.array([0.2, 0.2, 0.2]),
        color=np.array([1.0, 0.0, 0.0])
    )
)

green_circle = world.scene.add(
    DynamicSphere(
        prim_path="/World/GreenSphere",
        name="green_circle",
        position=np.array([-3.0, 0.0, 0.2]),
        scale=np.array([0.2, 0.2, 0.2]),  
        color=np.array([0.0, 1.0, 0.0])
    )
)


# Тази функция парсва от текст на команда и вика съответната функция - ще е полезно ако LLM-а изкарва командите по специфичен начин (можем парсера да си го направиш както искаш така е само пример)
def move_parts(command: str):
    func_name = command.replace(" ", "_")

    func = globals().get(func_name)
    if callable(func):
        func()
    else:
        print(f"Function '{func_name}' not found.")

# това е gameloop-a на симулацията:
while simulation_app.is_running(): # |
    world.step(render=True)        # |
    if world.is_playing():         # |

        # пример как да викаш мърдане
         move_parts("Move Red Cube Up")
         move_parts("Move Green Circle Forward")

simulation_app.close() # |


# Hardcoded функции за местене на 2та обекта из сцената

step = 0.005
def Move_Red_Cube_Up():
    pos, rot = red_cube.get_world_pose()
    pos[2] += step  
    red_cube.set_world_pose(pos, rot)

def Move_Red_Cube_Down():
    pos, rot = red_cube.get_world_pose()
    pos[2] -= step  
    red_cube.set_world_pose(pos, rot)

def Move_Red_Cube_Left():
    pos, rot = red_cube.get_world_pose()
    pos[1] -= step  
    red_cube.set_world_pose(pos, rot)

def Move_Red_Cube_Right():
    pos, rot = red_cube.get_world_pose()
    pos[1] += step  
    red_cube.set_world_pose(pos, rot)

def Move_Red_Cube_Forward():
    pos, rot = red_cube.get_world_pose()
    pos[0] += step  
    red_cube.set_world_pose(pos, rot)

def Move_Red_Cube_Backward():
    pos, rot = red_cube.get_world_pose()
    pos[0] -= step  
    red_cube.set_world_pose(pos, rot)

def Move_Green_Circle_Up():
    pos, rot = green_circle.get_world_pose()
    pos[2] += step
    green_circle.set_world_pose(pos, rot)

def Move_Green_Circle_Down():
    pos, rot = green_circle.get_world_pose()
    pos[2] -= step
    green_circle.set_world_pose(pos, rot)

def Move_Green_Circle_Left():
    pos, rot = green_circle.get_world_pose()
    pos[1] -= step
    green_circle.set_world_pose(pos, rot)

def Move_Green_Circle_Right():
    pos, rot = green_circle.get_world_pose()
    pos[1] += step
    green_circle.set_world_pose(pos, rot)

def Move_Green_Circle_Forward():
    pos, rot = green_circle.get_world_pose()
    pos[0] += step
    green_circle.set_world_pose(pos, rot)

def Move_Green_Circle_Backward():
    pos, rot = green_circle.get_world_pose()
    pos[0] -= step
    green_circle.set_world_pose(pos, rot)