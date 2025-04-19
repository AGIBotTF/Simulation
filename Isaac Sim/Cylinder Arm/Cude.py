# from omni.isaac.kit import SimulationApp
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=np.array([0.0, 0.0, 1.0]),
        scale=np.array([0.2, 0.2, 0.2]),
        color=np.array([1.0, 0.0, 0.0])
    )
)

while simulation_app.is_running():
    world.step(render=True)
    if world.is_playing():
         pos, rot = cube.get_world_pose()

         pos[0] += 0.005

         cube.set_world_pose(pos, rot)
simulation_app.close()

