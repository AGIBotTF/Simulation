# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script demonstrates how to spawn prims into the scene.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/00_sim/spawn_prims.py

"""

"""Launch Isaac Sim Simulator first."""


import argparse

from isaaclab.app import AppLauncher
import inspect

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on spawning prims into the scene.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import isaacsim.core.utils.prims as prim_utils

import isaaclab.sim as sim_utils

cfg_cube = sim_utils.CuboidCfg(
        size=(0.5, 1, 0.5),
        rigid_props = sim_utils.RigidBodyPropertiesCfg(),
        collision_props = sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
    )

def design_scene():
    """Designs the scene by spawning ground plane, light, objects and meshes from usd files."""
    # Ground-plane
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    # spawn distant light
    cfg_light_distant = sim_utils.DistantLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1, 0, 10))

    # create a new xform prim for all objects to be spawned under
    prim_utils.create_prim("/World/Objects", "Xform")
    # spawn a red cone
    print("\n\n\n\n\n\n\n")
    try:
        # Get file path and source code
        file_path = inspect.getfile(sim_utils.CuboidCfg)
        source_code = inspect.getsource(sim_utils.CuboidCfg)
        
        print(f"Class definition found in: {file_path}")
        print("\nSource code:")
        print(source_code)
    except Exception as e:
        print(f"Error: {e}")
        # Alternative approach
        print("\nModule info:")
        module_name = sim_utils.CuboidCfg.__module__
        module = __import__(module_name, fromlist=[''])
        print(f"Module: {module_name}")
        if hasattr(module, '__file__'):
            print(f"Module file: {module.__file__}")
    print("\n\n\n\n\n\n\n")
    
    cfg_cube.func("/World/Objects/Cube", cfg_cube, translation=(-3.0, 1.0, 1.0))



def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.0, 0.0, 2.5], [-0.5, 0.0, 0.5])

    # Design scene by adding assets to it
    design_scene()

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()
        cfg_cube.translation = (cfg_cube.translation[0], cfg_cube.translation[1], cfg_cube.translation[2] + 0.01)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
