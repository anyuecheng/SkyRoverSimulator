"""
| File: params.py
| Author: Fei Wang (feiwang@dlmu.edu.cn)
| License: BSD-3-Clause. Copyright (c) 2025, Fei Wang. All rights reserved.
"""
import os
from pathlib import Path

import isaacsim.storage.native as nucleus

# Extension configuration
EXTENSION_NAME = "SkyRover Simulator"
WINDOW_TITLE = "SkyRover Simulator"
MENU_PATH = "Window/" + WINDOW_TITLE
# DOC_LINK = "https://docs.omniverse.nvidia.com"
# EXTENSION_OVERVIEW = "This extension shows how to incorporate drones into Isaac Sim"

# Get the current directory of where this extension is located
EXTENSION_FOLDER_PATH = Path(os.path.dirname(os.path.realpath(__file__)))
ROOT = str(EXTENSION_FOLDER_PATH.parent.parent.parent.parent.resolve())

# Define the Extension Assets Path
ASSET_PATH = ROOT + "/skyrover.simulator/skyrover/simulator/assets"
ROBOTS_ASSETS = ASSET_PATH + "/Robots"

# Define the built in robots of the extension
AERIAL_ROBOTS = {"Iris": ROBOTS_ASSETS + "/Iris/iris.usd"} #, "Flying Cube": ROBOTS_ASSETS + "/iris_cube.usda"}
GROUND_ROBOTS = {"Iris": ROBOTS_ASSETS + "/Iris/iris.usd"} #, "Flying Cube": ROBOTS_ASSETS + "/iris_cube.usda"}

# Setup the default simulation environments path
NVIDIA_ASSETS_PATH = str(nucleus.get_assets_root_path())
ISAAC_SIM_ENVIRONMENTS = "/Isaac/Environments"
NVIDIA_SIMULATION_ENVIRONMENTS = {
    "Default Environment": "Grid/default_environment.usd",
    "Black Gridroom": "Grid/gridroom_black.usd",
    "Curved Gridroom": "Grid/gridroom_curved.usd",
    "Hospital": "Hospital/hospital.usd",
    "Office": "Office/office.usd",
    "Simple Room": "Simple_Room/simple_room.usd",
    "Warehouse": "Simple_Warehouse/warehouse.usd",
    "Warehouse with Forklifts": "Simple_Warehouse/warehouse_with_forklifts.usd",
    "Warehouse with Shelves": "Simple_Warehouse/warehouse_multiple_shelves.usd",
    "Full Warehouse": "Simple_Warehouse/full_warehouse.usd",
    "Flat Plane": "Terrains/flat_plane.usd",
    "Rough Plane": "Terrains/rough_plane.usd",
    "Slope Plane": "Terrains/slope.usd",
    "Stairs Plane": "Terrains/stairs.usd",
}

# OMNIVERSE_ENVIRONMENTS = {
#     "Exhibition Hall": "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/NVIDIA/Assets/Scenes/Templates/Interior/ZetCG_ExhibitionHall.usd"
# }


SIMULATION_ENVIRONMENTS = {}

# Add the Isaac Sim assets to the list
for asset in NVIDIA_SIMULATION_ENVIRONMENTS:
    SIMULATION_ENVIRONMENTS[asset] = (
        NVIDIA_ASSETS_PATH + ISAAC_SIM_ENVIRONMENTS + "/" + NVIDIA_SIMULATION_ENVIRONMENTS[asset]
    )

# # Add the omniverse assets to the list
# for asset in OMNIVERSE_ENVIRONMENTS:
#     SIMULATION_ENVIRONMENTS[asset] = OMNIVERSE_ENVIRONMENTS[asset]

BACKENDS = {
    "px4": "px4",
    "ardupilot": "ardupilot",
    "ros2": "ros2"
}

# Define the default settings for the simulation environment
WORLD_SETTINGS = {
    'px4': {
        "physics_dt": 1.0 / 250.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
        "device": "cpu"
    },
    'ardupilot': {
        "physics_dt": 1.0 / 800.0, # Reach communication of 250hz with ardupilot sitl
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 100.0,
        "device": "cpu"
    },
    'ros2': {
        "physics_dt": 1.0 / 250.0,
        "stage_units_in_meters": 1.0,
        "rendering_dt": 1.0 / 60.0,
        "device": "cpu"
    }
}
DEFAULT_WORLD_SETTINGS = WORLD_SETTINGS['px4']

# Get the configurations file path
CONFIG_FILE = ROOT + "/skyrover.simulator/config/configs.yaml"

# Define where the thumbnail of the vehicle is located
THUMBNAIL = ROBOTS_ASSETS + "/Iris/iris_thumbnail.png"

# Define where the thumbail of the world is located
WORLD_THUMBNAIL = ASSET_PATH + "/Worlds/Empty_thumbnail.png"

# BACKENDS_THUMBMAILS_PATH = ASSET_PATH + "/Backends"
# BACKENDS_THUMBMAILS = {
#     "px4": BACKENDS_THUMBMAILS_PATH + "/px4_logo.png",
#     "ardupilot": BACKENDS_THUMBMAILS_PATH + "/ardupilot_logo.png",
#     "ros2": BACKENDS_THUMBMAILS_PATH + "/ros2_logo.png"
# }
