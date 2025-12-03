"""
| File: ui_handler.py
| Author: Fei Wang (feiwang@dlmu.edu.cn)
| License: BSD-3-Clause. Copyright (c) 2025, Fei Wang. All rights reserved.
"""

# External packages
import os
import gc
import asyncio
from scipy.spatial.transform import Rotation

# Omniverse extensions
import carb
import omni.ui as ui

# Extension Configurations
from skyrover.simulator.impl.params import SIMULATION_ENVIRONMENTS, WORLD_SETTINGS, BACKENDS, AERIAL_ROBOTS, GROUND_ROBOTS
from skyrover.simulator.core.interface.skyrover_interface import SkyRoverInterface

# Vehicle Manager to spawn Vehicles
from skyrover.simulator.core.backends import Backend, BackendConfig, PX4MavlinkBackendConfig, PX4MavlinkBackend
from skyrover.simulator.core.vehicles.multirotor_aerial import MultirotorAerialConfig, MultirotorAerial
from skyrover.simulator.core.vehicles.multirotor_ground import MultirotorGroundConfig, MultirotorGround
# from skyrover.simulator.core.vehicle_manager import VehicleManager
from skyrover.simulator.core.graphical_sensors import MonocularCamera, Lidar

try:
    from skyrover.simulator.core.backends import ROS2MultiRotorBackend, ROS2MultiRotorAerialBackendConfig, ROS2MultiRotorGroundBackendConfig
    ROS2_available = True
except ImportError:
    ROS2_available = False
    carb.log_warn("ROS2 backend not available. Please install the ROS2 extension to use this feature.")


class UIHandler:
    """
    Object that will interface between the logic/dynamic simulation part of the extension and the Widget UI
    """

    def __init__(self):

        # The window that will be bound to this delegate
        self._window = None

        # Get an instance of the skyrover simulator
        self._skyrover_sim: SkyRoverInterface = SkyRoverInterface()

        # Attribute that holds the currently selected scene from the dropdown menu
        self._scene_dropdown: ui.AbstractItemModel = None
        self._scene_names = list(SIMULATION_ENVIRONMENTS.keys())

        # Selected latitude, longitude and altitude
        self._latitude_field: ui.AbstractValueModel = None
        self._latitude = SkyRoverInterface().latitude
        self._longitude_field: ui.AbstractValueModel = None
        self._longitude = SkyRoverInterface().longitude
        self._altitude_field: ui.AbstractValueModel = None
        self._altitude = SkyRoverInterface().altitude

        self._aerial_vehicle_dropdown: ui.AbstractItemModel = None
        self._aerial_vehicles_names = list(AERIAL_ROBOTS.keys())
        self._aerial_vehicle_num_field: ui.AbstractValueModel = None
        self._aerial_vehicle_num: int = 0

        self._ground_vehicle_dropdown: ui.AbstractItemModel = None
        self._ground_vehicles_names = list(GROUND_ROBOTS.keys())
        self._ground_vehicle_num_field: ui.AbstractValueModel = None
        self._ground_vehicle_num: int = 0

        # # Get an instance of the vehicle manager
        # self._vehicle_manager = VehicleManager()

        # Selected option for broadcasting the simulated vehicle (PX4+ROS2 or just ROS2)
        # By default we assume ROS2
        self._aerial_backend: str = BACKENDS['ros2']      
        self._ground_backend: str = BACKENDS['ros2']

        self._vehicle_id: int = 0

#         # Attribute that will save the model for the px4-autostart checkbox
#         self._px4_autostart_checkbox: ui.AbstractValueModel = None
#         self._autostart_px4: bool = True

#         # Atributes to store the path for the Px4 directory
#         self._px4_directory_field: ui.AbstractValueModel = None
#         self._px4_dir: str = PegasusInterface().px4_path

#         # Atributes to store the PX4 airframe
#         self._px4_airframe_field: ui.AbstractValueModel = None
#         self._px4_airframe: str = self._pegasus_sim.px4_default_airframe

#         # Attribute that will save the model for the ardupilot-autostart checkbox
#         self._ardupilot_autostart_checkbox: ui.AbstractValueModel = None
#         self._autostart_ardupilot: bool = True

#         # Atributes to store the path for the ArduPilot directory
#         self._ardupilot_directory_field: ui.AbstractValueModel = None
#         self._ardupilot_dir: str = PegasusInterface().ardupilot_path

#         # Atributes to store the ArduPilot airframe
#         self._ardupilot_airframe_field: ui.AbstractValueModel = None
#         self._ardupilot_airframe: str = self._pegasus_sim.ardupilot_default_airframe

    def set_window_bind(self, window):
        self._window = window

    def set_scene_dropdown(self, scene_dropdown_model: ui.AbstractItemModel):
        self._scene_dropdown = scene_dropdown_model
    
    def set_latitude_field(self, latitude_model: ui.AbstractValueModel):
        self._latitude_field = latitude_model
    
    def set_longitude_field(self, longitude_model: ui.AbstractValueModel):
        self._longitude_field = longitude_model

    def set_altitude_field(self, altitude_model: ui.AbstractValueModel):
        self._altitude_field = altitude_model

    def set_aerial_vehicle_dropdown(self, vehicle_dropdown_model: ui.AbstractItemModel):
        self._aerial_vehicle_dropdown = vehicle_dropdown_model

    def set_aerial_vehicle_backend_dropdown(self, vehicle_backend_model: ui.AbstractItemModel):
        self._aerial_vehicle_backend_dropdown = vehicle_backend_model

    def set_aerial_vehicle_num_field(self, vehicle_num_field: ui.AbstractValueModel):
        self._aerial_vehicle_num_field = vehicle_num_field

    def set_aerial_vehicle_spawn_axis(self, spawn_axis_model: ui.AbstractItemModel):
        self._aerial_vehicle_spawn_axis = spawn_axis_model

    def set_aerial_vehicle_spawn_distance(self, spawn_distance_field: ui.AbstractValueModel):
        self._aerial_vehicle_spawn_distance = spawn_distance_field

    def set_ground_vehicle_dropdown(self, vehicle_dropdown_model: ui.AbstractItemModel):
        self._ground_vehicle_dropdown = vehicle_dropdown_model

    def set_ground_vehicle_backend_dropdown(self, vehicle_backend_model: ui.AbstractItemModel):
        self._ground_vehicle_backend_dropdown = vehicle_backend_model

    def set_ground_vehicle_num_field(self, vehicle_num_field: ui.AbstractValueModel):
        self._ground_vehicle_num_field = vehicle_num_field

    def set_ground_vehicle_spawn_axis(self, spawn_axis_model: ui.AbstractItemModel):
        self._ground_vehicle_spawn_axis = spawn_axis_model

    def set_ground_vehicle_spawn_distance(self, spawn_distance_field: ui.AbstractValueModel):
        self._ground_vehicle_spawn_distance = spawn_distance_field

    def set_aerial_backend(self, backend: str = BACKENDS['ros2']):
        self._aerial_backend = backend

    def set_ground_backend(self, backend: str = BACKENDS['ros2']):
        self._ground_backend = backend

#     def set_px4_autostart_checkbox(self, checkbox_model:ui.AbstractValueModel):
#         self._px4_autostart_checkbox = checkbox_model

#     def set_px4_directory_field(self, directory_field_model: ui.AbstractValueModel):
#         self._px4_directory_field = directory_field_model

#     def set_px4_airframe_field(self, airframe_field_model: ui.AbstractValueModel):
#         self._px4_airframe_field = airframe_field_model
    
#     def set_ardupilot_autostart_checkbox(self, checkbox_model: ui.AbstractValueModel):
#         self._ardupilot_autostart_checkbox = checkbox_model

#     def set_ardupilot_directory_field(self, directory_field_model: ui.AbstractValueModel):
#         self._ardupilot_directory_field = directory_field_model

#     def set_ardupilot_airframe_field(self, airframe_field_model: ui.AbstractValueModel):
#         self._ardupilot_airframe_field = airframe_field_model

#     """
#     ---------------------------------------------------------------------
#     Callbacks to handle user interaction with the extension widget window
#     ---------------------------------------------------------------------
#     """

    def on_set_new_global_coordinates(self):
        self._skyrover_sim.set_global_coordinates(
            self._latitude_field.get_value_as_float(),
            self._longitude_field.get_value_as_float(),
            self._altitude_field.get_value_as_float())
        

    def on_reset_global_coordinates(self):
        self._skyrover_sim.set_default_global_coordinates()

        self._latitude_field.set_value(self._skyrover_sim.latitude)
        self._longitude_field.set_value(self._skyrover_sim.longitude)
        self._altitude_field.set_value(self._skyrover_sim.altitude)


    def on_set_new_default_global_coordinates(self):
        self._skyrover_sim.set_new_default_global_coordinates(
            self._latitude_field.get_value_as_float(),
            self._longitude_field.get_value_as_float(),
            self._altitude_field.get_value_as_float()
        )


    def on_load_scene(self):
        # Check if a scene is selected in the drop-down menu
        if self._scene_dropdown is not None:

            # Get the id of the selected environment from the list
            environemnt_index = self._scene_dropdown.get_item_value_model().as_int

            # Get the name of the selected world
            selected_world = self._scene_names[environemnt_index]
            
            # Try to spawn the selected world
            self._skyrover_sim.set_world_settings(**WORLD_SETTINGS[self._aerial_backend])
            asyncio.ensure_future(self._skyrover_sim.load_environment_async(SIMULATION_ENVIRONMENTS[selected_world], force_clear=True))
            print("Loading scene: " + selected_world + " from path: " + SIMULATION_ENVIRONMENTS[selected_world])


    def on_clear_scene(self):
        """
        Method that should be invoked when the clear world button is pressed
        """
        self._skyrover_sim.clear_scene()
        self._vehicle_id = 0
        print("Scene cleared.")


    def on_load_vehicle(self):
        """
        Method that should be invoked when the button to load the selected vehicle is pressed
        """
        async def async_load_vehicle():
            # Check if we already have a physics environment activated. If not, then activate it
            # and only after spawn the vehicle. This is to avoid trying to spawn a vehicle without a physics
            # environment setup. This way we can even spawn a vehicle in an empty world and it won't care
            if hasattr(self._skyrover_sim.world, "_physics_context") == False:
                if self._skyrover_sim.world is None:
                    self._skyrover_sim.initialize_world()
                await self._skyrover_sim.world.initialize_simulation_context_async()

            # Check if a vehicle is selected in the drop-down menu
            if self._window is not None and self._aerial_vehicle_dropdown is not None and self._ground_vehicle_dropdown is not None:
                aerial_vehicle_index = self._aerial_vehicle_dropdown.get_item_value_model().as_int
                ground_vehicle_index = self._ground_vehicle_dropdown.get_item_value_model().as_int

                # Get the name of the selected vehicle
                selected_aerial_robot = self._aerial_vehicles_names[aerial_vehicle_index]
                selected_ground_robot = self._ground_vehicles_names[ground_vehicle_index]

                self._aerial_vehicle_num = self._aerial_vehicle_num_field.get_value_as_int()
                self._ground_vehicle_num = self._ground_vehicle_num_field.get_value_as_int()

                # Get the desired position and orientation of the vehicle from the UI transform
                aerial_position, aerial_oritation = self._window.get_selected_aerial_pos_ori()
                ground_position, ground_oritation = self._window.get_selected_ground_pos_ori()

                aerial_spawn_axis = self._aerial_vehicle_spawn_axis.get_item_value_model().as_int
                ground_spawn_axis = self._ground_vehicle_spawn_axis.get_item_value_model().as_int

                aerial_spawn_distance = self._aerial_vehicle_spawn_distance.get_value_as_float()
                ground_spawn_distance = self._ground_vehicle_spawn_distance.get_value_as_float()

                aerial_backend_index = self._aerial_vehicle_backend_dropdown.get_item_value_model().as_int
                ground_backend_index = self._ground_vehicle_backend_dropdown.get_item_value_model().as_int
                backends_names = list(BACKENDS.keys())
                self._aerial_backend = BACKENDS[backends_names[aerial_backend_index]]
                self._ground_backend = BACKENDS[backends_names[ground_backend_index]]

                for i in range(self._aerial_vehicle_num):
                    self._vehicle_id += 1

                    backend_config: BackendConfig = None
                    backend: Backend = None

                    if self._aerial_backend == BACKENDS["ros2"]:  
                        if ROS2_available:
                            backend_config = ROS2MultiRotorAerialBackendConfig()
                            backend = ROS2MultiRotorBackend(vehicle_id=self._vehicle_id, config=backend_config)
                            carb.log_warn("ROS2 backend selected for aerial vehicle.")
                        else:
                            carb.log_warn("ROS2 not available. Please run Isaac Sim with ROS 2 extension correctly enabled.")
                            return
                    elif self._aerial_backend == BACKENDS["px4"]:  
                        backend_config = PX4MavlinkBackendConfig(vehicle_id=self._vehicle_id)
                        backend = PX4MavlinkBackend(config=backend_config)
                        carb.log_warn("PX4 backend selected.")
                    else:
                        carb.log_warn("Invalid backend selected. Not spawning the vehicle.")
                        return
                    

                    # Create the multirotor configuration
                    config_multirotor = MultirotorAerialConfig()
                    config_multirotor.stage_prefix = "quadrotor"
                    config_multirotor.backends = [backend]
                    # config_multirotor.graphical_sensors = [MonocularCamera("camera", config={"frequency": 30.0}), 
                    #                                        Lidar("lidar", config={"frequency": 10.0, 
                    #                                                               "sensor_configuration": "OS1_REV6_32ch10hz2048res"})]

                    config_multirotor.test()

                    # Try to spawn the selected robot in the world to the specified namespace
                    MultirotorAerial(
                        "/World/quadrotor",
                        AERIAL_ROBOTS[selected_aerial_robot],
                        self._vehicle_id,
                        aerial_position,
                        Rotation.from_euler("XYZ", aerial_oritation, degrees=True).as_quat(),
                        config=config_multirotor,
                    )

                    print("Spawning aerial vehicle: " + selected_aerial_robot)
                    print("Spawning aerial vehicle with backend: " + self._aerial_backend)
                    print("Spawning aerial vehicle number: " + str(self._aerial_vehicle_num))
                    print("at position: " + str(aerial_position) + " and orientation: " + str(aerial_oritation))

                    aerial_position[aerial_spawn_axis] += aerial_spawn_distance

                for i in range(self._ground_vehicle_num):
                    self._vehicle_id += 1

                    backend_config: BackendConfig = None
                    backend: Backend = None

                    if self._ground_backend == BACKENDS["ros2"]:  
                        if ROS2_available:
                            backend_config = ROS2MultiRotorGroundBackendConfig()
                            backend = ROS2MultiRotorBackend(vehicle_id=self._vehicle_id, config=backend_config)
                            carb.log_warn("ROS2 backend selected for ground vehicle.")
                        else:
                            carb.log_warn("ROS2 not available. Please run Isaac Sim with ROS 2 extension correctly enabled.")
                            return
                    else:
                        carb.log_warn("Invalid backend selected. Not spawning the vehicle.")
                        return
                
                    # Create the multirotor configuration
                    config_multirotor = MultirotorGroundConfig()
                    config_multirotor.stage_prefix = "hunter_se_description"
                    config_multirotor.backends = [backend]

                    config_multirotor.test()

                    # Try to spawn the selected robot in the world to the specified namespace
                    MultirotorGround(
                        "/World/Root",
                        GROUND_ROBOTS[selected_ground_robot],
                        self._vehicle_id,
                        ground_position,
                        Rotation.from_euler("XYZ", ground_oritation, degrees=True).as_quat(),
                        config=config_multirotor,
                    )

                    print("Spawning ground vehicle: " + selected_ground_robot)
                    print("Spawning ground vehicle with backend: " + self._ground_backend)
                    print("Spawning ground vehicle number: " + str(self._ground_vehicle_num))
                    print("at position: " + str(ground_position) + " and orientation: " + str(ground_oritation))

                    ground_position[ground_spawn_axis] += ground_spawn_distance


                    
                carb.log_info("Spawned the " + str(self._aerial_vehicle_num) + " robots: " + selected_aerial_robot)
                carb.log_info("Spawned the " + str(self._ground_vehicle_num) + " robots: " + selected_ground_robot)  
            else:
                carb.log_error("Could not spawn the robot using the SkyRover Simulator UI")

        # Run the actual vehicle spawn async so that the UI does not freeze
        asyncio.ensure_future(async_load_vehicle())        

    def on_set_viewport_camera(self):
        """
        Method that should be invoked when the button to set the viewport camera pose is pressed
        """
        carb.log_warn("The viewport camera pose has been adjusted")

        if self._window:

            # Get the current camera position value
            camera_position, camera_target = self._window.get_selected_camera_pos()

            if camera_position is not None and camera_target is not None:

                # Set the camera view to a fixed value
                self._skyrover_sim.set_viewport_camera(eye=camera_position, target=camera_target)
    
#     def on_set_new_default_px4_path(self):
#         """
#         Method that will try to update the new PX4 autopilot path with whatever is passed on the string field
#         """
#         carb.log_warn("A new default PX4 Path will be set for the extension.")

#         # Read the current path from the field
#         path = self._px4_directory_field.get_value_as_string()

#         # Set the path using the pegasus interface
#         self._pegasus_sim.set_px4_path(path)

#     def on_reset_px4_path(self):
#         """
#         Method that will reset the string field to the default PX4 path
#         """
#         carb.log_warn("Reseting the path to the default one")
#         self._px4_directory_field.set_value(self._pegasus_sim.px4_path)

#     def on_set_new_default_ardupilot_path(self):
#         """
#         Method that will try to update the new ArduPilot autopilot path with whatever is passed on the string field
#         """
#         carb.log_warn("A new default ArduPilot Path will be set for the extension.")

#         # Read the current path from the field
#         path = self._ardupilot_directory_field.get_value_as_string()

#         # Set the path using the pegasus interface
#         self._pegasus_sim.set_ardupilot_path(path)

#     def on_reset_ardupilot_path(self):
#         """
#         Method that will reset the string field to the default ArduPilot path
#         """
#         carb.log_warn("Reseting the path to the default one")
#         self._ardupilot_directory_field.set_value(self._pegasus_sim.ardupilot_path)
