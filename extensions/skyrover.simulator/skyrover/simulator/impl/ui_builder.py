"""
| File: ui_builder.py
| Author: Fei Wang (feiwang@dlmu.edu.cn)
| License: BSD-3-Clause. Copyright (c) 2025, Fei Wang. All rights reserved.
"""

"""
Omniverse UI Framework:
  https://docs.omniverse.nvidia.com/kit/docs/omni.ui/latest/Overview.html

Isaac Sim UI Utilities extension:
  https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.ui/docs/index.html
"""

import numpy as np

import omni.kit.ui
import omni.ui as ui
from omni.ui import color as cl

from skyrover.simulator.impl.ui_handler import UIHandler
from skyrover.simulator.impl.params import WINDOW_TITLE, SIMULATION_ENVIRONMENTS, WORLD_THUMBNAIL, THUMBNAIL, AERIAL_ROBOTS, GROUND_ROBOTS

class SkyRoverWindow(ui.Window):
    """Manage extension UI"""
    WINDOW_WIDTH = 325
    WINDOW_HEIGHT = 850

    LABEL_PADDING = 120
    BUTTON_HEIGHT = 50
    GENERAL_SPACING = 5

    BUTTON_SELECTED_STYLE = {
        "Button": {
            "background_color": cl("#3780ae"),
            "border_color": cl("#29587c"),
            "border_width": 2,
            "border_radius": 5,
            "padding": 2,
        }
    }

    BUTTON_BASE_STYLE = {
        "Button": {
            "background_color": cl("#292929"),
            "border_color": cl("#292929"),
            "border_width": 2,
            "border_radius": 5,
            "padding": 5,
        }
    }


    def __init__(self, handler: UIHandler, **kwargs):
        # Setup the base widget window
        super().__init__(
            WINDOW_TITLE, width=SkyRoverWindow.WINDOW_WIDTH, height=SkyRoverWindow.WINDOW_HEIGHT, visible=True, **kwargs
        )

        self.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

        # Setup the handler 
        self._handler = handler
        # Bind the UI delegate to this window
        self._handler.set_window_bind(self)

        # Auxiliar attributes for getting the transforms of the vehicle and the camera from the UI
        self._camera_transform_models = []
        self._aerial_vehicle_transform_models = []
        self._ground_vehicle_transform_models = []

        # Build the actual window UI
        self._build_ui()


    def destroy(self):
        # Clear the world and the stage correctly
        self._handler.on_clear_scene()

        # It will destroy all the children
        super().destroy()


    def _build_ui(self):
        # Define the UI of the widget window
        with self.frame:
        
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON, vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON):

                # Vertical Stack of menus
                with ui.VStack():
                    # # Create a frame for selecting which backend to load
                    # self._backend_selection_frame()
                    # ui.Spacer(height=5)

                    # Create a frame for selecting which scene to load
                    self._scene_selection_frame()
                    ui.Spacer(height=5)
                    
                    # Create a frame for selecting which vehicle to load in the simulation environment
                    self._robot_selection_frame()
                    ui.Spacer(height=5)

                    # Create a frame for selecting the camera position, and what it should point torwards to
                    self._viewport_camera_frame()
                    ui.Spacer()

                    self._button = ui.Button("Click me", clicked_fn=lambda: print("Button clicked"))


    def _scene_selection_frame(self):
        """
        Method that implements a dropdown menu with the list of available simulation environemts for the vehicle
        """
        # Frame for selecting the simulation environment to load
        with ui.CollapsableFrame("Scene Selection"):
            with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                ui.Spacer(height=SkyRoverWindow.GENERAL_SPACING)

                # Iterate over all existing pre-made worlds bundled with this extension
                with ui.HStack():
                    ui.Label("World Assets", width=SkyRoverWindow.LABEL_PADDING, height=10.0)

                    # Combo box with the available environments to select from
                    dropdown_menu = ui.ComboBox(0, height=10, name="environments")
                    for environment in SIMULATION_ENVIRONMENTS:
                        dropdown_menu.model.append_child_item(None, ui.SimpleStringModel(environment))

                    # Allow the handler to know which option was selected in the dropdown menu
                    self._handler.set_scene_dropdown(dropdown_menu.model)

                ui.Spacer(height=0)

                # UI to configure the default latitude, longitude and altitude coordinates
                with ui.CollapsableFrame("Geographic Coordinates", collapsed=False):
                    with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                        with ui.HStack():

                            # Latitude
                            ui.Label("Latitude", name="label", width=SkyRoverWindow.LABEL_PADDING-50)
                            latitude_field = ui.FloatField(name="latitude", precision=6)
                            latitude_field.model.set_value(self._handler._latitude)
                            self._handler.set_latitude_field(latitude_field.model)
                            ui.Circle(name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED)

                            # Longitude
                            ui.Label("Longitude", name="label", width=SkyRoverWindow.LABEL_PADDING-50)
                            longitude_field = ui.FloatField(name="longitude", precision=6)
                            longitude_field.model.set_value(self._handler._longitude)
                            self._handler.set_longitude_field(longitude_field.model)
                            ui.Circle(name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED)

                            # Altitude
                            ui.Label("Altitude", name="label", width=SkyRoverWindow.LABEL_PADDING-50)
                            altitude_field = ui.FloatField(name="altitude", precision=6)
                            altitude_field.model.set_value(self._handler._altitude)
                            self._handler.set_altitude_field(altitude_field.model)
                            ui.Circle(name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED)

                        with ui.HStack():
                            ui.Button("Set", enabled=True, clicked_fn=self._handler.on_set_new_global_coordinates)
                            ui.Button("Reset", enabled=True, clicked_fn=self._handler.on_reset_global_coordinates)
                            ui.Button("Make Default", enabled=True, clicked_fn=self._handler.on_set_new_default_global_coordinates)

                ui.Spacer(height=0)

                with ui.HStack():
                    # Add a thumbnail image to have a preview of the world that is about to be loaded
                    with ui.ZStack(width=SkyRoverWindow.LABEL_PADDING, height=SkyRoverWindow.BUTTON_HEIGHT * 2):
                        ui.Rectangle()
                        ui.Image(
                            WORLD_THUMBNAIL,
                            fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                            alignment=ui.Alignment.LEFT_CENTER,
                        )

                    ui.Spacer(width=SkyRoverWindow.GENERAL_SPACING)

                    with ui.VStack():
                        # Button for loading a desired scene
                        ui.Button(
                            "Load Scene",
                            height=SkyRoverWindow.BUTTON_HEIGHT,
                            clicked_fn=self._handler.on_load_scene,
                            style=SkyRoverWindow.BUTTON_BASE_STYLE,
                        )

                        # Button to reset the stage
                        ui.Button(
                            "Clear Scene",
                            height=SkyRoverWindow.BUTTON_HEIGHT,
                            clicked_fn=self._handler.on_clear_scene,
                            style=SkyRoverWindow.BUTTON_BASE_STYLE,
                        )


    def _robot_selection_frame(self):
        """
        Method that implements a frame that allows the user to choose which robot that is about to be spawned
        """
        # Frame for selecting the vehicle to load
        with ui.CollapsableFrame(title="Vehicle Selection"):
            with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                ui.Spacer(height=SkyRoverWindow.GENERAL_SPACING)

                with ui.CollapsableFrame("Aerial Vehicle Selection", collapsed=False):
                    with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                        with ui.HStack():
                            # Add a thumbnail image to have a preview of the world that is about to be loaded
                            with ui.ZStack(width=SkyRoverWindow.LABEL_PADDING, height=SkyRoverWindow.BUTTON_HEIGHT * 2):
                                ui.Rectangle()
                                ui.Image(
                                    THUMBNAIL,
                                    fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                    alignment=ui.Alignment.CENTER,
                                )
                            ui.Spacer(width=10)
                    
                            with ui.VStack():
                                with ui.HStack():
                                    # Iterate over all existing robots in the extension
                                    ui.Label("Vehicle Model", name="label", width=SkyRoverWindow.LABEL_PADDING, alignment=ui.Alignment.TOP)

                                    # Combo box with the available vehicles to select from
                                    dropdown_menu = ui.ComboBox(0, name="robots")
                                    for robot in AERIAL_ROBOTS:
                                        dropdown_menu.model.append_child_item(None, ui.SimpleStringModel(robot))
                                    self._handler.set_aerial_vehicle_dropdown(dropdown_menu.model)

                                with ui.HStack():
                                    ui.Label("Vehicle Number", name="label", width=SkyRoverWindow.LABEL_PADDING, alignment=ui.Alignment.TOP)
                                    vehicle_num_field = ui.IntDrag(name="number", min=0, max=1000, step=0.05)
                                    self._handler.set_aerial_vehicle_num_field(vehicle_num_field.model)
                        with ui.HStack():
                            # Add a frame transform to select the position of where to place the selected robot in the world
                            self._transform_frame(isAerial=True)

                with ui.CollapsableFrame("Ground Vehicle Selection", collapsed=False):
                    with ui.VStack(height=0, spacing=10, name="frame_v_stack"):
                        with ui.HStack():
                            # Add a thumbnail image to have a preview of the world that is about to be loaded
                            with ui.ZStack(width=SkyRoverWindow.LABEL_PADDING, height=SkyRoverWindow.BUTTON_HEIGHT * 2):
                                ui.Rectangle()
                                ui.Image(
                                    THUMBNAIL,
                                    fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                                    alignment=ui.Alignment.CENTER,
                                )
                            ui.Spacer(width=10)
                    
                            with ui.VStack():
                                with ui.HStack():
                                    # Iterate over all existing robots in the extension
                                    ui.Label("Vehicle Model", name="label", width=SkyRoverWindow.LABEL_PADDING, alignment=ui.Alignment.TOP)

                                    # Combo box with the available vehicles to select from
                                    dropdown_menu = ui.ComboBox(0, name="robots")
                                    for robot in GROUND_ROBOTS:
                                        dropdown_menu.model.append_child_item(None, ui.SimpleStringModel(robot))
                                    self._handler.set_ground_vehicle_dropdown(dropdown_menu.model)

                                with ui.HStack():
                                    ui.Label("Vehicle Number", name="label", width=SkyRoverWindow.LABEL_PADDING, alignment=ui.Alignment.TOP)
                                    vehicle_num_field = ui.IntDrag(name="number", min=0, max=1000, step=0.05)
                                    self._handler.set_ground_vehicle_num_field(vehicle_num_field.model)
                        with ui.HStack():
                            # Add a frame transform to select the position of where to place the selected robot in the world
                            self._transform_frame(isAerial=False)

                # with ui.HStack():
                #     # Add a frame transform to select the position of where to place the selected robot in the world
                #     self._transform_frame()
                
                # Button to load the drone
                ui.Button(
                    "Load Vehicle",
                    height=SkyRoverWindow.BUTTON_HEIGHT,
                    clicked_fn=self._handler.on_load_vehicle,
                    style=SkyRoverWindow.BUTTON_BASE_STYLE,
                )


    def _transform_frame(self, isAerial: bool=True):
        """
        Method that implements a transform frame to translate and rotate an object that is about to be spawned
        """
        components = ["Position", "Rotation"]
        all_axis = ["X", "Y", "Z"]
        colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
        default_values = [0.0, 0.0, 0.1]

        with ui.CollapsableFrame("Position and Orientation"):
            with ui.VStack(spacing=8):

                ui.Spacer(height=0)

                # Iterate over the position and rotation menus
                for component in components:
                    with ui.HStack():
                        with ui.HStack():
                            ui.Label(component, name="transform", width=50)
                            ui.Spacer()
                        # Fields X, Y and Z
                        for axis, default_value in zip(all_axis, default_values):
                            with ui.HStack():
                                with ui.ZStack(width=15):
                                    ui.Rectangle(
                                        width=15,
                                        height=20,
                                        style={
                                            "background_color": colors[axis],
                                            "border_radius": 3,
                                            "corner_flag": ui.CornerFlag.LEFT,
                                        },
                                    )
                                    ui.Label(axis, name="transform_label", alignment=ui.Alignment.CENTER)
                                if component == "Position":
                                    float_drag = ui.FloatDrag(name="transform", min=-1000000, max=1000000, step=0.01)
                                    float_drag.model.set_value(default_value)
                                else:
                                    float_drag = ui.FloatDrag(name="transform", min=-180.0, max=180.0, step=0.1)
                                # Save the model of each FloatDrag such that we can access its values later on
                                if isAerial:
                                    self._aerial_vehicle_transform_models.append(float_drag.model)
                                else:
                                    self._ground_vehicle_transform_models.append(float_drag.model)
                                ui.Circle(name="transform", width=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED)
                ui.Spacer(height=0)


    def _viewport_camera_frame(self):
        all_axis = ["X", "Y", "Z"]
        colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
        default_values = [5.0, 5.0, 5.0]
        target_default_values = [0.0, 0.0, 0.0]

        # Frame for setting the camera to visualize the vehicle in the simulator viewport
        with ui.CollapsableFrame("Viewport Camera"):
            with ui.VStack(spacing=8):
                ui.Spacer(height=0)

                # Iterate over the position and rotation menus
                with ui.HStack():
                    with ui.HStack():
                        ui.Label("Position", name="transform", width=50, height=20)
                        ui.Spacer()
                    # Fields X, Y and Z
                    for axis, default_value in zip(all_axis, default_values):
                        with ui.HStack():
                            with ui.ZStack(width=15):
                                ui.Rectangle(
                                    width=15,
                                    height=20,
                                    style={
                                        "background_color": colors[axis],
                                        "border_radius": 3,
                                        "corner_flag": ui.CornerFlag.LEFT,
                                    },
                                )
                                ui.Label(axis, height=20, name="transform_label", alignment=ui.Alignment.CENTER)
                            float_drag = ui.FloatDrag(name="transform", min=-1000000, max=1000000, step=0.01)
                            float_drag.model.set_value(default_value)
                            # Save the model of each FloatDrag such that we can access its values later on
                            self._camera_transform_models.append(float_drag.model)
                            ui.Circle(
                                name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED
                            )

                # Iterate over the position and rotation menus
                with ui.HStack():
                    with ui.HStack():
                        ui.Label("Target", name="transform", width=50, height=20)
                        ui.Spacer()
                    # Fields X, Y and Z
                    for axis, default_value in zip(all_axis, target_default_values):
                        with ui.HStack():
                            with ui.ZStack(width=15):
                                ui.Rectangle(
                                    width=15,
                                    height=20,
                                    style={
                                        "background_color": colors[axis],
                                        "border_radius": 3,
                                        "corner_flag": ui.CornerFlag.LEFT,
                                    },
                                )
                                ui.Label(axis, height=20, name="transform_label", alignment=ui.Alignment.CENTER)
                            float_drag = ui.FloatDrag(name="transform", min=-1000000, max=1000000, step=0.01)
                            float_drag.model.set_value(default_value)
                            # Save the model of each FloatDrag such that we can access its values later on
                            self._camera_transform_models.append(float_drag.model)
                            ui.Circle(
                                name="transform", width=20, height=20, radius=3.5, size_policy=ui.CircleSizePolicy.FIXED
                            )

                # Button to set the camera view
                ui.Button(
                    "Set Camera Pose",
                    height=SkyRoverWindow.BUTTON_HEIGHT,
                    clicked_fn=self._handler.on_set_viewport_camera,
                    style=SkyRoverWindow.BUTTON_BASE_STYLE,
                )
                ui.Spacer()


    def get_selected_camera_pos(self):
        """
        Method that returns the currently selected camera position in the camera transform widget
        """

        # Extract the camera desired position and the target it is pointing to
        if len(self._camera_transform_models) == 6:
            camera_pos = np.array([self._camera_transform_models[i].get_value_as_float() for i in range(3)])
            camera_target = np.array([self._camera_transform_models[i].get_value_as_float() for i in range(3, 6)])
            return camera_pos, camera_target

        return None, None
    

    def get_selected_aerial_pos_ori(self):
        # Extract the desired position and rotation
        if len(self._aerial_vehicle_transform_models) == 6:
            aerial_pos = np.array([self._aerial_vehicle_transform_models[i].get_value_as_float() for i in range(3)])
            aerial_ori = np.array([self._aerial_vehicle_transform_models[i].get_value_as_float() for i in range(3, 6)])
            return aerial_pos, aerial_ori

        return None, None
    

    def get_selected_ground_pos_ori(self):
        # Extract the desired position and rotation
        if len(self._ground_vehicle_transform_models) == 6:
            ground_pos = np.array([self._ground_vehicle_transform_models[i].get_value_as_float() for i in range(3)])
            ground_ori = np.array([self._ground_vehicle_transform_models[i].get_value_as_float() for i in range(3, 6)])
            return ground_pos, ground_ori

        return None, None
    