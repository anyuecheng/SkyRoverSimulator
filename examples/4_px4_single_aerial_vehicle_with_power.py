#!/usr/bin/env python
"""
| File: multi_vehicle.py
| Author: Fei Wang (feiwang@dlmu.edu.cn)
| License: BSD-3-Clause. Copyright (c) 2025, Fei Wang. All rights reserved.
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from skyrover.simulator.impl.params import AERIAL_ROBOTS, SIMULATION_ENVIRONMENTS
from skyrover.simulator.core.state import State
from skyrover.simulator.core.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from skyrover.simulator.core.backends.ros2_multirotor_backend import ROS2MultiRotorAerialBackendConfig, ROS2MultiRotorBackend
from skyrover.simulator.core.vehicles.multirotor_aerial import MultirotorAerial, MultirotorAerialConfig
from skyrover.simulator.core.interface.skyrover_interface import SkyRoverInterface
# Auxiliary scipy and numpy modules
import os.path
from scipy.spatial.transform import Rotation


class SkyApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.sk = SkyRoverInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        self.sk._world = World(**self.sk._world_settings)
        self.world = self.sk.world

        # Launch one of the worlds provided by NVIDIA
        self.sk.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Spawn 5 vehicles with the PX4 control backend in the simulation, separated by 1.0 m along the x-axis
        for i in range(1):
            self.vehicle_factory(i, gap_x_axis=1.0)

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def vehicle_factory(self, vehicle_id: int, gap_x_axis: float):
        """Auxiliar method to create multiple multirotor vehicles

        Args:
            vehicle_id (_type_): _description_
        """

        config_multirotor = MultirotorAerialConfig()
        config_ros2 = ROS2MultiRotorAerialBackendConfig()
        mavlink_config = PX4MavlinkBackendConfig(vehicle_id)
        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
            ROS2MultiRotorBackend(vehicle_id = vehicle_id, 
                            config=config_ros2)
            ]
        '''
        test:
        ROS2MultiRotorBackend(vehicle_id=1, 
                            config={
                            "namespace": 'drone', 
                            "pub_sensors": False,
                            "pub_graphical_sensors": True,
                            "pub_state": True,
                            "sub_control": False,})
        '''
        # ros2_backend_config = ROS2MultiRotorAerialBackendConfig()
        # ros2_backend = ROS2MultiRotorBackend(vehicle_id=vehicle_id, config=ros2_backend_config)
        # config_multirotor.backends = [ros2_backend]
        
        

        #print(AERIAL_ROBOTS['Iris'])
        MultirotorAerial(
            "/World/quadrotor",
            AERIAL_ROBOTS['Iris'],
            vehicle_id,
            [gap_x_axis * vehicle_id, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor)

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    # Instantiate the template app
    sk_app = SkyApp()

    # Run the application loop
    sk_app.run()


if __name__ == "__main__":
    main()