"""
| File: multirotor.py
| Author: Fei Wang (feiwang@dlmu.edu.cn)
| License: BSD-3-Clause. Copyright (c) 2025, Fei Wang. All rights reserved.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from isaacsim.core.utils.prims import get_prim_at_path

# The vehicle interface
from skyrover.simulator.core.vehicles.vehicle import Vehicle, get_world_transform_xform

# Mavlink interface
# from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig

from skyrover.simulator.core.config_yaml import ConfigYaml
from skyrover.simulator.impl.params import GROUND_ROBOT_CONFIG

# Sensors and dynamics setup
from skyrover.simulator.core.graphical_sensors import MonocularCamera
from skyrover.simulator.core.graphical_sensors import Lidar
from skyrover.simulator.core.dynamics import LinearDrag
from skyrover.simulator.core.thrusters import QuadraticThrustCurve
from skyrover.simulator.core.sensors import IMU, GPS

class MultirotorGroundConfig(ConfigYaml):
    def __init__(self, filename: str = GROUND_ROBOT_CONFIG):
        """Initialize the BackendConfig class
        """
        super().__init__(filename)

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = self.get("stage_prefix", "")

        # The USD file that describes the visual aspect of the vehicle (and some properties such as mass and moments of inertia)
        self.usd_file = self.get("usd_file", "")

        # The default thrust curve for a quadrotor and dynamics relating to drag

        # self.thrust_curve = QuadraticThrustCurve(self)

        # self.drag = LinearDrag([0.50, 0.30, 0.0])

        #self.drag = LinearDrag(self.get("drag", [0.50, 0.30, 0.0]))

        # The default sensors for a quadrotor
        self.sensors = []
        if self.get("use_imu", True):
            self.sensors.append(IMU())
        if self.get("use_gps", True):
            self.sensors.append(GPS())

        # The default graphical sensors for a quadrotor
        self.graphical_sensors = []
        if self.get("camera_list") is not None:
            for element in self.get("camera_list"):
                camera = MonocularCamera(element.get("camera_name"),config = element.get("camera_config"))
                self.graphical_sensors.append(camera)

        if self.get("lidar_list") is not None:
            for element in self.get("lidar_list"):
                lidar = Lidar(element.get("lidar_name"),config = element.get("lidar_config"))
                self.graphical_sensors.append(lidar)

        # The default omnigraphs for a quadrotor
        self.graphs = []

        # The backends for actually sending commands to the vehicle. By default use mavlink (with default mavlink configurations)
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle]. It can also be a ROS2 backend
        # or your own custom Backend implementation!
        # self.backends = [PX4MavlinkBackend(config=PX4MavlinkBackendConfig())]
        self.backends = []

    def test(self):
        """Test method to verify if the configuration is valid
        """
        print("stage_prefix:", self.stage_prefix)
        print("usd_file:", self.usd_file)
        # print("thrust_curve.rot_dir:", self.thrust_curve.rot_dir)
        # print("thrust_curve.rolling_moment:", self.thrust_curve.rolling_moment)
        # print("thrust_curve.velocity:", self.thrust_curve.velocity)
        # print("thrust_curve.force:", self.thrust_curve.force)
        # print("drag:", self.drag.drag)
        for sensor in self.sensors:
            print("sensor:", sensor.sensor_type)
            print("sensor update_rate:", sensor.update_rate)
        for sensor in self.graphical_sensors:
            print("graphical_sensors:", sensor.sensor_type)
            print("graphical_sensors update_rate:", sensor.update_rate)
        for backend in self.backends:
            print("backend:", type(backend))
            print("backend config:", backend.config.filename)


class MultirotorGround(Vehicle):
    """Multirotor class - It defines a base interface for creating a multirotor
    """
    def __init__(
        self,
        # Simulation specific configurations
        stage_prefix: str = "",
        usd_file: str = "",
        vehicle_id: int = 0,
        # Spawning pose of the vehicle
        init_pos=[0.0, 0.0, 0.07],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        config=MultirotorGroundConfig(),
    ):
        """Initializes the multirotor object

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "quadrotor".
            usd_file (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            vehicle_id (int): The id to be used for the vehicle. Defaults to 0.
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.07].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
            config (MultirotorConfig, optional): Defaults to MultirotorConfig().
        """

        # 1. Initiate the Vehicle object itself
        full_prefix = stage_prefix + str(vehicle_id)
        super().__init__(full_prefix, usd_file, init_pos, init_orientation, config.sensors, config.graphical_sensors, config.graphs, config.backends)

        prim = get_prim_at_path(self._stage_prefix + "/hunter_se_description")
        prim.GetAttribute("isaac:namespace").Set(config.get("namespace", "vehicle")  + str(vehicle_id))

        # 2. Setup the dynamics of the system - get the thrust curve of the vehicle from the configuration
        # self._thrusters = config.thrust_curve
        # self._drag = config.drag

    def start(self):
        """In this case we do not need to do anything extra when the simulation starts"""
        pass

    def stop(self):
        """In this case we do not need to do anything extra when the simulation stops"""
        pass

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in simulation based on the motor speed. 
        This method must be implemented by a class that inherits this type. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        
        # Call the update methods in all backends
        for backend in self._backends:
            backend.update(dt)


    """
    Operations
    """
    def update_state(self, dt: float):
        """
        Method that is called at every physics step to retrieve and update the current state of the vehicle, i.e., get
        the current position, orientation, linear and angular velocities and acceleration of the vehicle.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the body frame interface of the vehicle (this will be the frame used to get the position, orientation, etc.)
        body = self.get_dc_interface().get_rigid_body(self._stage_prefix + "/hunter_se_description/base_link")

        # Get the current position and orientation in the inertial frame
        pose = self.get_dc_interface().get_rigid_body_pose(body)

        # Get the attitude according to the convention [w, x, y, z]
        prim = self._world.stage.GetPrimAtPath(self._stage_prefix + "/hunter_se_description/base_link")
        rotation_quat = get_world_transform_xform(prim).GetQuaternion()
        rotation_quat_real = rotation_quat.GetReal()
        rotation_quat_img = rotation_quat.GetImaginary()

        # Get the angular velocity of the vehicle expressed in the body frame of reference
        ang_vel = self.get_dc_interface().get_rigid_body_angular_velocity(body)

        # The linear velocity [x_dot, y_dot, z_dot] of the vehicle's body frame expressed in the inertial frame of reference
        linear_vel = self.get_dc_interface().get_rigid_body_linear_velocity(body)

        # Get the linear acceleration of the body relative to the inertial frame, expressed in the inertial frame
        # Note: we must do this approximation, since the Isaac sim does not output the acceleration of the rigid body directly
        linear_acceleration = (np.array(linear_vel) - self._state.linear_velocity) / dt

        # Update the state variable X = [x,y,z]
        self._state.position = np.array(pose.p)

        # Get the quaternion according in the [qx,qy,qz,qw] standard
        self._state.attitude = np.array(
            [rotation_quat_img[0], rotation_quat_img[1], rotation_quat_img[2], rotation_quat_real]
        )

        # Express the velocity of the vehicle in the inertial frame X_dot = [x_dot, y_dot, z_dot]
        self._state.linear_velocity = np.array(linear_vel)

        # The linear velocity V =[u,v,w] of the vehicle's body frame expressed in the body frame of reference
        # Note that: x_dot = Rot * V
        self._state.linear_body_velocity = (
            Rotation.from_quat(self._state.attitude).inv().apply(self._state.linear_velocity)
        )

        # omega = [p,q,r]
        self._state.angular_velocity = Rotation.from_quat(self._state.attitude).inv().apply(np.array(ang_vel))

        # The acceleration of the vehicle expressed in the inertial frame X_ddot = [x_ddot, y_ddot, z_ddot]
        self._state.linear_acceleration = linear_acceleration

