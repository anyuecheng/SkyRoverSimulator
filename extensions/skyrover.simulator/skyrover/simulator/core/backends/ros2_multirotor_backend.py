"""
| File: ros2_backend.py
| Author: Fei Wang (feiwang@dlmu.edu.cn)
| License: BSD-3-Clause. Copyright (c) 2025, Fei Wang. All rights reserved.
"""

import os

# Make sure the ROS2 extension is enabled
import carb
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

# ROS2 imports
import rclpy
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped

# TF imports
# Check if these libraries exist in the system
try:
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from tf2_ros.transform_broadcaster import TransformBroadcaster
    tf2_ros_loaded = True
except ImportError:
    carb.log_warn("TF2 ROS not installed. Will not publish TFs with the ROS2 backend")
    tf2_ros_loaded = False

# Import the replicatore core module used for writing graphical data to ROS 2
import omni
import omni.graph.core as og
import omni.replicator.core as rep
from isaacsim.ros2.bridge import read_camera_info

from skyrover.simulator.core.backends.backend import Backend, BackendConfig
from skyrover.simulator.impl.params import BACKEND_CONFIG_PATH


class ROS2MultiRotorBackendConfig(BackendConfig):
    def __init__(self, config_file: str = None):
        if config_file is None:
            config_file = os.path.join(BACKEND_CONFIG_PATH, "ros2_multirotor_backend_config.yaml")
        super().__init__(config_file)

        self.set("num_rotors", 4)
        self.set("namespace", "vehicle")

        self.set("pub_state", True)
        self.set("pub_pose", True)
        self.set("pub_twist", True)
        self.set("pub_accel", True)
        self.set("pub_twist_inertial", True)

        self.set("pub_sensors", True)
        self.set("pub_imu", True)
        self.set("pub_mag", True)
        self.set("pub_gps", True)
        self.set("pub_gps_vel", True)

        self.set("pub_graphical_sensors", True)
        self.set("pub_tf", False)
        self.set("sub_control", True)

        self.set("pose_topic", "state/pose")
        self.set("twist_topic", "state/twist")
        self.set("accel_topic", "state/accel")
        self.set("twist_inertial_topic", "state/twist_inertial")
        self.set("imu_topic", "sensors/imu")
        self.set("mag_topic", "sensors/mag")
        self.set("gps_topic", "sensors/gps")
        self.set("gps_vel_topic", "sensors/gps_vel")


class ROS2MultiRotorAerialBackendConfig(BackendConfig):
    def __init__(self, config_file: str = None):
        if config_file is None:
            config_file = os.path.join(BACKEND_CONFIG_PATH, "ros2_multirotor_aerial_backend_config.yaml")
        super().__init__(config_file)

        self.set("num_rotors", 4)
        self.set("namespace", "vehicle")

        self.set("pub_state", True)
        self.set("pub_pose", True)
        self.set("pub_twist", True)
        self.set("pub_accel", True)
        self.set("pub_twist_inertial", True)

        self.set("pub_sensors", True)
        self.set("pub_imu", True)
        self.set("pub_mag", True)
        self.set("pub_gps", True)
        self.set("pub_gps_vel", True)

        self.set("pub_graphical_sensors", True)
        self.set("pub_tf", False)
        self.set("sub_control", True)

        self.set("pose_topic", "state/pose")
        self.set("twist_topic", "state/twist")
        self.set("accel_topic", "state/accel")
        self.set("twist_inertial_topic", "state/twist_inertial")
        self.set("imu_topic", "sensors/imu")
        self.set("mag_topic", "sensors/mag")
        self.set("gps_topic", "sensors/gps")
        self.set("gps_vel_topic", "sensors/gps_vel")


class ROS2MultiRotorGroundBackendConfig(BackendConfig):
    def __init__(self, config_file: str = None):
        if config_file is None:
            config_file = os.path.join(BACKEND_CONFIG_PATH, "ros2_multirotor_ground_backend_config.yaml")
        super().__init__(config_file)


class ROS2MultiRotorBackend(Backend):

    def __init__(self, vehicle_id: int, config: BackendConfig):
        # Initialize the Backend object
        super().__init__(config)

        # Save the configurations for this backend
        self._id = vehicle_id
        self._num_rotors = config.get("num_rotors", 4)
        self._namespace = config.get("namespace", "vehicle")  + str(vehicle_id)

        # Save what whould be published/subscribed
        self._pub_graphical_sensors = config.get("pub_graphical_sensors", True)
        self._pub_sensors = config.get("pub_sensors", True)
        self._pub_state = config.get("pub_state", True)
        self._sub_control = config.get("sub_control", True)

        # Check if the tf2_ros library is loaded and if the flag is set to True
        self._pub_tf = config.get("pub_tf", False) and tf2_ros_loaded

        # Start the actual ROS2 setup here
        try:
            rclpy.init()
        except:
            # If rclpy is already initialized, just ignore the exception
            pass

        self.node = rclpy.create_node("simulator_vehicle_" + str(vehicle_id))

        # Initialize the publishers and subscribers
        self.initialize_publishers(config)
        self.initialize_subscribers()

        # Create a dictionary that will store the writers for the graphical sensors
        # NOTE: this is done this way, because the writers move data from the GPU->CPU and then publish it to ROS2
        # in a separate thread. This is done to avoid blocking the simulation
        self.graphical_sensors_writers = {}
        
        # Setup zero input reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]

        # -----------------------------------------------------
        # Initialize the static and dynamic tf broadcasters
        # -----------------------------------------------------
        if self._pub_tf:

            # Initiliaze the static tf broadcaster for the sensors
            self.tf_static_broadcaster = StaticTransformBroadcaster(self.node)

            # Initialize the static tf broadcaster for the base_link transformation
            self.send_static_transforms()

            # Initialize the dynamic tf broadcaster for the position of the body of the vehicle (base_link) with respect to the inertial frame (map - ENU) expressed in the inertil frame (map - ENU)
            self.tf_broadcaster = TransformBroadcaster(self.node)
    
    
    def initialize_publishers(self, config: dict):
        # ----------------------------------------------------- 
        # Create publishers for the state of the vehicle in ENU
        # -----------------------------------------------------
        if self._pub_state:
            if config.get("pub_pose", True):
                self.pose_pub = self.node.create_publisher(PoseStamped, self._namespace + "/" + config.get("pose_topic", "state/pose"), rclpy.qos.qos_profile_sensor_data)
            
            if config.get("pub_twist", True):
                self.twist_pub = self.node.create_publisher(TwistStamped, self._namespace + "/" + config.get("twist_topic", "state/twist"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_twist_inertial", True):
                self.twist_inertial_pub = self.node.create_publisher(TwistStamped, self._namespace + "/" + config.get("twist_inertial_topic", "state/twist_inertial"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_accel", True):
                self.accel_pub = self.node.create_publisher(AccelStamped, self._namespace + "/" + config.get("accel_topic", "state/accel"), rclpy.qos.qos_profile_sensor_data)

        # -----------------------------------------------------
        # Create publishers for the sensors of the vehicle
        # -----------------------------------------------------
        if self._pub_sensors:
            if config.get("pub_imu", True):
                self.imu_pub = self.node.create_publisher(Imu, self._namespace + "/" + config.get("imu_topic", "sensors/imu"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_mag", True):
                self.mag_pub = self.node.create_publisher(MagneticField, self._namespace + "/" + config.get("mag_topic", "sensors/mag"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_gps", True):
                self.gps_pub = self.node.create_publisher(NavSatFix, self._namespace + "/" + config.get("gps_topic", "sensors/gps"), rclpy.qos.qos_profile_sensor_data)

            if config.get("pub_gps_vel", True):
                self.gps_vel_pub = self.node.create_publisher(TwistStamped, self._namespace + "/" + config.get("gps_vel_topic", "sensors/gps_twist"), rclpy.qos.qos_profile_sensor_data)


    def initialize_subscribers(self):

        if self._sub_control:
            # Subscribe to vector of floats with the target angular velocities to control the vehicle
            # This is not ideal, but we need to reach out to NVIDIA so that they can improve the ROS2 support with custom messages
            # The current setup as it is.... its a pain!!!!
            self.rotor_subs = []
            for i in range(self._num_rotors):
                self.rotor_subs.append(self.node.create_subscription(Float64, self._namespace + str(self._id) + "/control/rotor" + str(i) + "/ref", lambda x: self.rotor_callback(x, i),10))


    def rotor_callback(self, ros_msg: Float64, rotor_id):
        # Update the reference for the rotor of the vehicle
        self.input_ref[rotor_id] = float(ros_msg.data)


    def send_static_transforms(self):
        # Create the transformation from base_link FLU (ROS standard) to base_link FRD (standard in airborn and marine vehicles)
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = self._namespace + '_' + 'base_link'
        t.child_frame_id = self._namespace + '_' + 'base_link_frd'

        # Converts from FLU to FRD
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 1.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        self.tf_static_broadcaster.sendTransform(t)

        # Create the transform from map, i.e inertial frame (ROS standard) to map_ned (standard in airborn or marine vehicles)
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "map_ned"
        
        # Converts ENU to NED
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = -0.7071068
        t.transform.rotation.y = -0.7071068
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        self.tf_static_broadcaster.sendTransform(t)


    def update_sensor(self, sensor_type: str, data):
        if not self._pub_sensors:
            return

        if sensor_type == "IMU":
            self.update_imu_data(data)
        elif sensor_type == "GPS":
            self.update_gps_data(data)
        elif sensor_type == "Magnetometer":
            self.update_mag_data(data)
        else:
            pass


    def update_imu_data(self, data):

        msg = Imu()

        # Update the header
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self._namespace + '_' + "base_link_frd"
        
        # Update the angular velocity (NED + FRD)
        msg.angular_velocity.x = data["angular_velocity"][0]
        msg.angular_velocity.y = data["angular_velocity"][1]
        msg.angular_velocity.z = data["angular_velocity"][2]
        
        # Update the linear acceleration (NED)
        msg.linear_acceleration.x = data["linear_acceleration"][0]
        msg.linear_acceleration.y = data["linear_acceleration"][1]
        msg.linear_acceleration.z = data["linear_acceleration"][2]

        # Publish the message with the current imu state
        self.imu_pub.publish(msg)


    def update_gps_data(self, data):
        msg = NavSatFix()
        msg_vel = TwistStamped()

        # Update the headers
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map_ned"
        msg_vel.header.stamp = msg.header.stamp
        msg_vel.header.frame_id = msg.header.frame_id

        # Update the status of the GPS
        status_msg = NavSatStatus()
        status_msg.status = 0 # unaugmented fix position
        status_msg.service = 1 # GPS service
        msg.status = status_msg

        # Update the latitude, longitude and altitude
        msg.latitude = data["latitude"]
        msg.longitude = data["longitude"]
        msg.altitude = data["altitude"]

        # Update the velocity of the vehicle measured by the GPS in the inertial frame (NED)
        msg_vel.twist.linear.x = data["velocity_north"]
        msg_vel.twist.linear.y = data["velocity_east"]
        msg_vel.twist.linear.z = data["velocity_down"]

        # Publish the message with the current GPS state
        self.gps_pub.publish(msg)
        self.gps_vel_pub.publish(msg_vel)


    def update_mag_data(self, data):
        msg = MagneticField()

        # Update the headers
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link_frd"

        msg.magnetic_field.x = data["magnetic_field"][0]
        msg.magnetic_field.y = data["magnetic_field"][1]
        msg.magnetic_field.z = data["magnetic_field"][2]

        # Publish the message with the current magnetic data
        self.mag_pub.publish(msg)


    def update_graphical_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of graphical sensor data
        """

        # Only process graphical sensor data if the flag is set to True
        if not self._pub_graphical_sensors:
            return

        if sensor_type == "MonocularCamera":
            self.update_monocular_camera_data(data)
        elif sensor_type == "Lidar":
            self.update_lidar_data(data)
        else:
            pass


    def update_monocular_camera_data(self, data):
        # fei no send??????????????????????????????
        # Check if the camera name exists in the writers dictionary
        if data["camera_name"] not in self.graphical_sensors_writers:
            self.add_monocular_camera_writter(data)


    def add_monocular_camera_writter(self, data):
        # List all the available writers: print(rep.writers.WriterRegistry._writers)
        render_prod_path = data["camera"]._render_product_path

        # Create the writer for the rgb camera
        writer = rep.writers.get("LdrColorSDROS2PublishImage")
        writer.initialize(nodeNamespace=self._namespace, topicName=data["camera_name"] + "/color/image_raw", frameId=data["camera_name"], queueSize=1)
        writer.attach([render_prod_path])

        # Add the writer to the dictionary
        self.graphical_sensors_writers[data["camera_name"]] = [writer]

        # Check if depth is enabled, if so, set the depth properties
        if "depth" in data:

            # Create the writer for the depth camera
            writer_depth = rep.writers.get("DistanceToImagePlaneSDROS2PublishImage")
            writer_depth.initialize(nodeNamespace=self._namespace, topicName=data["camera_name"] + "/depth", frameId=data["camera_name"], queueSize=1)
            writer_depth.attach([render_prod_path])

            # Add the writer to the dictionary
            self.graphical_sensors_writers[data["camera_name"]].append(writer_depth)

        # Create a writer for publishing the camera info
        writer_info = rep.writers.get("ROS2PublishCameraInfo")
        camera_info = read_camera_info(render_product_path=render_prod_path)
        writer_info.initialize(
            nodeNamespace=self._namespace, 
            topicName=data["camera_name"] + "/color/camera_info", 
            frameId=data["camera_name"], 
            queueSize=1,
            width=camera_info["width"],
            height=camera_info["height"],
            projectionType=camera_info["projectionType"],
            k=camera_info["k"].reshape([1, 9]),
            r=camera_info["r"].reshape([1, 9]),
            p=camera_info["p"].reshape([1, 12]),
            physicalDistortionModel=camera_info["physicalDistortionModel"],
            physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"]
        )

        writer_info.attach([render_prod_path])

        # Add the writer to the dictionary
        self.graphical_sensors_writers[data["camera_name"]].append(writer_info)

        gate_path = omni.syntheticdata.SyntheticData._get_node_path("PostProcessDispatch" + "IsaacSimulationGate", render_prod_path)

        # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
        og.Controller.attribute(gate_path + ".inputs:step").set(int(60/data["frequency"]))


    def update_lidar_data(self, data):
        # fei no send??????????????????????????????
        # Check if the lidar name exists in the writers dictionary
        if data["lidar_name"] not in self.graphical_sensors_writers:
            self.add_lidar_writter(data)


    def add_lidar_writter(self, data):

        # List all the available writers: print(rep.writers.WriterRegistry._writers)
        render_prod_path = rep.create.render_product(data["stage_prim_path"], [1, 1], name=data["lidar_name"])

        # Create the writer for the lidar
        writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
        writer.initialize(nodeNamespace=self._namespace, topicName=data["lidar_name"] + "/pointcloud", frameId=data["lidar_name"])
        writer.attach([render_prod_path])

        # Add the writer to the dictionary
        self.graphical_sensors_writers[data["lidar_name"]] = [writer]

        # Create the writer for publishing a laser scan message along with the point cloud
        writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
        writer.initialize(nodeNamespace=self._namespace, topicName=data["lidar_name"] + "/laserscan", frameId=data["lidar_name"])
        writer.attach([render_prod_path])
        self.graphical_sensors_writers[data["lidar_name"]].append(writer)


    def update_state(self, state):
        # Publish the state of the vehicle only if the flag is set to True
        if not self._pub_state:
            return

        pose = PoseStamped()
        twist = TwistStamped()
        twist_inertial = TwistStamped()
        accel = AccelStamped()

        # Update the header
        pose.header.stamp = self.node.get_clock().now().to_msg()
        twist.header.stamp = pose.header.stamp
        twist_inertial.header.stamp = pose.header.stamp
        accel.header.stamp = pose.header.stamp

        pose.header.frame_id = "map"
        twist.header.frame_id = self._namespace + "_" + "base_link"
        twist_inertial.header.frame_id = "map"
        accel.header.frame_id = "map"

        # Fill the position and attitude of the vehicle in ENU
        pose.pose.position.x = state.position[0]
        pose.pose.position.y = state.position[1]
        pose.pose.position.z = state.position[2]

        pose.pose.orientation.x = state.attitude[0]
        pose.pose.orientation.y = state.attitude[1]
        pose.pose.orientation.z = state.attitude[2]
        pose.pose.orientation.w = state.attitude[3]

        # Fill the linear and angular velocities in the body frame of the vehicle
        twist.twist.linear.x = state.linear_body_velocity[0]
        twist.twist.linear.y = state.linear_body_velocity[1]
        twist.twist.linear.z = state.linear_body_velocity[2]

        twist.twist.angular.x = state.angular_velocity[0]
        twist.twist.angular.y = state.angular_velocity[1]
        twist.twist.angular.z = state.angular_velocity[2]

        # Fill the linear velocity of the vehicle in the inertial frame
        twist_inertial.twist.linear.x = state.linear_velocity[0]
        twist_inertial.twist.linear.y = state.linear_velocity[1]
        twist_inertial.twist.linear.z = state.linear_velocity[2]

        # Fill the linear acceleration in the inertial frame
        accel.accel.linear.x = state.linear_acceleration[0]
        accel.accel.linear.y = state.linear_acceleration[1]
        accel.accel.linear.z = state.linear_acceleration[2]

        # Publish the messages containing the state of the vehicle
        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.twist_inertial_pub.publish(twist_inertial)
        self.accel_pub.publish(accel)

        # Update the dynamic tf broadcaster with the current position of the vehicle in the inertial frame
        if self._pub_tf:
            t = TransformStamped()
            t.header.stamp = pose.header.stamp
            t.header.frame_id = "map"
            t.child_frame_id = self._namespace + '_' + 'base_link'
            t.transform.translation.x = state.position[0]
            t.transform.translation.y = state.position[1]
            t.transform.translation.z = state.position[2]
            t.transform.rotation.x = state.attitude[0]
            t.transform.rotation.y = state.attitude[1]
            t.transform.rotation.z = state.attitude[2]
            t.transform.rotation.w = state.attitude[3]
            self.tf_broadcaster.sendTransform(t)
        

    def input_reference(self):
        """
        Method that is used to return the latest target angular velocities to be applied to the vehicle

        Returns:
            A list with the target angular velocities for each individual rotor of the vehicle
        """
        return self.input_ref


    def update(self, dt: float):
        """
        Method that when implemented, should be used to update the state of the backend and the information being sent/received
        from the communication interface. This method will be called by the simulation on every physics step
        """

        # In this case, do nothing as we are sending messages as soon as new data arrives from the sensors and state
        # and updating the reference for the thrusters as soon as receiving from ROS2 topics
        # Just poll for new ROS 2 messages in a non-blocking way
        rclpy.spin_once(self.node, timeout_sec=0)


    def start(self):
        """
        Method that when implemented should handle the begining of the simulation of vehicle
        """
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]


    def stop(self):
        """
        Method that when implemented should handle the stopping of the simulation of vehicle
        """
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]


    def reset(self):
        """
        Method that when implemented, should handle the reset of the vehicle simulation to its original state
        """
        # Reset the reference for the thrusters
        self.input_ref = [0.0 for i in range(self._num_rotors)]