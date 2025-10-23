"""
| Author: Fei Wang (feiwang@dlmu.edu.cn)
| License: BSD-3-Clause. Copyright (c) 2025, Fei Wang. All rights reserved.
"""

from .backend import Backend, BackendConfig
# from .px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
# from .ardupilot_mavlink_backend import ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig

# Check if the ROS2 package is installed
try:
    from .ros2_multirotor_backend import ROS2MultiRotorBackend, ROS2MultiRotorBackendConfig, ROS2MultiRotorAerialBackendConfig, ROS2MultiRotorGroundBackendConfig
except:
    import carb
    carb.log_warn("ROS2 package not installed. ROS2MultiRotorBackend will not be available")