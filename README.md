# SkyRover Simulator

![IsaacSim 4.5.0](https://img.shields.io/badge/IsaacSim-4.5.0-brightgreen.svg)
![PX4-Autopilot 1.14.3](https://img.shields.io/badge/PX4--Autopilot-1.14.3-brightgreen.svg)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04LTS-brightgreen.svg)

**SkyRover Simulator** is a simulation framework built on [NVIDIA Omniverse](https://docs.omniverse.nvidia.com/) and [Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html). Adapted from the open-source [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator), it provides an integrated platform for simulating air-ground cooperative robotic systems, supporting both aerial multirotors and ground mobile robots with [PX4](https://px4.io/) autopilot and ROS 2 integration.

## Features

- **Multi-Domain Robotics**: Simultaneous simulation of aerial multirotors (quadcopters) and ground mobile robots for cooperative mission scenarios
- **Flexible Control Backends**: PX4 Mavlink SITL and ROS 2 interfaces with support for multiple concurrent backends
- **Comprehensive Sensor Suite**: IMU, GPS, Magnetometer, Barometer, Camera, and LiDAR
- **High-Fidelity Physics**: Quadratic thrust curves, linear/nonlinear drag models, and full rigid body dynamics powered by Isaac Sim

## Project Structure

```
SkyRoverSimulator/
├── examples/                          # Example simulation scripts
├── extensions/
│   └── skyrover.simulator/
│       ├── config/                    # Configuration files
│       └── skyrover/simulator/
│           ├── core/                  # Core modules
│           │   ├── backends/          # Control backends (PX4, ROS2)
│           │   ├── sensors/           # Sensor implementations
│           │   ├── dynamics/          # Physics and dynamics
│           │   └── vehicles/          # Robot models
│           └── impl/                  # Implementation details
└── docs/                              # Documentation
```

## Quick Start

### System Requirements

- **Operating System**: Ubuntu 22.04 LTS
- **Isaac Sim**: 4.5.0
- **Python**: 3.7+
- **PX4-Autopilot**: 1.14.3 (optional, for PX4 backend)
- **ROS 2**: Humble or later (optional, for ROS 2 backend)

### Installation

1. Install NVIDIA Isaac Sim 4.5.0

2. Set the Isaac Sim path environment variable:
```bash
export ISAACSIM_PATH="/path/to/isaac_sim"
```

3. Install the SkyRover Simulator extension:
```bash
cd extensions/skyrover.simulator
python setup.py install
```

### Running Examples

**Multi-vehicle aerial simulation with PX4:**
```bash
python examples/1_px4_multi_aerial_vehicle.py
```

**Ground robot simulation with ROS 2:**
```bash
python examples/2_ros2_ground_vehicle.py
```

**Hybrid aerial vehicle with PX4 and ROS 2:**
```bash
python examples/3_px4_single_aerial_vehicle.py
```

## Usage Guide

### Initialize the Simulation Interface
```python
from skyrover.simulator.core.interface.skyrover_interface import SkyRoverInterface
from skyrover.simulator.impl.params import SIMULATION_ENVIRONMENTS

sk = SkyRoverInterface()
sk.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
```

### Spawn an Aerial Vehicle
```python
from skyrover.simulator.core.vehicles.multirotor_aerial import MultirotorAerial, MultirotorAerialConfig
from skyrover.simulator.core.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from skyrover.simulator.impl.params import AERIAL_ROBOTS

config = MultirotorAerialConfig()
config.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig(vehicle_id=0))]

MultirotorAerial(
    stage_prefix="/World/quadrotor",
    usd_file=AERIAL_ROBOTS['Iris'],
    vehicle_id=0,
    init_pos=[0.0, 0.0, 0.07],
    init_orientation=[0.0, 0.0, 0.0, 1.0],
    config=config
)
```

### Spawn a Ground Robot
```python
from skyrover.simulator.core.vehicles.multirotor_ground import MultirotorGround, MultirotorGroundConfig
from skyrover.simulator.core.backends.ros2_multirotor_backend import ROS2MultiRotorBackend, ROS2MultiRotorGroundBackendConfig
from skyrover.simulator.impl.params import GROUND_ROBOTS

config = MultirotorGroundConfig()
config.backends = [ROS2MultiRotorBackend(vehicle_id=0, config=ROS2MultiRotorGroundBackendConfig())]

MultirotorGround(
    stage_prefix="/World/Root",
    usd_file=GROUND_ROBOTS['ground'],
    vehicle_id=0,
    init_pos=[0.0, 0.0, 5.0],
    init_orientation=[0.0, 0.0, 0.0, 1.0],
    config=config
)
```

## Configuration

Robot parameters and simulation environments can be customized through YAML configuration files:

- `extensions/skyrover.simulator/config/aerial_robot.yaml` - Aerial robot parameters
- `extensions/skyrover.simulator/config/ground_robot.yaml` - Ground robot parameters
- `extensions/skyrover.simulator/config/configs.yaml` - Global simulation settings

## Acknowledgements

SkyRover Simulator is adapted from the open-source [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) project. We express our gratitude to the Pegasus Simulator team for providing an excellent foundation for this work.

**Original Pegasus Simulator Authors:**
- [Marcelo Jacinto](https://github.com/MarceloJacinto) - Project Founder and Lead Developer
- [João Pinto](https://github.com/jschpinto) - Architecture and Examples

**Related Publication:**
```bibtex
@INPROCEEDINGS{10556959,
  author={Jacinto, Marcelo and Pinto, João and Patrikar, Jay and Keller, John and Cunha, Rita and Scherer, Sebastian and Pascoal, António},
  booktitle={2024 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation}, 
  year={2024},
  pages={917-922},
  doi={10.1109/ICUAS60882.2024.10556959}
}
```

## Development Team

**SkyRover Simulator:**
- **Project Lead**: Fei Wang (feiwang@dlmu.edu.cn)
- **Institution**: Dalian Maritime University

## License

SkyRover Simulator is released under the [BSD-3-Clause License](LICENSE).

**Dependencies:**
- NVIDIA Isaac Sim is available under [individual license](https://www.nvidia.com/en-us/omniverse/download/)
- PX4-Autopilot is licensed under [BSD-3 License](https://github.com/PX4/PX4-Autopilot)

## Contributing

We welcome contributions from the community! Please feel free to:
- Report bugs and request features via [Issues](../../issues)
- Submit improvements via [Pull Requests](../../pulls)
- Share your simulation scenarios and robot configurations

## Sponsor

This project is supported by Dalian Maritime University.

---

**Disclaimer**: This project is under active development. API changes may occur in future releases.
