# SkyRover Simulator

![IsaacSim 4.5.0](https://img.shields.io/badge/IsaacSim-4.5.0-brightgreen.svg)
![PX4-Autopilot 1.14.3](https://img.shields.io/badge/PX4--Autopilot-1.14.3-brightgreen.svg)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04LTS-brightgreen.svg)

**SkyRover Simulator** 是一个基于 [NVIDIA Omniverse](https://docs.omniverse.nvidia.com/) 和 [IsaacSim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html) 构建的仿真框架。该项目基于开源项目 [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) 改编而来，专注于提供空地协同机器人的仿真能力。它为空中多旋翼飞行器和地面移动机器人提供了易用且强大的动力学仿真接口，支持 [PX4](https://px4.io/) 飞控集成以及 ROS 2 控制接口。

## 主要特性

### 支持的机器人类型
- **多旋翼飞行器 (Multirotor Aerial Vehicles)**: 支持基于 PX4 Mavlink 和 ROS 2 的四旋翼无人机仿真
- **地面移动机器人 (Ground Vehicles)**: 支持基于 ROS 2 的地面机器人仿真
- **空地协同**: 能够同时仿真多个空中和地面机器人，适用于空地协同任务研究

### 控制后端
- **PX4 Mavlink Backend**: 完整的 PX4 SITL (Software-In-The-Loop) 集成，支持标准的 MAVLink 通信协议
- **ROS 2 Backend**: 原生支持 ROS 2 接口，便于与 ROS 2 生态系统集成
- **多后端支持**: 单个机器人可同时启用多个控制后端（如 PX4 + ROS 2）

### 传感器系统
- **惯性测量单元 (IMU)**: 加速度计和陀螺仪仿真
- **GPS**: 全球定位系统仿真
- **磁力计 (Magnetometer)**: 地磁场传感器仿真
- **气压计 (Barometer)**: 大气压力传感器仿真
- **视觉传感器**: 单目相机和激光雷达 (LiDAR) 支持

### 动力学模型
- **推力曲线模型**: 二次推力曲线仿真
- **空气阻力模型**: 线性和非线性阻力仿真
- **完整的刚体动力学**: 基于 Isaac Sim 的物理引擎

## 项目结构

```
SkyRoverSimulator/
├── examples/                      # 示例脚本
│   ├── 1_px4_multi_aerial_vehicle.py
│   ├── 2_ros2_ground_vehicle.py
│   ├── 3_px4_single_aerial_vehicle.py
│   └── 4_px4_single_aerial_vehicle_with_power.py
├── extensions/
│   └── skyrover.simulator/        # 核心扩展
│       ├── skyrover/
│       │   └── simulator/
│       │       ├── core/          # 核心模块
│       │       │   ├── backends/  # 控制后端 (PX4, ROS2)
│       │       │   ├── sensors/   # 传感器模块
│       │       │   ├── graphical_sensors/  # 图形传感器
│       │       │   ├── dynamics/  # 动力学模型
│       │       │   ├── thrusters/ # 推进器模型
│       │       │   ├── vehicles/  # 机器人模型
│       │       │   └── interface/ # SkyRover 接口
│       │       └── impl/          # 实现细节
│       └── config/                # 配置文件
└── docs/                          # 文档

```

## 快速开始

### 环境要求

- **操作系统**: Ubuntu 22.04 LTS
- **Isaac Sim**: 4.5.0
- **Python**: 3.7+
- **PX4-Autopilot**: 1.14.3 (如果使用 PX4 后端)
- **ROS 2**: Humble 或更高版本 (如果使用 ROS 2 后端)

### 安装

1. 安装 NVIDIA Isaac Sim 4.5.0

2. 设置环境变量:
```bash
export ISAACSIM_PATH="path/to/isaac_sim"
```

3. 安装 SkyRover Simulator 扩展:
```bash
cd extensions/skyrover.simulator
python setup.py install
```

### 运行示例

#### 示例 1: PX4 多机仿真
```bash
python examples/1_px4_multi_aerial_vehicle.py
```
该示例启动 3 架配备 PX4 Mavlink 后端的四旋翼无人机。

#### 示例 2: ROS 2 地面机器人
```bash
python examples/2_ros2_ground_vehicle.py
```
该示例启动一个基于 ROS 2 控制的地面移动机器人。

#### 示例 3: PX4 单机仿真（含 ROS 2）
```bash
python examples/3_px4_single_aerial_vehicle.py
```
该示例启动一架同时配备 PX4 Mavlink 和 ROS 2 后端的四旋翼无人机。

## 核心组件

### SkyRoverInterface
主接口类，用于管理仿真环境和机器人实例:
```python
from skyrover.simulator.core.interface.skyrover_interface import SkyRoverInterface

sk = SkyRoverInterface()
sk.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
```

### 多旋翼飞行器 (MultirotorAerial)
配置和生成空中多旋翼机器人:
```python
from skyrover.simulator.core.vehicles.multirotor_aerial import MultirotorAerial, MultirotorAerialConfig
from skyrover.simulator.core.backends.px4_mavlink_backend import PX4MavlinkBackend

config = MultirotorAerialConfig()
config.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig(0))]

MultirotorAerial(
    "/World/quadrotor",
    AERIAL_ROBOTS['Iris'],
    vehicle_id=0,
    init_pos=[0.0, 0.0, 0.07],
    init_orientation=[0.0, 0.0, 0.0, 1.0],
    config=config
)
```

### 地面机器人 (MultirotorGround)
配置和生成地面移动机器人:
```python
from skyrover.simulator.core.vehicles.multirotor_ground import MultirotorGround, MultirotorGroundConfig
from skyrover.simulator.core.backends.ros2_multirotor_backend import ROS2MultiRotorBackend

config = MultirotorGroundConfig()
config.backends = [ROS2MultiRotorBackend(vehicle_id=0, config=ROS2MultiRotorGroundBackendConfig())]

MultirotorGround(
    "/World/Root",
    GROUND_ROBOTS['ground'],
    vehicle_id=0,
    init_pos=[0.0, 0.0, 5.0],
    init_orientation=[0.0, 0.0, 0.0, 1.0],
    config=config
)
```

## 配置

项目使用 YAML 配置文件来定义机器人参数和仿真环境:

- `extensions/skyrover.simulator/config/aerial_robot.yaml`: 空中机器人配置
- `extensions/skyrover.simulator/config/ground_robot.yaml`: 地面机器人配置
- `extensions/skyrover.simulator/config/configs.yaml`: 全局配置

## 致谢

SkyRover Simulator 基于开源项目 [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) 改编而来。感谢 Pegasus Simulator 团队为社区提供的优秀基础框架。

**Pegasus Simulator 原作者**:
- [Marcelo Jacinto](https://github.com/MarceloJacinto) - 项目创始人和主要开发者
- [João Pinto](https://github.com/jschpinto) - 架构和示例应用

**相关引用**:
```
@INPROCEEDINGS{10556959,
  author={Jacinto, Marcelo and Pinto, João and Patrikar, Jay and Keller, John and Cunha, Rita and Scherer, Sebastian and Pascoal, António},
  booktitle={2024 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation}, 
  year={2024},
  pages={917-922},
  doi={10.1109/ICUAS60882.2024.10556959}
}
```

## 开发团队

**SkyRover Simulator**:
- **项目负责人**: Fei Wang (feiwang@dlmu.edu.cn)
- **单位**: 大连海事大学 (Dalian Maritime University)

## 许可证

SkyRover Simulator 基于 [BSD-3-Clause License](LICENSE) 发布。

依赖项和资源的许可证文件位于 `docs/licenses` 目录中。

- NVIDIA Isaac Sim 根据 [个人许可证](https://www.nvidia.com/en-us/omniverse/download/) 免费提供
- PX4-Autopilot 是一个开源项目，采用 [BSD-3 License](https://github.com/PX4/PX4-Autopilot)

## 贡献

欢迎社区贡献来改进本项目。如有问题、建议或功能请求，请通过以下方式参与:

- 使用 [Issues](../../issues) 跟踪开发工作、bug 和文档问题
- 使用 [Pull Requests](../../pulls) 修复 bug 或贡献代码、示例或改进文档

## 项目赞助

- 大连海事大学 (Dalian Maritime University)

---
