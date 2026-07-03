SkyRover Simulator
##################

概述
====

**SkyRover Simulator** 是一个基于 `NVIDIA Omniverse <https://docs.omniverse.nvidia.com/>`__ 和 `Isaac Sim <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html>`__ 构建的仿真框架。该项目基于开源项目 `Pegasus Simulator <https://github.com/PegasusSimulator/PegasusSimulator>`__ 改编而来，专注于提供空地协同机器人的仿真能力。它为空中多旋翼飞行器和地面移动机器人提供了易用且强大的动力学仿真接口，支持 `PX4 <https://px4.io/>`__ 飞控集成以及 ROS 2 控制接口。

主要特性
========

支持的机器人类型
~~~~~~~~~~~~~~~~
- **多旋翼飞行器 (Multirotor Aerial Vehicles)**: 支持基于 PX4 Mavlink 和 ROS 2 的四旋翼无人机仿真
- **地面移动机器人 (Ground Vehicles)**: 支持基于 ROS 2 的地面机器人仿真
- **空地协同**: 能够同时仿真多个空中和地面机器人，适用于空地协同任务研究

控制后端
~~~~~~~~
- **PX4 Mavlink Backend**: 完整的 PX4 SITL (Software-In-The-Loop) 集成，支持标准的 MAVLink 通信协议
- **ROS 2 Backend**: 原生支持 ROS 2 接口，便于与 ROS 2 生态系统集成
- **多后端支持**: 单个机器人可同时启用多个控制后端（如 PX4 + ROS 2）

传感器系统
~~~~~~~~~~
- **惯性测量单元 (IMU)**: 加速度计和陀螺仪仿真
- **GPS**: 全球定位系统仿真
- **磁力计 (Magnetometer)**: 地磁场传感器仿真
- **气压计 (Barometer)**: 大气压力传感器仿真
- **视觉传感器**: 单目相机和激光雷达 (LiDAR) 支持

动力学模型
~~~~~~~~~~
- **推力曲线模型**: 二次推力曲线仿真
- **空气阻力模型**: 线性和非线性阻力仿真
- **完整的刚体动力学**: 基于 Isaac Sim 的物理引擎

致谢
====

SkyRover Simulator 基于开源项目 `Pegasus Simulator <https://github.com/PegasusSimulator/PegasusSimulator>`__ 改编而来。感谢 Pegasus Simulator 团队为社区提供的优秀基础框架。

**Pegasus Simulator 原作者**:
- `Marcelo Jacinto <https://github.com/MarceloJacinto>`__ - 项目创始人和主要开发者
- `João Pinto <https://github.com/jschpinto>`__ - 架构和示例应用

**相关引用**:

.. code-block:: bibtex

   @INPROCEEDINGS{10556959,
      author={Jacinto, Marcelo and Pinto, João and Patrikar, Jay and Keller, John and Cunha, Rita and Scherer, Sebastian and Pascoal, António},
      booktitle={2024 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
      title={Pegasus Simulator: An Isaac Sim Framework for Multiple Aerial Vehicles Simulation}, 
      year={2024},
      pages={917-922},
      doi={10.1109/ICUAS60882.2024.10556959}
   }

开发团队
========

**SkyRover Simulator**:
- **项目负责人**: Fei Wang (feiwang@dlmu.edu.cn)
- **单位**: 大连海事大学 (Dalian Maritime University)

项目赞助
========

- 大连海事大学 (Dalian Maritime University)

其他仿真框架
============

在此我们感谢前人的杰出工作，他们为本项目提供了灵感：

- Gazebo simulator
- RotorS simulation plugin for gazebo
- PX4-SITL simulation plugin for gazebo
- Microsoft Airsim project for Unreal Engine
- Flightmare simulator for Unity
- jMAVSim java simulator

*"如果我看得更远，那是因为我站在巨人的肩膀上。"* —— 艾萨克·牛顿

许可证
======

SkyRover Simulator 基于 BSD-3-Clause License 发布。

依赖项和资源的许可证文件位于 ``docs/licenses`` 目录中。

- NVIDIA Isaac Sim 根据 `个人许可证 <https://www.nvidia.com/en-us/omniverse/download/>`__ 免费提供
- PX4-Autopilot 是一个开源项目，采用 `BSD-3 License <https://github.com/PX4/PX4-Autopilot>`__

.. automodule::"skyrover.simulator"
    :platform: Linux-x86_64
    :members:
    :undoc-members:
    :show-inheritance:
    :imported-members:
    :exclude-members: contextmanager
