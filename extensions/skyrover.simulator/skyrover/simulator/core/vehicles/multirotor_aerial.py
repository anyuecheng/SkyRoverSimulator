"""
| File: multirotor.py
| Author: Fei Wang (feiwang@dlmu.edu.cn)
| License: BSD-3-Clause. Copyright (c) 2025, Fei Wang. All rights reserved.
"""

import numpy as np

from omni.isaac.dynamic_control import _dynamic_control

# The vehicle interface
from skyrover.simulator.core.vehicles.vehicle import Vehicle

# Mavlink interface
# from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig

from skyrover.simulator.core.config_yaml import ConfigYaml
from skyrover.simulator.impl.params import AERIAL_ROBOT_CONFIG

# Sensors and dynamics setup
from skyrover.simulator.core.graphical_sensors import MonocularCamera
from skyrover.simulator.core.graphical_sensors import Lidar
from skyrover.simulator.core.dynamics import LinearDrag
from skyrover.simulator.core.thrusters import QuadraticThrustCurve
from skyrover.simulator.core.sensors import Barometer, IMU, Magnetometer, GPS

class MultirotorAerialConfig(ConfigYaml):
    def __init__(self, filename: str = AERIAL_ROBOT_CONFIG):
        """Initialize the BackendConfig class
        """
        super().__init__(filename)

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = self.get("stage_prefix", "quadrotor")
        # --- START: 新增电气和物理参数 (用于功率模型) ---
        self.kv = self.get("kv", 1000) 
        self.k_t = 60 / (2 * np.pi * self.kv)             # K_t (Nm/A)
        self.resistance = self.get("resistance_motor", 0.25) # R_motor (Ohm)
        self.io_current = self.get("io_current", 0.4)     # I_o (A)
        self.esc_efficiency = self.get("esc_efficiency", 0.95)
        self.V_full = self.get("battery_v_full", 16.8)    # 4S 满电电压 (V)
        self.R_internal = self.get("resistance_internal", 0.03) # R_internal (Ohm)
        self.base_power_electronics = self.get("base_power_electronics", 5.0) # 基础电子功耗 (W)
        self.C_Q = self.get("prop_C_Q", 0.01)            # 螺旋桨扭矩系数
        self.diameter = self.get("prop_diameter", 0.203)  # 螺旋桨直径 (m, 8英寸)
        self.air_density = self.get("air_density", 1.225) # 空气密度 (kg/m^3)
        # --- END: 新增电气和物理参数 ---

        # The USD file that describes the visual aspect of the vehicle (and some properties such as mass and moments of inertia)
        self.usd_file = self.get("usd_file", "")

        # The default thrust curve for a quadrotor and dynamics relating to drag
        self.thrust_curve = QuadraticThrustCurve(self)
        # self.drag = LinearDrag([0.50, 0.30, 0.0])
        self.drag = LinearDrag(self.get("drag", [0.50, 0.30, 0.0]))

        # The default sensors for a quadrotor
        self.sensors = []
        if self.get("use_barometer", True):
            self.sensors.append(Barometer())
        if self.get("use_imu", True):
            self.sensors.append(IMU())
        if self.get("use_magnetometer", True):
            self.sensors.append(Magnetometer())
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
        # print("stage_prefix:", self.stage_prefix)
        # print("usd_file:", self.usd_file)
        # print("thrust_curve.rot_dir:", self.thrust_curve.rot_dir)
        # print("thrust_curve.rolling_moment:", self.thrust_curve.rolling_moment)
        # print("thrust_curve.velocity:", self.thrust_curve.velocity)
        # print("thrust_curve.force:", self.thrust_curve.force)
        # print("drag:", self.drag.drag)
        # for sensor in self.sensors:
        #     print("sensor:", sensor.sensor_type)
        #     print("sensor update_rate:", sensor.update_rate)
        # for sensor in self.graphical_sensors:
        #     print("graphical_sensors:", sensor.sensor_type)
        #     print("graphical_sensors update_rate:", sensor.update_rate)
        # for backend in self.backends:
        #     print("backend:", type(backend))
        #     print("backend config:", backend.config.filename)


class MultirotorAerial(Vehicle):
    """Multirotor class - It defines a base interface for creating a multirotor
    """
    def __init__(
        self,
        # Simulation specific configurations
        stage_prefix: str = "quadrotor",
        usd_file: str = "",
        vehicle_id: int = 0,
        # Spawning pose of the vehicle
        init_pos=[0.0, 0.0, 0.07],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        config=MultirotorAerialConfig(),
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

        # 2. Setup the dynamics of the system - get the thrust curve of the vehicle from the configuration
        self._thrusters = config.thrust_curve
        self._drag = config.drag
        self.total_current = 0.0
        # --- START: 新增功率模型状态和配置引用 ---
        self._power_config = config
        self._instantaneous_power_W = 0.0  # 瞬时总功率 (W)
        self._total_energy_J = 0.0         # 累积总能量 (J)
        self._V_actual = self._power_config.V_full # 实际电池端电压 (V)
        
        # 预计算扭矩的物理常数部分 Q = K_Q_phys * omega^2
        self._K_Q_phys = (self._power_config.C_Q * self._power_config.air_density * (self._power_config.diameter**5)) / (4*np.pi**2)
        
        # --- END: 新增功率模型状态和配置引用 ---


    def start(self):
        """In this case we do not need to do anything extra when the simulation starts"""
        pass

    def stop(self):
        """In this case we do not need to do anything extra when the simulation stops"""
        pass
    
    def get_body_velocity(self) -> np.ndarray:
        """
        获取机体坐标系下的瞬时线性速度向量 [vx, vy, vz]。
        
        Returns:
            np.ndarray: 3x1 速度向量 (m/s)
        """
        # 1. 优先从内部状态对象获取
        if self._state is not None:
            # 根据 State 类的定义，self._state.linear_body_velocity 
            # 存储的就是 FLU 身体坐标系下的速度 [u, v, w]
            return self._state.linear_body_velocity
        else:
            # 2. 如果 _state 丢失，这是严重的系统错误。
            # 为了防止程序崩溃，我们返回零，但必须发出警告。
            # 注意：在实际运行中，如果 _state 丢失，这会导致阻力计算错误。
            print(f"⚠️ Warning: self._state is None. Cannot retrieve body velocity. Returning 0.")
            return np.array([0.0, 0.0, 0.0]) 

     # --- 新增方法: 计算瞬时功率 ---

    def _calculate_instantaneous_power(self, desired_rotor_velocities: list, dt: float):
        """
        根据期望转速计算瞬时功率消耗，使用物理电机/电池模型。
        """
        
        total_ideal_current = 0.0
        
        # --- 步骤 1: 计算总“期望”电流 (假设所有电机都可达) ---
        # 存储每个电机的中间计算值，以便在步骤 3 中复用
        motor_calcs = []
        
        for omega in desired_rotor_velocities:
            motor_torque_needed = self._K_Q_phys * (omega**2)
            I_prop_needed = motor_torque_needed / self._power_config.k_t
            I_single_ideal = I_prop_needed + (self._power_config.io_current if omega > 0.1 else 0)
            
            V_bemf = self._power_config.k_t * omega
            V_required_motor = V_bemf + I_single_ideal * self._power_config.resistance
            
            motor_calcs.append({
                'omega': omega,
                'I_single_ideal': I_single_ideal,
                'I_prop_needed': I_prop_needed,
                'V_bemf': V_bemf,
                'V_required_motor': V_required_motor
            })
            
            total_ideal_current += I_single_ideal

        # --- 步骤 2: 计算“实际”电池电压 (基于总期望电流) ---
        I_total_system = total_ideal_current
        self.total_current = total_ideal_current
        V_actual = self._power_config.V_full - I_total_system * self._power_config.R_internal
        
        total_power_motor = 0.0
        # print(self._K_Q_phys)
        # print(self._thrusters._rotor_constant[0],self._thrusters._rotor_constant[1],self._thrusters._rotor_constant[2],self._thrusters._rotor_constant[3])
        # print(self._thrusters._rolling_moment_coefficient[0],self._thrusters._rolling_moment_coefficient[1],self._thrusters._rolling_moment_coefficient[2],self._thrusters._rolling_moment_coefficient[3])

        print(f"[K_Q_phys] {self._K_Q_phys}")

        for i in range(4):
            print(
                f"[Motor {i}] "
                f"rotor_constant={self._thrusters._rotor_constant[i]:.8e}, "
                f"rolling_moment_coefficient={self._thrusters._rolling_moment_coefficient[i]:.8e}"
            )
            
        # --- 步骤 3: 检查可达性并计算每个电机的实际功率 ---
        for calc in motor_calcs:
            
            P_in_motor_single = 0.0 # 初始化此电机的功率

            if V_actual < calc['V_required_motor']:
                 # --- START: 降级计算逻辑 ---
                 # 警告: 电机无法达到所需转速（因为电压不足）
                 rpm_unreachable = calc['omega'] * 60 / (2 * np.pi)
                 print(f"⚠️ Power Warning: Motor required V {calc['V_required_motor']:.2f}V exceeds available V {V_actual:.2f}V (Target RPM: {rpm_unreachable:.0f})!")
                 
                 # 重新计算该电机在 V_actual 限制下的实际扭矩电流
                 # I_prop_limited = (V_actual - V_bemf) / R_motor
                 I_prop_limited = (V_actual - calc['V_bemf']) / self._power_config.resistance
                 # 确保电流不为负（如果 V_bemf > V_actual，说明电机在反向制动）
                 I_prop_limited = max(0, I_prop_limited) 
                 
                 I_single_actual = I_prop_limited + (self._power_config.io_current if calc['omega'] > 0.1 else 0)
                 
                 # 功率 P = V * I (使用实际电压和受限后的实际电流)
                 P_in_motor_single = V_actual * I_single_actual
                 # --- END: 降级计算逻辑 ---

            else:
                # --- 正常计算逻辑 ---
                # 电压充足，使用步骤 1 中计算的理想电流
                P_in_motor_single = calc['V_required_motor'] * calc['I_single_ideal']
                # --- END: 正常计算逻辑 ---
            
            total_power_motor += P_in_motor_single
            
        # --- 步骤 4: 计算系统总功率 ---
        battery_power_total = total_power_motor / self._power_config.esc_efficiency
        total_system_power = battery_power_total + self._power_config.base_power_electronics
        
        # --- 步骤 5: 更新内部状态和能量 (积分) ---
        self._instantaneous_power_W = total_system_power
        self._total_energy_J += total_system_power * dt
        self._V_actual = V_actual # 存储当前步的实际电压
        
        return total_system_power, V_actual



    def update(self, dt: float):
        """
        Method that computes and applies the forces to the vehicle in simulation based on the motor speed. 
        This method must be implemented by a class that inherits this type. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the articulation root of the vehicle
        articulation = self.get_dc_interface().get_articulation(self._stage_prefix)

        # Get the desired angular velocities for each rotor from the first backend (can be mavlink or other) expressed in rad/s
        if len(self._backends) != 0:
            desired_rotor_velocities = self._backends[0].input_reference()
        else:
            desired_rotor_velocities = [0.0 for i in range(self._thrusters._num_rotors)]

        # --- 新增: 在应用推力前计算功率 ---
        self._calculate_instantaneous_power(desired_rotor_velocities, dt)
        
        # 您现在可以通过 self._instantaneous_power_W 访问瞬时功率，
        # 并且 self._total_energy_J 储存了累积能耗。
        for omega in desired_rotor_velocities:
            print(f"omega: {omega}rad/s")
        print(f"total_current:{self.total_current}")
        print(f"actual V:{self._V_actual}")
        print(f"P: {self._instantaneous_power_W:.1f} W, E: {self._total_energy_J:.1f} J") # 可选的调试输出
        # ------------------------------------

        # Input the desired rotor velocities in the thruster model
        self._thrusters.set_input_reference(desired_rotor_velocities)

        # Get the desired forces to apply to the vehicle
        forces_z, _, yawing_moment = self._thrusters.update(self._state, dt)

        # Apply force to each rotor
        for i in range(len(forces_z)):

            # Apply the force in Z on the rotor frame
            self.apply_force([0.0, 0.0, forces_z[i]], body_part="/rotor" + str(i))

            # Generate the rotating propeller visual effect
            self.handle_propeller_visual(i, forces_z[i], articulation)

        
        # Apply the torque to the body frame of the vehicle that corresponds to the yawing moment
        self.apply_torque([0.0, 0.0, yawing_moment], "/body")

        # Compute the total linear drag force to apply to the vehicle's body frame
        drag = self._drag.update(self._state, dt)
        self.apply_force(drag, body_part="/body")

        # Call the update methods in all backends
        for backend in self._backends:
            backend.update(dt)



    def handle_propeller_visual(self, rotor_number, force: float, articulation):
        """
        Auxiliar method used to set the joint velocity of each rotor (for animation purposes) based on the 
        amount of force being applied on each joint

        Args:
            rotor_number (int): The number of the rotor to generate the rotation animation
            force (float): The force that is being applied on that rotor
            articulation (_type_): The articulation group the joints of the rotors belong to
        """

        # Rotate the joint to yield the visual of a rotor spinning (for animation purposes only)
        joint = self.get_dc_interface().find_articulation_dof(articulation, "joint" + str(rotor_number))

        # Spinning when armed but not applying force
        if 0.0 < force < 0.1:
            self.get_dc_interface().set_dof_velocity(joint, 5 * self._thrusters.rot_dir[rotor_number])
        # Spinning when armed and applying force
        elif 0.1 <= force:
            self.get_dc_interface().set_dof_velocity(joint, 100 * self._thrusters.rot_dir[rotor_number])
        # Not spinning
        else:
            self.get_dc_interface().set_dof_velocity(joint, 0)


    def calculate_roll_pitch_torques(self):
        """
        计算所有电机产生的滚转(x轴)和俯仰(y轴)总力矩
        
        Returns:
            tuple: (roll_torque, pitch_torque) 滚转和俯仰力矩 [Nm]
        """
        
        # 获取机体参考系
        rb = self.get_dc_interface().get_rigid_body(self._stage_prefix + "/body")
        
        # 获取所有电机
        rotors = [self.get_dc_interface().get_rigid_body(self._stage_prefix + "/rotor" + str(i)) 
                for i in range(self._thrusters._num_rotors)]
        
        # 获取电机相对位置
        relative_poses = self.get_dc_interface().get_relative_body_poses(rb, rotors)
        
        roll_torque = 0.0    # x轴力矩 - 滚转
        pitch_torque = 0.0   # y轴力矩 - 俯仰
        
        for i in range(self._thrusters._num_rotors):
            # 计算当前电机推力
            thrust_i = self._thrusters._rotor_constant[i] * np.power(self._thrusters._velocity[i], 2)
            
            # 获取电机位置坐标
            # relative_poses[i].p[0] = x坐标, relative_poses[i].p[1] = y坐标
            x_pos = relative_poses[i].p[0]   # 电机x坐标
            y_pos = relative_poses[i].p[1]   # 电机y坐标
            
            # 计算单个电机的力矩贡献
            # 滚转力矩(x轴): 推力 × y坐标
            roll_torque += thrust_i * y_pos
            
            # 俯仰力矩(y轴): -推力 × x坐标 (注意负号)
            pitch_torque += -thrust_i * x_pos
        
        return roll_torque, pitch_torque


    def force_and_torques_to_velocities(self, force: float, torque: np.ndarray):
        """
        Auxiliar method used to get the target angular velocities for each rotor, given the total desired thrust [N] and
        torque [Nm] to be applied in the multirotor's body frame.

        Note: This method assumes a quadratic thrust curve. This method will be improved in a future update,
        and a general thrust allocation scheme will be adopted. For now, it is made to work with multirotors directly.

        Args:
            force (np.ndarray): A vector of the force to be applied in the body frame of the vehicle [N]
            torque (np.ndarray): A vector of the torque to be applied in the body frame of the vehicle [Nm]

        Returns:
            list: A list of angular velocities [rad/s] to apply in reach rotor to accomplish suchs forces and torques
        """

        # Get the body frame of the vehicle
        rb = self.get_dc_interface().get_rigid_body(self._stage_prefix + "/body")

        # Get the rotors of the vehicle
        rotors = [self.get_dc_interface().get_rigid_body(self._stage_prefix + "/rotor" + str(i)) for i in range(self._thrusters._num_rotors)]

        # Get the relative position of the rotors with respect to the body frame of the vehicle (ignoring the orientation for now)
        relative_poses = self.get_dc_interface().get_relative_body_poses(rb, rotors)

        # Define the alocation matrix
        aloc_matrix = np.zeros((4, self._thrusters._num_rotors))
        
        # Define the first line of the matrix (T [N])
        aloc_matrix[0, :] = np.array(self._thrusters._rotor_constant)                                           

        # Define the second and third lines of the matrix (\tau_x [Nm] and \tau_y [Nm])
        aloc_matrix[1, :] = np.array([relative_poses[i].p[1] * self._thrusters._rotor_constant[i] for i in range(self._thrusters._num_rotors)])
        aloc_matrix[2, :] = np.array([-relative_poses[i].p[0] * self._thrusters._rotor_constant[i] for i in range(self._thrusters._num_rotors)])

        # Define the forth line of the matrix (\tau_z [Nm])
        aloc_matrix[3, :] = np.array([self._thrusters._rolling_moment_coefficient[i] * self._thrusters._rot_dir[i] for i in range(self._thrusters._num_rotors)])

        # Compute the inverse allocation matrix, so that we can get the angular velocities (squared) from the total thrust and torques
        aloc_inv = np.linalg.pinv(aloc_matrix)

        # Compute the target angular velocities (squared)
        squared_ang_vel = aloc_inv @ np.array([force, torque[0], torque[1], torque[2]])

        # Making sure that there is no negative value on the target squared angular velocities
        squared_ang_vel[squared_ang_vel < 0] = 0.0

        # ------------------------------------------------------------------------------------------------
        # Saturate the inputs while preserving their relation to each other, by performing a normalization
        # ------------------------------------------------------------------------------------------------
        max_thrust_vel_squared = np.power(self._thrusters.max_rotor_velocity[0], 2)
        max_val = np.max(squared_ang_vel)

        if max_val >= max_thrust_vel_squared:
            normalize = np.maximum(max_val / max_thrust_vel_squared, 1.0)

            squared_ang_vel = squared_ang_vel / normalize

        # Compute the angular velocities for each rotor in [rad/s]
        ang_vel = np.sqrt(squared_ang_vel)

        return ang_vel
    