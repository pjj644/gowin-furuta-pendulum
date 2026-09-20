"""
Rotary Inverted Pendulum (Furuta Pendulum) - J280 Competition Kit Configuration
全国大学生嵌入式芯片与系统设计竞赛 - 选题一：基于FPGA的实时姿态控制系统
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class PendulumConfig:
    # ==================== 1. 机械结构参数 (Mechanical Parameters) ====================
    # 水平旋转臂 (Rotary Arm) - 对标 J280 套件官方 FAQ 实测参数 (长 15.2cm, 宽 3.6cm, 重 90g)
    L1: float = 0.152         # 水平转臂轴心到摆杆转轴的有效回转半径 (m)
    m1: float = 0.090         # 水平转臂质量 (kg)
    J1: float = 0.000693      # 转臂绕电机轴的转动惯量 J1 = (1/3)*m1*L1^2 (kg*m^2)
    b1: float = 0.0010        # 转臂机械轴承黏性摩擦系数 (N*m*s/rad)
    coulomb_tau: float = 0.0020 # 转臂机械轴承库仑摩擦力矩 (N*m)

    # 垂直摆杆 (Inverted Pendulum) - 对标 J280 套件官方 FAQ 实测参数 (长 15cm, 宽 3.4cm, 重 90g)
    L2: float = 0.150         # 摆杆全长 (m)
    l2: float = 0.075         # 摆杆转轴到质心的距离 l2 = L2 / 2 (m)
    m2: float = 0.090         # 摆杆质量 (kg)
    Jp: float = 0.000675      # 摆杆绕转轴的转动惯量 Jp = (1/3)*m2*L2^2 (kg*m^2)
    b2: float = 0.00010       # 摆杆转轴角度传感器阻尼系数 (N*m*s/rad)

    # 重力加速度
    g: float = 9.81           # 重力加速度 (m/s^2)

    # ==================== 2. 执行机构与直流电机参数 (DC Motor & Driver) ====================
    V_max: float = 12.0       # 电机驱动电源供电电压 (V)
    R_m: float = 4.0          # 电机电枢电阻 (Ohm)
    K_t: float = 0.050        # 电机转矩常数 (N*m/A)
    K_b: float = 0.050        # 电机反电动势常数 (V*s/rad)
    gear_ratio: float = 1.0   # 传动减速比 (直接驱动为 1.0)
    motor_deadband: float = 0.25 # 电机死区电压 (V)，低于此电压电机静摩擦不动作

    # ==================== 3. 传感器与离散采样参数 (Sensors & FPGA Interface) ====================
    T_s: float = 0.001        # FPGA 控制算法执行周期 Ts = 1ms (1000 Hz 控制主频)
    dt_phys: float = 0.0002   # 物理动力学求解器积分时间步长 dt = 0.2ms (保障 RK4 高精度)
    enable_delay: bool = False # 是否开启 1 拍 (1ms) 控制执行延时 (z^-1)

    # 传感器量化配置 (模拟真实板卡 ADC 与编码器)
    enable_quantization: bool = True
    adc_bits: int = 12        # 角度传感器 ADC 采样位数 (12-bit, 4096 counts / 360 deg)
    encoder_lines: int = 1000 # 电机光电编码器线数 (4倍频后为 4000 CPR)
    sensor_noise_std: float = 0.001 # 角度传感器噪声标准差 (rad, 约 0.057 度)

    # ==================== 4. 控制器与状态机切换阈值 (Controller Thresholds) ====================
    # 平滑自适应能量起摆参数
    swing_smooth: bool = True  # 是否启用平滑连续能量泵 (False 则使用 Bang-Bang 阶跃泵)
    swing_ke: float = 14.0     # 能量泵能量差比例增益
    swing_gamma: float = 2.5   # 连续化平滑饱和因子
    swing_acc_amp: float = 30.0 # 起摆控制最大等效加速度限幅 (rad/s^2)

    # 切换至平衡模式的角度与角速度阈值
    balance_angle_thresh: float = np.radians(22.0)  # 摆杆与垂直倒立方向夹角小于 22 度时切换
    balance_omega_thresh: float = 4.0               # 摆杆角速度绝对值小于 4.0 rad/s 时切入

    # LQR / LQI 控制器配置
    use_lqi: bool = True      # 是否启用 LQI 积分消除静差 (True: 5状态LQI; False: 4状态LQR)
    use_fixed_point: bool = False # 是否启用 Q12.16 定点数逐比特硬件仿真模式

    # 状态权重矩阵 Q 与控制输入权重 R
    # 状态向量 x = [theta (摆角误差), dtheta (摆角速度), alpha (转臂位移误差), dalpha (转臂角速度), alpha_int (位移积分误差)]
    q_theta: float = 60.0     # 倒立摆直立角度误差惩罚 (最高优先级)
    q_dtheta: float = 16.0    # 倒立摆角速度惩罚 (针对 90g 重摆杆阻尼匹配，消除捕获过冲)
    q_alpha: float = 25.0     # 水平转臂位置误差惩罚 (强化刚度，克服 0.25V 电机死区)
    q_dalpha: float = 4.0     # 水平转臂角速度惩罚 (抑制前冲振荡)
    q_alpha_int: float = 8.0  # 水平转臂积分误差惩罚 (快速消除静差，K5 = -4.0)
    alpha_int_limit: float = 0.25 # 抗积分饱和积分限幅 (rad)
    r_u: float = 0.50         # 控制电压能量消耗惩罚

    # 连续轨迹跟踪配置 (拓展要求 2)
    traj_amp: float = np.radians(25.0) # 正弦轨迹幅值 (rad, 约 25 度)
    traj_freq: float = 0.20            # 正弦轨迹跟踪频率 (Hz, 5秒周期)

    def __post_init__(self):
        # 预计算复合常数
        self.J0 = self.J1 + self.m2 * (self.L1 ** 2)
        self.km = (self.gear_ratio * self.K_t) / self.R_m
        self.b_emf = (self.gear_ratio * self.K_t * self.K_b) / self.R_m
        self.b1_total = self.b1 + self.b_emf
        self.det_M0 = self.J0 * self.Jp - (self.m2 * self.L1 * self.l2) ** 2
