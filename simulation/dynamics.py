"""
Furuta Pendulum (Rotary Inverted Pendulum) Non-linear Dynamics & RK4 Integrator
全国大学生嵌入式芯片与系统设计竞赛 - 选题一
"""

import sys
import numpy as np

try:
    from simulation.config import PendulumConfig
except ImportError:
    from config import PendulumConfig


class FurutaDynamics:
    """
    非线性旋转倒立摆动力学模型
    广义坐标系定义：
      alpha (q0): 水平转臂角度 (rad), 沿垂直Z轴正方向逆时针为正
      dalpha (q1): 水平转臂角速度 (rad/s)
      theta (q2): 倒立摆摆杆角度 (rad), 0 为垂直倒立平衡点 (+Z轴), +/-pi 为自由下垂稳定点
      dtheta (q3): 倒立摆摆杆角速度 (rad/s)
    """

    def __init__(self, cfg: PendulumConfig = None):
        self.cfg = cfg if cfg is not None else PendulumConfig()
        # 初始状态: [alpha, dalpha, theta, dtheta]
        # 默认下垂状态: theta = pi (180度)
        self.state = np.array([0.0, 0.0, np.pi, 0.0], dtype=np.float64)

        # 瞬时外力矩扰动 (模拟人用手轻推摆杆)
        self.disturb_tau_pendulum = 0.0
        self.disturb_tau_arm = 0.0

        # 控制指令执行延迟队列 (模拟 FPGA 1ms 拍延时 z^-1)
        self.delay_v = 0.0

    def reset(self, alpha=0.0, dalpha=0.0, theta=np.pi, dtheta=0.0):
        """重置状态"""
        self.state = np.array([alpha, dalpha, theta, dtheta], dtype=np.float64)
        self.disturb_tau_pendulum = 0.0
        self.disturb_tau_arm = 0.0
        self.delay_v = 0.0

    def set_disturbance(self, tau_pendulum: float = 0.0, tau_arm: float = 0.0):
        """设置外部推力扰动"""
        self.disturb_tau_pendulum = tau_pendulum
        self.disturb_tau_arm = tau_arm

    def normalize_theta(self, theta: float) -> float:
        """将角度规范化至 [-pi, pi] 区间，0 代表垂直倒立"""
        return ((theta + np.pi) % (2.0 * np.pi)) - np.pi

    def compute_energy(self, state=None) -> float:
        """
        计算摆杆机械能相对倒立顶点的相对能量 (用于起摆能量泵算法)
        E = (1/2)*Jp*dtheta^2 + m2*g*l2*(cos(theta) - 1)
        倒立静止时 E = 0, 下垂静止时 E = -2*m2*g*l2
        """
        if state is None:
            state = self.state
        _, _, theta, dtheta = state
        th_norm = self.normalize_theta(theta)
        cfg = self.cfg
        E = 0.5 * cfg.Jp * (dtheta ** 2) + cfg.m2 * cfg.g * cfg.l2 * (np.cos(th_norm) - 1.0)
        return E

    def compute_total_energy(self, state=None) -> float:
        """
        计算旋转倒立摆完整多体系统的机械总能量 (系统动能 T + 重力势能 V)
        用于验证无阻尼状态下的动力学能量守恒
        """
        if state is None:
            state = self.state
        _, dalpha, theta, dtheta = state
        cfg = self.cfg
        th = self.normalize_theta(theta)
        T = (
            0.5 * (cfg.J0 + cfg.Jp * (np.sin(th) ** 2)) * (dalpha ** 2)
            + 0.5 * cfg.Jp * (dtheta ** 2)
            + cfg.m2 * cfg.L1 * cfg.l2 * np.cos(th) * dalpha * dtheta
        )
        V = cfg.m2 * cfg.g * cfg.l2 * np.cos(th)
        return T + V

    def forward_kinematics(self, state=None):
        """
        正运动学解算：用于 3D/2D 空间可视化
        返回:
          p_base: 基座原点 [0, 0, 0]
          p_arm: 水平转臂末端 (也是摆杆轴心) [x1, y1, z1]
          p_pend_cm: 摆杆质心坐标 [x2, y2, z2]
          p_pend_tip: 摆杆末梢端点 [x3, y3, z3]
        """
        if state is None:
            state = self.state
        alpha, _, theta, _ = state
        cfg = self.cfg

        # 水平转臂末端 (在水平 XY 平面内绕 Z 轴回转)
        x_arm = cfg.L1 * np.cos(alpha)
        y_arm = cfg.L1 * np.sin(alpha)
        z_arm = 0.0
        p_arm = np.array([x_arm, y_arm, z_arm])

        # 摆杆转轴沿着转臂径向，摆杆摆动方向为切向
        # 径向单位矢量: r_hat = [cos(alpha), sin(alpha), 0]
        # 切向单位矢量: phi_hat = [-sin(alpha), cos(alpha), 0]
        # 竖直单位矢量: z_hat = [0, 0, 1]
        phi_hat = np.array([-np.sin(alpha), np.cos(alpha), 0.0])
        z_hat = np.array([0.0, 0.0, 1.0])

        # 摆杆轴线方向矢量
        pend_dir = np.sin(theta) * phi_hat + np.cos(theta) * z_hat

        p_base = np.array([0.0, 0.0, 0.0])
        p_pend_cm = p_arm + cfg.l2 * pend_dir
        p_pend_tip = p_arm + cfg.L2 * pend_dir

        return p_base, p_arm, p_pend_cm, p_pend_tip

    def derivatives(self, state: np.ndarray, V_motor: float) -> np.ndarray:
        """
        求解非线性动力学状态导数: d/dt [alpha, dalpha, theta, dtheta]
        """
        alpha, dalpha, theta, dtheta = state
        cfg = self.cfg

        # 角度规范化与三角函数计算
        th = self.normalize_theta(theta)
        sin_th = np.sin(th)
        cos_th = np.cos(th)
        sin_2th = np.sin(2.0 * th)

        # 1. 电机电磁力矩模型 (考虑反电动势阻尼与供电饱和)
        V_eff = np.clip(V_motor, -cfg.V_max, cfg.V_max)
        # 电机死区效应
        if abs(V_eff) < cfg.motor_deadband:
            V_eff = 0.0
        tau_motor = cfg.km * V_eff - cfg.b_emf * dalpha + self.disturb_tau_arm

        # 轴承库仑静摩擦 (平滑连续可微模型，避免数值抖振)
        if cfg.coulomb_tau > 0.0:
            tau_coulomb = cfg.coulomb_tau * np.tanh(50.0 * dalpha)
        else:
            tau_coulomb = 0.0

        # 2. 广义质量惯性矩阵 M(q)
        M11 = cfg.J0 + cfg.Jp * (sin_th ** 2)
        M12 = cfg.m2 * cfg.L1 * cfg.l2 * cos_th
        M21 = M12
        M22 = cfg.Jp

        # 3. 广义力与外力矩项 (包含科氏力、向心力、重力力矩、机械黏性与库仑阻尼与外加扰动力矩)
        F1 = (
            tau_motor
            - cfg.b1 * dalpha
            - tau_coulomb
            - cfg.Jp * sin_2th * dalpha * dtheta
            + cfg.m2 * cfg.L1 * cfg.l2 * sin_th * (dtheta ** 2)
        )
        F2 = (
            cfg.m2 * cfg.g * cfg.l2 * sin_th
            + 0.5 * cfg.Jp * sin_2th * (dalpha ** 2)
            - cfg.b2 * dtheta
            + self.disturb_tau_pendulum
        )

        # 4. 求解广义加速度 [ddalpha, ddtheta]^T = M^-1 * [F1, F2]^T
        det_M = M11 * M22 - M12 * M21
        ddalpha = (M22 * F1 - M12 * F2) / det_M
        ddtheta = (-M21 * F1 + M11 * F2) / det_M

        return np.array([dalpha, ddalpha, dtheta, ddtheta], dtype=np.float64)

    def step_rk4(self, V_motor: float, dt: float = None):
        """
        四阶龙格-库塔数值积分单步更新 (RK4)
        """
        if dt is None:
            dt = self.cfg.dt_phys

        # 控制指令 1 拍延时处理 (若使能)
        if self.cfg.enable_delay:
            V_applied = self.delay_v
            self.delay_v = V_motor
        else:
            V_applied = V_motor

        y = self.state
        k1 = self.derivatives(y, V_applied)
        k2 = self.derivatives(y + 0.5 * dt * k1, V_applied)
        k3 = self.derivatives(y + 0.5 * dt * k2, V_applied)
        k4 = self.derivatives(y + dt * k3, V_applied)

        self.state = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return self.state
