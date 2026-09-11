"""
Discrete Controller for Furuta Pendulum (FPGA Hardware-in-Loop Emulation)
包含：
  1. 纯 NumPy LQR / LQI 状态反馈增益求解器 (无需 scipy 依赖，支持积分消除静差)
  2. 传感器 12-bit ADC / 编码器量化与噪声模拟
  3. 角速度滤波估算器 (一阶低通 IIR 滤波器与防突跳去卷绕)
  4. 平滑自适应 Lyapunov 能量起摆 (Swing-up) 控制器
  5. 连续轨迹与速度跟踪生成器 (竞赛拓展要求 2)
  6. FPGA Q12.16 逐比特定点数硬件仿真引擎
  7. 状态机切换逻辑 (Hanging -> Swing-up -> Balance / Setpoint / Trajectory Tracking)
"""

import sys
import numpy as np

try:
    from simulation.config import PendulumConfig
except ImportError:
    from config import PendulumConfig


def solve_care_numpy(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray):
    """
    使用哈密顿矩阵特征值分解法求解连续代数黎卡提方程 (CARE)
    A^T * P + P * A - P * B * R^-1 * B^T * P + Q = 0
    返回: 最优反馈增益 K = R^-1 * B^T * P
    """
    n = A.shape[0]
    R_inv = np.linalg.inv(R)
    BRB = B @ R_inv @ B.T
    H = np.block([[A, -BRB], [-Q, -A.T]])

    eigvals, eigvecs = np.linalg.eig(H)
    # 选取具有负实部的稳定特征值
    stable_indices = [i for i, val in enumerate(eigvals) if val.real < 0]
    if len(stable_indices) != n:
        raise RuntimeError(f"LQR 求解失败：预期间包含 {n} 个稳定特征值，实际得到 {len(stable_indices)}")

    V_stable = eigvecs[:, stable_indices]
    U1 = V_stable[:n, :]
    U2 = V_stable[n:, :]

    P = np.real(U2 @ np.linalg.inv(U1))
    P = (P + P.T) / 2.0  # 保证对称正定
    K = np.real(R_inv @ B.T @ P)
    return K, P


class DiscreteFPGAController:
    """
    FPGA 离散数字控制器
    模拟硬件定时器 1ms 周期中断触发的控制流程
    """

    STATE_HANGING = 0
    STATE_SWINGUP = 1
    STATE_BALANCE = 2

    def __init__(self, cfg: PendulumConfig = None):
        self.cfg = cfg if cfg is not None else PendulumConfig()

        # 状态机模式
        self.mode = self.STATE_HANGING

        # 目标转臂位置 (rad)
        self.alpha_target = 0.0
        self.alpha_int = 0.0  # LQI 积分状态累加器 (rad*s)

        # 轨迹跟踪模式: 'setpoint' (定点) 或 'sine' (连续正弦)
        self.tracking_mode = "setpoint"
        self.traj_time = 0.0

        # 上一次采样值 (用于角速度离散差分)
        self.prev_raw_theta = np.pi
        self.prev_raw_alpha = 0.0

        # 滤波后状态估算值
        self.est_theta = np.pi
        self.est_dtheta = 0.0
        self.est_alpha = 0.0
        self.est_dalpha = 0.0

        # 一阶低通滤波器滤波系数 (截止频率 ~ 40Hz)
        self.filter_alpha = 0.35

        # 逐比特定点数模式开关
        self.use_fixed_point = self.cfg.use_fixed_point
        self.SCALE_Q16 = 65536

        # 求解控制增益
        self.K_gain = self._compute_controller_gain()
        self.K_fixed_q16 = [int(np.round(k * self.SCALE_Q16)) for k in self.K_gain]
        ctrl_name = "LQI (5状态含积分)" if self.cfg.use_lqi else "LQR (4状态)"
        print(f"[Controller] {ctrl_name} 最优反馈增益矩阵 K 计算完成: {np.round(self.K_gain, 4)}")

    @property
    def K_lqr(self) -> np.ndarray:
        """向后兼容属性"""
        return self.K_gain

    def _compute_controller_gain(self) -> np.ndarray:
        """
        在垂直倒立平衡点线性化状态方程
        若 use_lqi=True: 5 状态 x = [theta, dtheta, alpha, dalpha, alpha_int]^T
        若 use_lqi=False: 4 状态 x = [theta, dtheta, alpha, dalpha]^T
        """
        cfg = self.cfg
        D = cfg.det_M0

        # 基础线性化系统矩阵 A (4x4)
        A = np.zeros((4, 4), dtype=np.float64)
        A[0, 1] = 1.0
        A[1, 0] = (cfg.J0 * cfg.m2 * cfg.g * cfg.l2) / D
        A[1, 1] = (-cfg.J0 * cfg.b2) / D
        A[1, 3] = (cfg.m2 * cfg.L1 * cfg.l2 * cfg.b1_total) / D

        A[2, 3] = 1.0
        A[3, 0] = (-cfg.m2 * cfg.L1 * cfg.l2 * cfg.m2 * cfg.g * cfg.l2) / D
        A[3, 1] = (cfg.m2 * cfg.L1 * cfg.l2 * cfg.b2) / D
        A[3, 3] = (-cfg.Jp * cfg.b1_total) / D

        # 输入矩阵 B (4x1)
        B = np.zeros((4, 1), dtype=np.float64)
        B[1, 0] = (-cfg.m2 * cfg.L1 * cfg.l2 * cfg.km) / D
        B[3, 0] = (cfg.Jp * cfg.km) / D

        if cfg.use_lqi:
            # 5 维增广系统，第 5 状态为转臂位置误差的积分 e_I: d(e_I)/dt = alpha - alpha_target
            A_aug = np.zeros((5, 5), dtype=np.float64)
            A_aug[:4, :4] = A
            A_aug[4, 2] = 1.0  # d(alpha_int)/dt = alpha

            B_aug = np.zeros((5, 1), dtype=np.float64)
            B_aug[:4, :] = B

            Q_aug = np.diag([cfg.q_theta, cfg.q_dtheta, cfg.q_alpha, cfg.q_dalpha, cfg.q_alpha_int])
            R = np.array([[cfg.r_u]])

            K, _ = solve_care_numpy(A_aug, B_aug, Q_aug, R)
            return K.flatten()
        else:
            Q = np.diag([cfg.q_theta, cfg.q_dtheta, cfg.q_alpha, cfg.q_dalpha])
            R = np.array([[cfg.r_u]])
            K, _ = solve_care_numpy(A, B, Q, R)
            return K.flatten()

    def set_target_position(self, alpha_deg: float):
        """设置转臂目标定位角度 (度) 并恢复定点跟踪模式"""
        self.tracking_mode = "setpoint"
        self.alpha_target = np.radians(alpha_deg)

    def set_trajectory_mode(self, mode: str = "sine"):
        """启用连续轨迹与速度跟踪模式 (拓展要求 2)"""
        self.tracking_mode = mode
        self.traj_time = 0.0

    def toggle_fixed_point_mode(self) -> bool:
        """切换 Q12.16 定点数硬件仿真模式"""
        self.use_fixed_point = not self.use_fixed_point
        return self.use_fixed_point

    def reset(self, initial_theta: float = np.pi, initial_alpha: float = 0.0):
        """复位控制器状态"""
        self.mode = self.STATE_HANGING
        self.alpha_target = initial_alpha
        self.alpha_int = 0.0
        self.tracking_mode = "setpoint"
        self.traj_time = 0.0
        self.prev_raw_theta = initial_theta
        self.prev_raw_alpha = initial_alpha
        self.est_theta = initial_theta
        self.est_dtheta = 0.0
        self.est_alpha = initial_alpha
        self.est_dalpha = 0.0

    def quantize_sensors(self, true_alpha: float, true_theta: float):
        """
        模拟实际 FPGA 外围硬件采集：
          1. 角度传感器 12-bit ADC 量化与高斯噪声
          2. 光电编码器离散脉冲计数
        """
        cfg = self.cfg
        if not cfg.enable_quantization:
            return true_alpha, true_theta

        # 1. 角度传感器 ADC 量化 (12-bit, 4096 刻度对应 360 度)
        noise = np.random.normal(0.0, cfg.sensor_noise_std)
        theta_with_noise = true_theta + noise
        adc_res = 2.0 * np.pi / (2 ** cfg.adc_bits)
        quant_theta = np.round(theta_with_noise / adc_res) * adc_res

        # 2. 编码器量化 (4000 CPR 脉冲计数)
        enc_res = 2.0 * np.pi / (cfg.encoder_lines * 4)
        quant_alpha = np.round(true_alpha / enc_res) * enc_res

        return quant_alpha, quant_theta

    def update_state_estimator(self, raw_alpha: float, raw_theta: float, dt: float):
        """
        状态估算器：根据采样角度计算平滑角速度
        带去卷绕与防 +/-pi 边界假性突跳保护，配合一阶 IIR 滤波
        """
        # 摆角角度去卷绕 [-pi, pi]
        norm_theta = ((raw_theta + np.pi) % (2.0 * np.pi)) - np.pi
        norm_prev_theta = ((self.prev_raw_theta + np.pi) % (2.0 * np.pi)) - np.pi

        # 使用最短角位移差分求角速度
        dth_diff = ((norm_theta - norm_prev_theta + np.pi) % (2.0 * np.pi)) - np.pi
        diff_dtheta = dth_diff / dt
        diff_dalpha = (raw_alpha - self.prev_raw_alpha) / dt

        # 一阶低通数字滤波平滑
        beta = self.filter_alpha
        self.est_dtheta = (1.0 - beta) * self.est_dtheta + beta * diff_dtheta
        self.est_dalpha = (1.0 - beta) * self.est_dalpha + beta * diff_dalpha

        self.est_theta = norm_theta
        self.est_alpha = raw_alpha

        self.prev_raw_theta = raw_theta
        self.prev_raw_alpha = raw_alpha

    def get_reference_trajectory(self, dt: float) -> tuple:
        """
        生成当前时刻的目标参考位置与目标参考角速度
        返回: (alpha_ref, dalpha_ref)
        """
        if self.tracking_mode == "sine":
            self.traj_time += dt
            omega = 2.0 * np.pi * self.cfg.traj_freq
            alpha_ref = self.cfg.traj_amp * np.sin(omega * self.traj_time)
            dalpha_ref = self.cfg.traj_amp * omega * np.cos(omega * self.traj_time)
            self.alpha_target = alpha_ref
            return alpha_ref, dalpha_ref
        else:
            return self.alpha_target, 0.0

    def compute_control_voltage(self, true_state: np.ndarray, dt: float) -> tuple:
        """
        1ms 周期控制算法执行主入口
        返回: (V_cmd, current_mode_str, est_states)
        """
        cfg = self.cfg
        true_alpha, _, true_theta, _ = true_state

        # 1. 模拟传感器采集与量化
        raw_alpha, raw_theta = self.quantize_sensors(true_alpha, true_theta)

        # 2. 状态估算与低通滤波
        self.update_state_estimator(raw_alpha, raw_theta, dt)

        th = self.est_theta
        dth = self.est_dtheta
        alpha = self.est_alpha
        dalpha = self.est_dalpha

        # 获取参考轨迹
        alpha_ref, dalpha_ref = self.get_reference_trajectory(dt)

        # 3. 计算相对机械总能量
        E = 0.5 * cfg.Jp * (dth ** 2) + cfg.m2 * cfg.g * cfg.l2 * (np.cos(th) - 1.0)

        just_switched_to_balance = False
        if self.mode == self.STATE_HANGING:
            # 初始给予微小激励打破对称性死点
            self.mode = self.STATE_SWINGUP
            V_cmd = 3.0
            mode_str = "HANGING"

        elif self.mode == self.STATE_SWINGUP:
            # 检查是否进入倒立平衡捕获窗口
            if abs(th) < cfg.balance_angle_thresh and abs(dth) < cfg.balance_omega_thresh:
                self.mode = self.STATE_BALANCE
                just_switched_to_balance = True
                self.alpha_int = 0.0  # 进入平衡时清零积分器
                mode_str = "SWITCH_TO_BAL"
            else:
                mode_str = "SWING_UP"

            if self.mode == self.STATE_SWINGUP:
                if cfg.swing_smooth:
                    # 连续自适应平滑能量泵 (tanh 连续过渡，消除硬阶跃跳变，大幅减小机械抖振)
                    if E < 0:
                        smooth_sign = np.tanh(20.0 * dth * np.cos(th)) if abs(dth) > 0.03 else 0.0
                        if abs(smooth_sign) < 0.05 and abs(th) > 2.5:
                            a_pump = 5.0
                        else:
                            a_pump = -cfg.swing_acc_amp * smooth_sign
                    else:
                        a_pump = 0.0
                    # 叠加转臂居中限位阻尼，防止起摆无限漂移
                    a_pump -= (1.0 * alpha + 0.3 * dalpha)
                    a_pump = np.clip(a_pump, -cfg.swing_acc_amp, cfg.swing_acc_amp)
                    V_cmd = (cfg.J0 * a_pump) / cfg.km
                else:
                    # 经典 Bang-Bang 阶跃能量泵
                    if E < 0:
                        a_pump = -cfg.swing_acc_amp * np.sign(dth * np.cos(th)) if abs(dth) > 0.05 else 5.0
                    else:
                        a_pump = 0.0
                    a_pump -= (1.0 * alpha + 0.3 * dalpha)
                    V_cmd = (cfg.J0 * a_pump) / cfg.km

        if self.mode == self.STATE_BALANCE:
            # 倒立自平衡与转臂定点/轨迹跟踪
            if abs(th) > np.radians(45.0):
                # 极端冲击倾倒保护，自动切回重新起摆
                self.mode = self.STATE_SWINGUP
                self.alpha_int = 0.0
                mode_str = "FALL_RE_SWING"
                V_cmd = 0.0
            else:
                if not just_switched_to_balance:
                    mode_str = "BALANCE" if self.tracking_mode == "setpoint" else "TRACKING"

                # 计算误差
                alpha_err = alpha - alpha_ref
                dalpha_err = dalpha - dalpha_ref

                # 积分分离与抗饱和：当处于线性捕获区内时启用积分以消除死区静差，避免大阶跃阶段过度超调
                if cfg.use_lqi:
                    if abs(alpha_err) < np.radians(10.0):
                        self.alpha_int += alpha_err * dt
                        self.alpha_int = np.clip(self.alpha_int, -cfg.alpha_int_limit, cfg.alpha_int_limit)
                    else:
                        self.alpha_int = 0.0

                if self.use_fixed_point:
                    # =========================================================
                    # FPGA Q12.16 逐比特定点数硬件仿真分支 (Bit-Exact RTL Model)
                    # =========================================================
                    S = self.SCALE_Q16
                    th_q16 = int(np.round(th * S))
                    dth_q16 = int(np.round(dth * S))
                    al_err_q16 = int(np.round(alpha_err * S))
                    dal_err_q16 = int(np.round(dalpha_err * S))

                    if cfg.use_lqi:
                        int_err_q16 = int(np.round(self.alpha_int * S))
                        q_vec = [th_q16, dth_q16, al_err_q16, dal_err_q16, int_err_q16]
                    else:
                        q_vec = [th_q16, dth_q16, al_err_q16, dal_err_q16]

                    # 硬件流水线乘加 (64位有符号累加器)
                    prod_sum = sum(x * k for x, k in zip(q_vec, self.K_fixed_q16))
                    v_cmd_q16 = -(prod_sum >> 16)

                    # 映射至 PWM 占空比 [-1000, 1000] (Q16 * Q16 -> Q32, 右移 32 位获取整数占空比)
                    VOLT_TO_PWM = 5461163  # (1000 / 12) * 65536
                    pwm_calc = (v_cmd_q16 * VOLT_TO_PWM) >> 32
                    pwm_duty = int(np.clip(pwm_calc, -1000, 1000))
                    V_cmd = pwm_duty * (12.0 / 1000.0)
                else:
                    # =========================================================
                    # 浮点数最优状态反馈控制
                    # =========================================================
                    if cfg.use_lqi:
                        x_err = np.array([th, dth, alpha_err, dalpha_err, self.alpha_int], dtype=np.float64)
                    else:
                        x_err = np.array([th, dth, alpha_err, dalpha_err], dtype=np.float64)

                    V_cmd = -float(np.dot(self.K_gain, x_err))

        # 5. 电压供电限幅
        V_clipped = np.clip(V_cmd, -cfg.V_max, cfg.V_max)

        return V_clipped, mode_str, (th, dth, alpha, dalpha, E)
