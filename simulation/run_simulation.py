"""
Interactive Physical Simulation & Visualization for Furuta Pendulum
全国大学生嵌入式芯片与系统设计竞赛 - 选题一：基于FPGA的实时姿态控制系统

运行特性：
  1. 动态双连杆空间姿态渲染与实时曲线追踪
  2. 交互按键：
     - 'd' / 'D': 施加瞬时推力扰动脉冲 (模拟外力轻推摆杆)
     - '1' / '2' / '3': 切换转臂目标定点位置 (0度 / +45度 / -45度)
     - '4' / 'T': 开启/切换连续正弦轨迹与速度跟踪 (拓展要求 2)
     - '5' / 'F': 切换 FPGA Q12.16 逐比特定点数硬件仿真 / 浮点模式
     - 'r' / 'R': 重置状态至下垂重新起摆
     - 'q' / 'Q': 退出并保存高清性能曲线图
"""

import sys
import io
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 自动处理路径与 Windows 编码保护
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if base_dir not in sys.path:
    sys.path.insert(0, base_dir)

if sys.platform == "win32":
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    except Exception:
        pass

# 统一中文字体设置
plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "Arial Unicode MS", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

try:
    from simulation.config import PendulumConfig
    from simulation.dynamics import FurutaDynamics
    from simulation.controller import DiscreteFPGAController
except ImportError:
    from config import PendulumConfig
    from dynamics import FurutaDynamics
    from controller import DiscreteFPGAController


class InteractiveSimulation:
    def __init__(self, cfg: PendulumConfig = None):
        self.cfg = cfg if cfg is not None else PendulumConfig()
        self.dyn = FurutaDynamics(self.cfg)
        self.ctrl = DiscreteFPGAController(self.cfg)

        # 仿真时钟与记录数据
        self.sim_time = 0.0
        self.history_t = []
        self.history_theta = []
        self.history_alpha = []
        self.history_alpha_tgt = []
        self.history_voltage = []
        self.history_energy = []
        self.history_mode = []

        # 交互扰动标志
        self.disturb_counter = 0

    def reset(self):
        self.dyn.reset()
        self.ctrl.reset(initial_theta=np.pi, initial_alpha=0.0)
        self.sim_time = 0.0
        self.disturb_counter = 0
        self.history_t.clear()
        self.history_theta.clear()
        self.history_alpha.clear()
        self.history_alpha_tgt.clear()
        self.history_voltage.clear()
        self.history_energy.clear()
        self.history_mode.clear()

    def step(self):
        """执行单步控制周期 (1ms) 与多步物理积分"""
        cfg = self.cfg
        # 处理按键触发的瞬时扰动力矩 (持续 50ms)
        if self.disturb_counter > 0:
            self.dyn.set_disturbance(tau_pendulum=0.045)
            self.disturb_counter -= 1
        else:
            self.dyn.set_disturbance(tau_pendulum=0.0)

        # 1. FPGA 离散控制器运算
        V_cmd, mode_str, est_info = self.ctrl.compute_control_voltage(self.dyn.state, cfg.T_s)

        # 2. 物理动力学 RK4 多子步积分
        n_sub = int(np.round(cfg.T_s / cfg.dt_phys))
        for _ in range(n_sub):
            self.dyn.step_rk4(V_motor=V_cmd, dt=cfg.dt_phys)

        self.sim_time += cfg.T_s

        # 3. 记录历史轨迹
        th_deg = np.degrees(self.dyn.normalize_theta(self.dyn.state[2]))
        alpha_deg = np.degrees(self.dyn.state[0])
        alpha_tgt_deg = np.degrees(self.ctrl.alpha_target)
        energy = self.dyn.compute_energy()

        self.history_t.append(self.sim_time)
        self.history_theta.append(th_deg)
        self.history_alpha.append(alpha_deg)
        self.history_alpha_tgt.append(alpha_tgt_deg)
        self.history_voltage.append(V_cmd)
        self.history_energy.append(energy)
        self.history_mode.append(mode_str)

        return mode_str, V_cmd, th_deg, alpha_deg

    def run_batch(self, duration=8.0):
        """
        非交互批量运行 (用于自动生成涵盖全赛题全部要求的综合评估图表)
        时序规划：
          0.0s ~ 3.5s: 下垂自适应平滑起摆至倒立平衡 (基础要求 1)
          3.5s ~ 3.55s: 注入 50ms 外力轻推脉冲扰动 (0.045 N*m) 并自恢复 (基础要求 2, 3)
          4.6s ~ 6.0s: 切换转臂定点目标至 +40.0 度，验证 LQI 零静差 (拓展要求 1, 3)
          6.0s ~ 8.0s: 启动连续正弦轨迹与速度跟踪，验证动态直立保持 (拓展要求 2)
        """
        print(f"[Simulation] 正在运行全流程物理仿真 (全赛题覆盖时长: {duration}s)...")
        steps = int(duration / self.cfg.T_s)
        disturb_triggered = False
        step_pos_triggered = False
        sine_traj_triggered = False

        for i in range(steps):
            t = i * self.cfg.T_s

            # 阶段 2: 注入测试扰动 (单次触发 50ms 脉冲)
            if t >= 3.5 and not disturb_triggered:
                disturb_triggered = True
                self.disturb_counter = int(0.05 / self.cfg.T_s)
                print(f"  -> 在 t = {t:.2f}s 注入瞬时推力扰动脉冲 (模拟外力轻推，持续 50ms)！")

            # 阶段 3: 注入定点阶跃目标位置 (单次触发)
            if t >= 4.6 and not step_pos_triggered:
                step_pos_triggered = True
                self.ctrl.set_target_position(40.0)
                print(f"  -> 在 t = {t:.2f}s 切换转臂目标位置至 +40.0 度 (LQI 零静差定点控制)！")

            # 阶段 4: 启动连续正弦轨迹与速度跟踪 (拓展要求 2)
            if t >= 6.0 and not sine_traj_triggered:
                sine_traj_triggered = True
                self.ctrl.set_trajectory_mode("sine")
                print(f"  -> 在 t = {t:.2f}s 启用连续正弦轨迹与速度跟踪模式 (拓展要求 2)！")

            self.step()

        print("[Simulation] 仿真计算完毕，正在绘制高质量综合评估曲线...")


def generate_evaluation_plots(sim: InteractiveSimulation, output_path="simulation/simulation_report.png"):
    """绘制并保存包含全部赛题基础与拓展指标的高清分析报告图"""
    t = np.array(sim.history_t)
    theta = np.array(sim.history_theta)
    alpha = np.array(sim.history_alpha)
    alpha_tgt = np.array(sim.history_alpha_tgt)
    voltage = np.array(sim.history_voltage)
    energy = np.array(sim.history_energy)

    fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    fig.suptitle("全国大学生嵌入式芯片与系统设计竞赛 - 选题一（旋转倒立摆实时姿态控制系统仿真综合评估报告）", fontsize=12, fontweight="bold", y=0.98)

    # 1. 摆杆倾角曲线
    ax1 = axs[0]
    ax1.plot(t, theta, color="#D32F2F", linewidth=1.5, label=r"摆杆角度 $\theta$ (deg)")
    ax1.axhline(0, color="gray", linestyle="--", alpha=0.6)
    ax1.axhspan(-15, 15, color="#E8F5E9", alpha=0.5, label="线性平衡捕获区 (±15°)")
    ax1.set_ylabel("摆杆倾角 (°)", fontsize=10)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper right")
    ax1.set_title("【基础要求 1, 2, 3】平滑自适应起摆、0° 倒立平衡自锁与抗外力推力扰动快速恢复", fontsize=10, loc="left")

    # 2. 水平转臂位置与轨迹跟踪曲线
    ax2 = axs[1]
    ax2.plot(t, alpha, color="#1976D2", linewidth=1.5, label=r"实际转臂位置 $\alpha$ (deg)")
    ax2.plot(t, alpha_tgt, color="#E65100", linestyle="--", linewidth=1.4, label=r"目标轨迹 $\alpha_{cmd}$ (deg)")
    ax2.set_ylabel("转臂角度 (°)", fontsize=10)
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="upper right")
    ax2.set_title("【拓展要求 1, 2, 3】转臂 LQI 零静差定点伺服 (40°) 与连续正弦动态轨迹/速度高保真跟踪", fontsize=10, loc="left")

    # 3. 电机控制电压与 PWM 饱和状态
    ax3 = axs[2]
    ax3.plot(t, voltage, color="#7B1FA2", linewidth=1.1, label=r"电机驱动电压 $V_{motor}$ (V)")
    ax3.axhline(12, color="red", linestyle=":", label="供电饱和上限 (+12V)")
    ax3.axhline(-12, color="red", linestyle=":", label="供电饱和下限 (-12V)")
    ax3.set_ylabel("控制电压 (V)", fontsize=10)
    ax3.grid(True, linestyle=":", alpha=0.6)
    ax3.legend(loc="upper right")
    ax3.set_title("【硬件驱动特性】FPGA 定点流水线输出电压与 PWM 响应 (平滑连续无硬抖振)", fontsize=10, loc="left")

    # 4. 摆杆相对能量与模式收敛
    ax4 = axs[3]
    ax4.plot(t, energy, color="#388E3C", linewidth=1.3, label="相对机械能 E (J, 倒立平衡目标 = 0)")
    ax4.axhline(0, color="black", linestyle="--", alpha=0.6)
    ax4.set_xlabel("仿真时间 (s)", fontsize=10)
    ax4.set_ylabel("相对机械能 (J)", fontsize=10)
    ax4.grid(True, linestyle=":", alpha=0.6)
    ax4.legend(loc="upper right")
    ax4.set_title("【能量收敛特性】Lyapunov 能量泵快速泵浦至临界倒立势能态 (收敛平滑度提升)", fontsize=10, loc="left")

    plt.tight_layout(rect=[0, 0.02, 1, 0.96])
    plt.savefig(output_path, dpi=200)
    plt.close()
    print(f"[Simulation] 综合评估图表已成功保存至: {output_path}")


def launch_gui():
    """启动交互式动态动画窗口"""
    sim = InteractiveSimulation()
    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.2, 1.0])

    # 左侧 3D 姿态示意图
    ax_3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_3d.set_title("J280 旋转倒立摆空间姿态仿真\n[按键: D-轻推 | 1/2/3-切定点 | 4/T-正弦轨迹 | 5/F-定点数切换 | R-重置 | Q-退出]", fontsize=10)
    ax_3d.set_xlim(-0.35, 0.35)
    ax_3d.set_ylim(-0.35, 0.35)
    ax_3d.set_zlim(-0.25, 0.35)
    ax_3d.set_xlabel("X (m)")
    ax_3d.set_ylabel("Y (m)")
    ax_3d.set_zlabel("Z (m)")

    # 连杆渲染线段
    (line_base,) = ax_3d.plot([0], [0], [0], "ko", markersize=8, label="底座回转轴")
    (line_arm,) = ax_3d.plot([], [], [], "b-", linewidth=4, label="水平转臂")
    (line_pend,) = ax_3d.plot([], [], [], "r-", linewidth=3, label="倒立摆杆")
    (pt_tip,) = ax_3d.plot([], [], [], "ro", markersize=6)
    ax_3d.legend(loc="upper right")

    # 右侧实时监控曲线
    ax_th = fig.add_subplot(gs[0, 1])
    ax_th.set_ylabel(r"摆角 $\theta$ (°)")
    (line_th_curve,) = ax_th.plot([], [], "r-", linewidth=1.2)
    ax_th.grid(True, linestyle=":")

    ax_al = fig.add_subplot(gs[1, 1])
    ax_al.set_ylabel(r"转臂 $\alpha$ (°)")
    (line_al_curve,) = ax_al.plot([], [], "b-", linewidth=1.2, label="实际")
    (line_tgt_curve,) = ax_al.plot([], [], "k--", linewidth=1.0, label="目标")
    ax_al.legend(loc="upper right", fontsize=8)
    ax_al.grid(True, linestyle=":")

    ax_v = fig.add_subplot(gs[2, 1])
    ax_v.set_ylabel(r"电压 $V$ (V)")
    ax_v.set_xlabel("时间 (s)")
    (line_v_curve,) = ax_v.plot([], [], "m-", linewidth=1.0)
    ax_v.grid(True, linestyle=":")

    status_text = ax_3d.text2D(0.05, 0.95, "", transform=ax_3d.transAxes, fontsize=9,
                                bbox=dict(boxstyle="round", facecolor="white", alpha=0.85))

    # 按键响应函数
    def on_key(event):
        key = event.key.lower() if event.key else ""
        if key == "d":
            sim.disturb_counter = int(0.05 / sim.cfg.T_s)  # 50ms 脉冲扰动
            print(">>> 交互事件：施加推力扰动！")
        elif key == "1":
            sim.ctrl.set_target_position(0.0)
            print(">>> 交互事件：设定转臂目标角度 0°")
        elif key == "2":
            sim.ctrl.set_target_position(45.0)
            print(">>> 交互事件：设定转臂目标角度 +45°")
        elif key == "3":
            sim.ctrl.set_target_position(-45.0)
            print(">>> 交互事件：设定转臂目标角度 -45°")
        elif key in ["4", "t"]:
            sim.ctrl.set_trajectory_mode("sine")
            print(">>> 交互事件：启用连续正弦轨迹跟踪模式 (拓展要求 2)")
        elif key in ["5", "f"]:
            is_fx = sim.ctrl.toggle_fixed_point_mode()
            mode_desc = "FPGA Q12.16 逐比特定点数模式" if is_fx else "浮点高精度模式"
            print(f">>> 交互事件：已切换为 [{mode_desc}]")
        elif key == "r":
            sim.reset()
            print(">>> 交互事件：重置系统状态！")
        elif key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(frame):
        # 每次刷新执行 10 个 1ms 控制步 (约 10ms 仿真时间)
        for _ in range(10):
            mode, V_cmd, th_deg, al_deg = sim.step()

        p_base, p_arm, _, p_tip = sim.dyn.forward_kinematics()

        # 更新 3D 机械连杆
        line_arm.set_data([p_base[0], p_arm[0]], [p_base[1], p_arm[1]])
        line_arm.set_3d_properties([p_base[2], p_arm[2]])

        line_pend.set_data([p_arm[0], p_tip[0]], [p_arm[1], p_tip[1]])
        line_pend.set_3d_properties([p_arm[2], p_tip[2]])

        # 变色提示：平衡/跟踪为绿色，起摆为红色
        if mode in ["BALANCE", "TRACKING"]:
            line_pend.set_color("#2E7D32")
        else:
            line_pend.set_color("#D32F2F")

        pt_tip.set_data([p_tip[0]], [p_tip[1]])
        pt_tip.set_3d_properties([p_tip[2]])

        math_mode = "Q12.16定点" if sim.ctrl.use_fixed_point else "Float64"
        traj_info = "连续正弦跟踪" if sim.ctrl.tracking_mode == "sine" else "定点伺服"

        status_text.set_text(
            f"时间: {sim.sim_time:.2f}s | 状态: {mode}\n"
            f"算法内核: {math_mode} | 任务: {traj_info}\n"
            f"摆杆角度: {th_deg:.1f}° | 转臂角度: {al_deg:.1f}°\n"
            f"驱动电压: {V_cmd:.1f}V"
        )

        # 更新右侧曲线 (保留最近 4 秒窗口: 4.0s / 1ms = 4000 点)
        t_arr = np.array(sim.history_t)
        if len(t_arr) > 1:
            idx_start = max(0, len(t_arr) - int(4.0 / sim.cfg.T_s))
            t_win = t_arr[idx_start:]

            line_th_curve.set_data(t_win, np.array(sim.history_theta)[idx_start:])
            ax_th.set_xlim(t_win[0], t_win[-1])
            ax_th.set_ylim(-190, 190)

            line_al_curve.set_data(t_win, np.array(sim.history_alpha)[idx_start:])
            line_tgt_curve.set_data(t_win, np.array(sim.history_alpha_tgt)[idx_start:])
            ax_al.set_xlim(t_win[0], t_win[-1])
            ax_al.set_ylim(-70, 70)

            line_v_curve.set_data(t_win, np.array(sim.history_voltage)[idx_start:])
            ax_v.set_xlim(t_win[0], t_win[-1])
            ax_v.set_ylim(-13, 13)

        return line_arm, line_pend, pt_tip, line_th_curve, line_al_curve, line_v_curve

    ani = FuncAnimation(fig, update, interval=25, blit=False)
    plt.tight_layout()
    plt.show()

    # 退出后自动保存综合评估报告
    generate_evaluation_plots(sim, "simulation/simulation_report.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Furuta Pendulum Simulation")
    parser.add_argument("--save-plot-only", action="store_true", help="直接生成并保存全赛题综合评估图表，不弹出交互窗口")
    args = parser.parse_args()

    if args.save_plot_only:
        sim = InteractiveSimulation()
        sim.run_batch(duration=8.0)
        generate_evaluation_plots(sim, "simulation/simulation_report.png")
    else:
        launch_gui()
