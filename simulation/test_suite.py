"""
Automated Test Suite for Furuta Pendulum Physical Simulation
全国大学生嵌入式芯片与系统设计竞赛 - 选题一
验证项目：
  1. 无阻尼自由摆动动力学与能量守恒性校验
  2. 从静止下垂到垂直倒立的平滑起摆与自平衡捕获 (基础要求 1)
  3. 倒立平衡稳态精度与抗外力推力扰动恢复 (基础要求 2, 3)
  4. 转臂目标位置 LQI 阶跃定点零静差跟踪 (拓展要求 1, 3)
  5. 连续正弦轨迹与动态速度跟踪 (拓展要求 2)
  6. FPGA Q12.16 逐比特定点数硬件在环等效性测试
"""

import sys
import io
import os
import numpy as np

# 自动处理路径与 Windows 编码保护
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if base_dir not in sys.path:
    sys.path.insert(0, base_dir)

if sys.platform == "win32":
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    except Exception:
        pass

try:
    from simulation.config import PendulumConfig
    from simulation.dynamics import FurutaDynamics
    from simulation.controller import DiscreteFPGAController
except ImportError:
    from config import PendulumConfig
    from dynamics import FurutaDynamics
    from controller import DiscreteFPGAController


def test_free_energy_conservation():
    """测试 1: 物理动力学守恒性验证 (关闭阻尼与电机力矩，机械总能保持守恒)"""
    print("\n[TEST 1] 运行物理动力学无阻尼自由摆动测试...")
    cfg = PendulumConfig()
    cfg.b1 = 0.0
    cfg.b2 = 0.0
    cfg.b_emf = 0.0
    cfg.coulomb_tau = 0.0

    dyn = FurutaDynamics(cfg)
    # 释放角度: 偏离倒立 30 度
    dyn.reset(alpha=0.0, dalpha=0.0, theta=np.radians(30.0), dtheta=0.0)

    E_initial = dyn.compute_total_energy()
    energies = []

    for _ in range(2000):  # 1.0s, dt = 0.5ms
        dyn.step_rk4(V_motor=0.0, dt=0.0005)
        energies.append(dyn.compute_total_energy())

    max_energy_drift = np.max(np.abs(np.array(energies) - E_initial))
    print(f"  -> 初始机械能: {E_initial:.6f} J")
    print(f"  -> 1秒内全系统机械能最大漂移: {max_energy_drift:.6e} J")
    assert max_energy_drift < 1e-4, f"RK4 积分能量漂移过大: {max_energy_drift}"
    print("  [PASS] 物理动力学与 RK4 数值积分器高精度验证通过！")


def test_swingup_and_balance():
    """测试 2: 闭环起摆与自平衡捕获 (下垂 180 度 -> 倒立 0 度)"""
    print("\n[TEST 2] 运行下垂起摆与倒立自平衡测试...")
    cfg = PendulumConfig()
    dyn = FurutaDynamics(cfg)
    ctrl = DiscreteFPGAController(cfg)

    # 初始状态: 静止下垂
    dyn.reset(alpha=0.0, dalpha=0.0, theta=np.pi, dtheta=0.0)
    ctrl.reset(initial_theta=np.pi, initial_alpha=0.0)

    catch_time = None
    t_total = 5.0
    steps = int(t_total / cfg.T_s)

    for step in range(steps):
        t = step * cfg.T_s
        V_cmd, mode, _ = ctrl.compute_control_voltage(dyn.state, cfg.T_s)

        if mode == "SWITCH_TO_BAL" and catch_time is None:
            catch_time = t
            print(f"  -> 在 t = {catch_time:.3f}s 成功由平滑起摆切入倒立自平衡模式！")

        # 物理动力学子步积分 (1ms 内细分 5 个 0.2ms 物理步)
        n_sub = int(np.round(cfg.T_s / cfg.dt_phys))
        for _ in range(n_sub):
            dyn.step_rk4(V_motor=V_cmd, dt=cfg.dt_phys)

    final_th_deg = np.degrees(dyn.normalize_theta(dyn.state[2]))
    final_alpha_deg = np.degrees(dyn.state[0])

    print(f"  -> 5秒末摆杆残余倾角: {final_th_deg:.3f} deg")
    print(f"  -> 5秒末转臂残余位置: {final_alpha_deg:.2f} deg")

    assert catch_time is not None, "未能在规定时间内完成起摆！"
    assert catch_time < 3.0, f"起摆耗时超出 3 秒限制: {catch_time:.2f}s"
    assert abs(final_th_deg) < 0.5, f"稳态倒立倾角偏大: {final_th_deg:.2f} deg"
    print("  [PASS] 平滑自适应起摆控制与自平衡控制验证通过 (满足基础要求 1, 2)！")


def test_disturbance_rejection():
    """测试 3: 倒立自平衡抗外力推力扰动测试"""
    print("\n[TEST 3] 运行抗外力推力扰动测试 (模拟轻推摆杆)...")
    cfg = PendulumConfig()
    dyn = FurutaDynamics(cfg)
    ctrl = DiscreteFPGAController(cfg)

    # 初始直接置于倒立平衡状态
    dyn.reset(alpha=0.0, dalpha=0.0, theta=0.0, dtheta=0.0)
    ctrl.reset(initial_theta=0.0, initial_alpha=0.0)
    ctrl.mode = ctrl.STATE_BALANCE

    max_deflection = 0.0
    recovered_time = None

    t_total = 3.0
    steps = int(t_total / cfg.T_s)

    for step in range(steps):
        t = step * cfg.T_s

        # 在 t = 0.8s ~ 0.85s 施加持续 50ms 的推力脉冲力矩 (0.04 N*m)
        if 0.80 <= t <= 0.85:
            dyn.set_disturbance(tau_pendulum=0.04)
        else:
            dyn.set_disturbance(tau_pendulum=0.0)

        V_cmd, _, _ = ctrl.compute_control_voltage(dyn.state, cfg.T_s)

        n_sub = int(np.round(cfg.T_s / cfg.dt_phys))
        for _ in range(n_sub):
            dyn.step_rk4(V_motor=V_cmd, dt=cfg.dt_phys)

        th_deg = abs(np.degrees(dyn.normalize_theta(dyn.state[2])))
        if t >= 0.80:
            if th_deg > max_deflection:
                max_deflection = th_deg
            # 扰动结束后，误差回到 0.5 度以内的时间
            if t > 0.85 and th_deg < 0.5 and recovered_time is None:
                recovered_time = t - 0.85

    print(f"  -> 受推力扰动最大瞬时偏角: {max_deflection:.2f} deg")
    print(f"  -> 恢复稳定耗时: {recovered_time:.3f} s")

    assert max_deflection < 15.0, "扰动偏角超出安全范围！"
    assert recovered_time is not None and recovered_time < 0.8, f"恢复稳定耗时过长: {recovered_time}"
    print("  [PASS] 抗外力扰动能力验证通过 (满足基础要求 3)！")


def test_position_tracking():
    """测试 4: 转臂定点位置伺服控制与直立保持 (LQI 积分消除静差)"""
    print("\n[TEST 4] 运行转臂定点位置伺服控制测试 (0 deg -> 45 deg, LQI 消除静差)...")
    cfg = PendulumConfig()
    dyn = FurutaDynamics(cfg)
    ctrl = DiscreteFPGAController(cfg)

    dyn.reset(alpha=0.0, dalpha=0.0, theta=0.0, dtheta=0.0)
    ctrl.reset(initial_theta=0.0, initial_alpha=0.0)
    ctrl.mode = ctrl.STATE_BALANCE

    t_total = 4.0
    steps = int(t_total / cfg.T_s)

    for step in range(steps):
        t = step * cfg.T_s

        # 在 t = 1.0s 下发阶跃目标位置: 45度
        if t >= 1.0:
            ctrl.set_target_position(45.0)

        V_cmd, _, _ = ctrl.compute_control_voltage(dyn.state, cfg.T_s)

        n_sub = int(np.round(cfg.T_s / cfg.dt_phys))
        for _ in range(n_sub):
            dyn.step_rk4(V_motor=V_cmd, dt=cfg.dt_phys)

    final_alpha_deg = np.degrees(dyn.state[0])
    final_th_deg = np.degrees(dyn.normalize_theta(dyn.state[2]))
    pos_err = abs(final_alpha_deg - 45.0)

    print(f"  -> 目标位置: 45.00 deg, 最终实际位置: {final_alpha_deg:.3f} deg, 静差: {pos_err:.4f} deg")
    print(f"  -> 定位稳态摆杆倒立偏差: {final_th_deg:.3f} deg")

    assert pos_err < 0.20, f"转臂定点稳态误差偏大: {pos_err:.3f} deg"
    assert abs(final_th_deg) < 0.5, f"定位过程摆杆未保持直立: {final_th_deg:.2f} deg"
    print("  [PASS] LQI 积分抗静差转臂定点位移控制验证通过 (满足拓展要求 1, 3)！")


def test_continuous_trajectory_tracking():
    """测试 5: 连续正弦轨迹与动态速度跟踪 (拓展要求 2)"""
    print("\n[TEST 5] 运行连续正弦轨迹与速度跟踪测试 (拓展要求 2: 动态运动中保持直立)...")
    cfg = PendulumConfig()
    dyn = FurutaDynamics(cfg)
    ctrl = DiscreteFPGAController(cfg)

    dyn.reset(alpha=0.0, dalpha=0.0, theta=0.0, dtheta=0.0)
    ctrl.reset(initial_theta=0.0, initial_alpha=0.0)
    ctrl.mode = ctrl.STATE_BALANCE
    ctrl.set_trajectory_mode("sine")

    t_total = 4.0
    steps = int(t_total / cfg.T_s)
    tracking_errors = []
    max_pendulum_angle = 0.0

    for step in range(steps):
        V_cmd, _, _ = ctrl.compute_control_voltage(dyn.state, cfg.T_s)

        n_sub = int(np.round(cfg.T_s / cfg.dt_phys))
        for _ in range(n_sub):
            dyn.step_rk4(V_motor=V_cmd, dt=cfg.dt_phys)

        ref_deg = np.degrees(ctrl.alpha_target)
        act_deg = np.degrees(dyn.state[0])
        th_deg = abs(np.degrees(dyn.normalize_theta(dyn.state[2])))

        if step > int(0.5 / cfg.T_s):  # 忽略启动前 0.5s 初始过渡
            tracking_errors.append(abs(act_deg - ref_deg))
            if th_deg > max_pendulum_angle:
                max_pendulum_angle = th_deg

    rms_tracking_err = np.sqrt(np.mean(np.array(tracking_errors) ** 2))
    print(f"  -> 动态正弦轨迹跟踪 RMS 误差: {rms_tracking_err:.2f} deg")
    print(f"  -> 连续动态跟踪全过程摆杆最大倾角: {max_pendulum_angle:.2f} deg")

    assert max_pendulum_angle < 3.0, f"动态轨迹运动中摆杆偏角过大: {max_pendulum_angle:.2f} deg"
    assert rms_tracking_err < 8.0, f"轨迹跟踪误差偏大: {rms_tracking_err:.2f} deg"
    print("  [PASS] 连续轨迹与速度动态伺服跟踪验证通过 (满足拓展要求 2)！")


def test_fixed_point_hardware_equivalence():
    """测试 6: FPGA Q12.16 逐比特定点数硬件仿真与浮点等效性测试"""
    print("\n[TEST 6] 运行 FPGA Q12.16 逐比特定点硬件算法与浮点仿真闭环等效性测试...")
    cfg = PendulumConfig()
    cfg.sensor_noise_std = 0.0  # 关闭随机测量噪声以精确比对纯算术精度

    dyn_fl = FurutaDynamics(cfg)
    ctrl_fl = DiscreteFPGAController(cfg)
    ctrl_fl.use_fixed_point = False

    dyn_fx = FurutaDynamics(cfg)
    ctrl_fx = DiscreteFPGAController(cfg)
    ctrl_fx.use_fixed_point = True

    dyn_fl.reset(alpha=0.0, dalpha=0.0, theta=0.0, dtheta=0.0)
    ctrl_fl.reset(initial_theta=0.0, initial_alpha=0.0)
    ctrl_fl.mode = ctrl_fl.STATE_BALANCE

    dyn_fx.reset(alpha=0.0, dalpha=0.0, theta=0.0, dtheta=0.0)
    ctrl_fx.reset(initial_theta=0.0, initial_alpha=0.0)
    ctrl_fx.mode = ctrl_fx.STATE_BALANCE

    max_diff_alpha = 0.0
    max_diff_theta = 0.0
    steps = int(2.0 / cfg.T_s)

    for step in range(steps):
        t = step * cfg.T_s
        if t >= 0.4:
            ctrl_fl.set_target_position(30.0)
            ctrl_fx.set_target_position(30.0)

        V_fl, _, _ = ctrl_fl.compute_control_voltage(dyn_fl.state, cfg.T_s)
        V_fx, _, _ = ctrl_fx.compute_control_voltage(dyn_fx.state, cfg.T_s)

        n_sub = int(np.round(cfg.T_s / cfg.dt_phys))
        for _ in range(n_sub):
            dyn_fl.step_rk4(V_motor=V_fl, dt=cfg.dt_phys)
            dyn_fx.step_rk4(V_motor=V_fx, dt=cfg.dt_phys)

        diff_al = abs(np.degrees(dyn_fl.state[0] - dyn_fx.state[0]))
        diff_th = abs(np.degrees(dyn_fl.state[2] - dyn_fx.state[2]))

        if diff_al > max_diff_alpha:
            max_diff_alpha = diff_al
        if diff_th > max_diff_theta:
            max_diff_theta = diff_th

    print(f"  -> 2000步闭环控制后 转臂位置最大定点漂移: {max_diff_alpha:.4f} deg")
    print(f"  -> 2000步闭环控制后 摆杆倾角最大定点漂移: {max_diff_theta:.4f} deg")

    assert max_diff_alpha < 0.25, f"定点化转臂位置漂移偏大: {max_diff_alpha:.4f} deg"
    assert max_diff_theta < 0.15, f"定点化摆杆倾角漂移偏大: {max_diff_theta:.4f} deg"
    print("  [PASS] 高云 FPGA Q12.16 逐比特定点数硬件仿真与浮点高精度等效验证通过！")


if __name__ == "__main__":
    print("=" * 70)
    print("  全国大学生嵌入式芯片与系统设计竞赛 - 选题一物理仿真自动测试 (强化版)")
    print("=" * 70)
    try:
        test_free_energy_conservation()
        test_swingup_and_balance()
        test_disturbance_rejection()
        test_position_tracking()
        test_continuous_trajectory_tracking()
        test_fixed_point_hardware_equivalence()
        print("\n" + "=" * 70)
        print("🎉 全部 6 项核心测试全部通过！系统算法与硬件仿真完全达标！")
        print("=" * 70)
    except AssertionError as e:
        print(f"\n❌ 测试未通过: {e}")
        sys.exit(1)
