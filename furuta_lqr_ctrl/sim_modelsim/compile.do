onerror {quit -f}

# =============================================================================
# ModelSim 编译脚本：仅建立 work 库并编译全部 RTL 与 Testbench
# 由 run_all.bat 调用；仿真执行请见 run_all.bat（四个 TB 需各自独立进程运行，
# 因为每个 TB 以 $finish 结尾，会终止同一 do 脚本内的后续命令）
# =============================================================================

vlib work
vmap work work

# ---- RTL 设计文件 (8 个) ----
vlog -work work ../src/furuta_lqr_ctrl.v
vlog -work work ../src/motor_pwm_driver.v
vlog -work work ../src/encoder_quad_reader.v
vlog -work work ../src/angle_sensor_reader.v
vlog -work work ../src/swing_up_ctrl.v
vlog -work work ../src/traj_gen.v
vlog -work work ../src/ctrl_fsm.v
vlog -work work ../src/j280_hw_top.v

# ---- Testbench 文件 (3 个: 开环单元/集成激励) ----
vlog -work work ../src/swing_up_ctrl_tb.v
vlog -work work ../src/j280_hw_top_tb.v
vlog -work work ../src/furuta_lqr_ctrl_tb.v

# ---- F13-b 闭环硬件在环 (HIL) 仿真组件 (2 个, 仅仿真, 不参与综合) ----
# furuta_plant_model.v 必须在 furuta_hil_tb.v 之前编译 (后者例化前者)
vlog -work work ../src/furuta_plant_model.v
vlog -work work ../src/furuta_hil_tb.v

# 编译完成后显式退出: 否则 vsim -c 会停在命令解释器等待 stdin,
# 在非交互 (批处理/重定向) 场景下永久挂起, 导致 run_all.bat 卡在 Step 1。
quit -f
