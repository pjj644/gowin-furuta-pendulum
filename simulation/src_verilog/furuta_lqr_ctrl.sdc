// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 时序约束文件 (SDC: Synopsys Design Constraints)
// 目标器件: 高云半导体 GW2A-LV55PG484C8/I7
// 顶层模块: j280_hw_top
// =============================================================================

// 1. 系统主时钟约束: 板载 50MHz 有源晶振 (周期 20.000ns, 占空比 50%)
create_clock -name clk_50m -period 20.000 -waveform {0.000 10.000} [get_ports {clk_50m}]

// 2. 异步复位与慢速用户开关/按键假路径 (False Path)
set_false_path -from [get_ports {rst_n}]
set_false_path -from [get_ports {sw_motor_en sw_brake_mode key_zero_calib key_pos_clear}]

// 3. 异步传感器输入端口假路径 (F12 修复: 已在内部经过双级同步器与消抖采样)
set_false_path -from [get_ports {enc_a enc_b enc_z adc_miso}]

// 4. 板载状态指示 LED 假路径 (人眼观察慢速显示)
set_false_path -to [get_ports {led_balance led_calib_ok led_motor_run}]
