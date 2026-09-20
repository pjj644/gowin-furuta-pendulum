// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: furuta_hil_tb
// 功能描述: 旋转倒立摆 **闭环硬件在环 (HIL) 仿真测试平台**  —— 关闭 F13-b 缺口
//
//   本 TB 例化两个对象构成真实闭环:
//     DUT   : j280_hw_top   (完整 RTL 顶层, TIMER_1MS_LIMIT = 50000, 即真实 1ms 控制周期)
//     PLANT : furuta_plant_model (dynamics.py 逐式移植的非线性被控对象 + RK4)
//
//   闭环路径 (全部经由真实外设引脚, 绝无直写 DUT 内部信号):
//     PLANT.raw_adc --(SPI 从机行为模型)--> adc_miso 引脚 --> u_dut.angle_sensor_reader
//     PLANT.enc_a / enc_b ---------------> enc_a/enc_b 引脚 --> u_dut.encoder_quad_reader
//     u_dut.motor_in1 / motor_in2 / motor_stby 引脚 --> PLANT (1ms 窗口占空比 -> V_motor)
//
//   因此 DUT 的 12-bit 量化、零点扣减、360° 解卷绕、1ms 差分测速、一阶 IIR 滤波、
//   正交 4 倍频鉴相、8 拍数字消抖、LQI 定点流水线、四态状态机、20kHz PWM 发生器
//   全部真实参与闭环 —— 这与既有 3 个开环 TB 有本质区别。
//
// -----------------------------------------------------------------------------
// 上电时序 (严格遵守 ctrl_fsm.v 的 calib_done 前置条件):
//   1) 复位释放, sw_motor_en = 0
//   2) PLANT 冻结在 theta = 0 (倒立, ADC = 2048), 按下 key_zero_calib
//      -> 锁存零点 zero_offset_reg = 2048, led_calib_ok 常亮, zero_calibrated = 1
//   3) PLANT 冻结搬迁到 theta = pi (下垂) 并保持 60ms, 让 angle_sensor_reader 的
//      IIR 角速度从码值阶跃中收敛到 ~0, 随后释放冻结 (下垂是稳定平衡点, 保持静止)
//   4) sw_motor_en = 1 -> FSM 自动 HANGING -> SWINGUP 并注入 250 (=3.0V) 起动脉冲
//
// -----------------------------------------------------------------------------
// 运行时长控制 (仿真耗时提示):
//   1 秒物理时间 = 50,000,000 个 50MHz 时钟周期 = 5000 次 RK4 (每次 4 回导数求值)。
//   默认 HIL_CTRL_CYCLES = 8000 (8.0 秒物理时间), 足够完整覆盖 A/B/C/D 四个场景
//   (setup 0.07s + 起摆窗口 4.0s + 平衡 1.0s + 抗扰 1.1s + 定点 1.5s = 7.67s)。
//   ModelSim SE-64 10.7 / i7 级 CPU 实测吞吐量约 22 ms wall-clock / 1 ms 物理时间,
//   即默认档约需 3 分钟。
//
//   完整验收 (赛题"长时间保持稳定不倒下", 目标 >= 60s) 请用 plusarg 拉长:
//       cd sim_modelsim
//       vsim -c -voptargs=+acc work.furuta_hil_tb +HIL_CTRL_CYCLES=65000 ^
//            -do "run -all; quit -f"
//   注意 65000 个控制周期 = 65 秒物理时间 = 3.25e9 个时钟周期, 实测约需 25 分钟,
//   建议先跑默认档确认闭环方向正确, 再决定是否拉长。
//
//   可用 plusarg:
//     +HIL_CTRL_CYCLES=<n>  总预算 (单位: 1ms 控制周期数)      默认 8000
//     +HIL_TRACE_DIV=<n>    每 n 个控制周期打印一行轨迹          默认 100 (0 = 关闭)
//     +PLANT_SELFTEST=<0|1> 被控对象传感器映射自检打印           默认 1
//
// -----------------------------------------------------------------------------
// 参数覆盖说明 (重要):
//   TIMER_1MS_LIMIT 保持硬件默认 50000 (真实 1ms), **不得缩小** —— 否则
//   "每控制周期 5 次 RK4 / dt_phys = 0.2ms" 的对应关系与 LQI 积分步长全部失真,
//   闭环结论无效。仅 CALIB_DEBOUNCE_CYCLES 由 1,000,000 缩小到 100 以加速按键消抖;
//   该参数同时决定 KEY2 短按窗口 [100, 3500) clk = [2us, 70us), 本 TB 的按键
//   脉宽取 20us (1000 clk), 落在短按窗口内, 不会误触发长按 clear_pos_pulse
//   (clear_pos 会清零 pulse_count, 使编码器与 PLANT 的 alpha 失同步)。
// =============================================================================

`timescale 1ns / 1ps

module furuta_hil_tb;

    // -------------------------------------------------------------------------
    // 场景预算与判据常量
    // -------------------------------------------------------------------------
    localparam integer SWING_MAX     = 5000;  // A: 起摆最长等待 5.0 s
                                              //    依据: 官方 FAQ 90g 摆杆蓄能耗时相应增加 (test_suite.py TEST2 断言
                                              //    catch_time < 5.0 s, 实测 ~3.9 s); 本窗口取 5.0 s。
    localparam integer BAL_HOLD_PLAN = 1000;  // B: 平衡保持计划时长 1.0 s
    localparam integer DIST_PLAN     = 1100;  // C: 抗扰窗口 50ms 推力 + 1050ms 恢复
    // D: 定点伺服观察预算。必须 >= 斜坡时长 + TAIL_SETTLE_MS + TAIL_AVG_MS。
    //    斜坡 45deg / (RAMP_STEP_Q16=57 -> 0.0498 deg/ms) = 904 ms，
    //    对标 90g 摆杆双倍转动惯量与摩擦阻尼沉降周期:
    //    故 904 + 3600 + 200 = 4704，取 5000 留充分余量 (消除极限环振荡残余)。
    localparam integer TRAJ_PLAN     = 5000;
    localparam integer RECOVER_PLAN  = 2000;  // 跌落后额外观察重新起摆 2.0 s
    localparam integer DIST_PUSH_MS  = 50;    // 推力脉冲持续时间 (与 test_suite.py 一致)
    localparam integer DIST_TAU_uNm  = 40000; // 推力力矩 0.040 N*m (与 test_suite.py 一致)
    localparam integer TAIL_AVG_MS   = 200;   // 稳态误差取最后 200ms 均值 (见下说明)
    localparam integer TAIL_SETTLE_MS= 3600;  // 斜坡完成后到开尾窗之间的沉降时间 (3600ms，对标 90g 摆杆惯量沉降)

    // E: 正弦动态轨迹跟踪观察预算 (拓展要求 2)
    //    0.2Hz 正弦 1 个完整周期 5000 ms，幅值 25 deg
    localparam integer SINE_PLAN     = 5000;
    localparam real    SINE_TH_LIMIT = 8.0;   // 判据 E: 动态跟踪期间摆杆直立倾角上限 (deg)
    localparam real    SINE_RMS_LIMIT= 10.0;  // 判据 E: 转臂正弦跟踪 RMS 误差上限 (deg)

    localparam integer CAPTURE_TH_Q16= 17188; // 诊断回退投放角 15 deg (= 0.2618 rad) 的 Q16

    localparam real RAD2DEG          = 57.29577951308232087680;
    localparam real BAL_ANG_LIMIT    = 22.0;   // 判据 B: 平衡区角度上限 (deg)
    localparam real DIST_DEF_LIMIT   = 15.0;   // 判据 C: 扰动最大偏角上限 (deg)   —— test_suite.py TEST3
    localparam real DIST_REC_TH      = 0.5;    // 判据 C: 恢复判定的角度阈值 (deg) —— test_suite.py TEST3
    localparam integer DIST_REC_LIMIT= 800;    // 判据 C: 恢复时间上限 (ms)          —— test_suite.py TEST3
    // 判据 D: 转臂定点稳态位置误差上限 (deg) —— test_suite.py TEST4 的 pos_err < 0.20。
    //   由于 4000 CPR 编码器 (0.088 deg/脉冲) 与 12-bit ADC (0.088 deg/码) 的量化,
    //   稳态存在 ±1 个量化级的极限环, 故本判据作用于**最后 TAIL_AVG_MS 的均值**
    //   (阈值数值与 Python 侧完全一致, 仅将“末点瞬时值”换为统计上更合理的稳态均值),
    //   同时如实打印末点瞬时值与过程峰值, 不隐藏任何信息。
    localparam real TRAJ_POS_LIMIT   = 0.20;
    localparam real TRAJ_TH_SS_LIMIT = 0.5;    // 判据 D: 稳态 |theta| 均值上限 (deg) —— test_suite.py TEST4
    localparam real TRAJ_TH_LIMIT    = 22.0;   // 判据 D: 位置控制全程直立保持上限 (deg)

    localparam integer PI_Q16        = 205887;  // pi * 65536 (round)
    localparam integer CALIB_DEB     = 100;     // 覆盖后的按键消抖阈值 (clk)
    localparam integer KEY_PRESS_NS  = 20000;   // 短按脉宽 20us = 1000 clk (窗口 [100,3500) clk)
    localparam integer KEY_LONG_NS   = 100000;  // 长按脉宽 100us = 5000 clk (>= 3500 clk -> 位置清零)

    // 诊断阈值: 用于验证 angle_sensor_reader.v 的最短角位移解卷绕是否生效。
    //   该模块的差分测速曾在 theta 跨越 ±180° 时产生 diff_theta_step = -2*pi
    //   (Q16: -411775)，raw_dtheta = -411775 * 1000 = -6283 rad/s 的假尖峰；
    //   解卷绕修复后此类尖峰应为 0 次，并由判据 A2 强制检查 (见 PHASE A)。
    //   真实 |dtheta| 物理上限约 40 rad/s, 故 100 rad/s 门限只会由该类假尖峰触发。
    //   计数器同时用于给"起摆失败"提供 DUT 内部的直接证据 (纯观测, 不参与闭环)。
    localparam real DTH_SPIKE_RAD_S  = 100.0;

    // -------------------------------------------------------------------------
    // 板级信号
    // -------------------------------------------------------------------------
    reg  clk_50m;
    reg  rst_n;
    reg  sw_motor_en;
    reg  sw_brake_mode;
    reg  key_zero_calib;
    reg  key_pos_clear;
    reg  enc_z;                 // 恒 0: 见 furuta_plant_model.v 头部 S4 说明

    wire motor_pwm;
    wire motor_dir;
    wire motor_in1;
    wire motor_in2;
    wire motor_stby;

    wire enc_a;                 // 由 PLANT 驱动
    wire enc_b;                 // 由 PLANT 驱动

    wire adc_cs_n;
    wire adc_sclk;
    reg  adc_miso;              // 由 SPI 从机行为模型驱动

    wire led_balance;
    wire led_calib_ok;
    wire led_motor_run;

    // -------------------------------------------------------------------------
    // DUT: j280_hw_top (真实 1ms 控制周期, 仅缩小按键消抖)
    // -------------------------------------------------------------------------
    j280_hw_top #(
        .CLK_FREQ_HZ          (50_000_000),
        .TIMER_1MS_LIMIT      (50_000),      // 真实 1ms —— 闭环有效性前提, 不得缩小
        .CALIB_DEBOUNCE_CYCLES(CALIB_DEB)    // 仿真加速: 2us 按键消抖
        // ARM_SOFT_LIMIT_Q16 采用硬件默认 32'sd823548 (±2 圈), 验证真实阈值
    ) u_dut (
        .clk_50m       (clk_50m),
        .rst_n         (rst_n),
        .sw_motor_en   (sw_motor_en),
        .sw_brake_mode (sw_brake_mode),
        .key_zero_calib(key_zero_calib),
        .key_pos_clear (key_pos_clear),
        .motor_pwm     (motor_pwm),
        .motor_dir     (motor_dir),
        .motor_in1     (motor_in1),
        .motor_in2     (motor_in2),
        .motor_stby    (motor_stby),
        .enc_a         (enc_a),
        .enc_b         (enc_b),
        .enc_z         (enc_z),
        .adc_cs_n      (adc_cs_n),
        .adc_sclk      (adc_sclk),
        .adc_miso      (adc_miso),
        .led_balance   (led_balance),
        .led_calib_ok  (led_calib_ok),
        .led_motor_run (led_motor_run)
    );

    // -------------------------------------------------------------------------
    // PLANT: furuta_plant_model (被控对象)
    // -------------------------------------------------------------------------
    reg                state_load;
    reg  signed [31:0] init_alpha_q16;
    reg  signed [31:0] init_dalpha_q16;
    reg  signed [31:0] init_theta_q16;
    reg  signed [31:0] init_dtheta_q16;
    reg  signed [31:0] disturb_pend_uNm;
    reg  signed [31:0] disturb_arm_uNm;
    reg  signed [31:0] sensor_noise_urad;

    wire [11:0]        plant_raw_adc;
    wire signed [31:0] plant_theta_q16;
    wire signed [31:0] plant_dtheta_q16;
    wire signed [31:0] plant_alpha_q16;
    wire signed [31:0] plant_dalpha_q16;
    wire signed [31:0] plant_vmotor_mv;
    wire signed [31:0] plant_energy_mj;
    wire signed [31:0] plant_rk4_steps;
    wire               plant_phys_step;
    wire               plant_ctrl_tick;
    wire               plant_diverged;

    furuta_plant_model #(
        .CLK_FREQ_HZ    (50_000_000),
        .PHYS_RATE_HZ   (5_000),       // dt_phys = 0.2ms
        .CTRL_RATE_HZ   (1_000),       // 1ms 控制周期 -> 每周期 5 次 RK4
        .OUT_REFRESH_DIV(100),         // 传感器输出 2us 刷新
        .ADC_ZERO_CODE  (2048),
        .ADC_BITS       (12),
        .ENC_CPR        (4000),
        .V_SUPPLY       (12.0)
    ) u_plant (
        .clk                  (clk_50m),
        .rst_n                (rst_n),
        .motor_in1            (motor_in1),
        .motor_in2            (motor_in2),
        .motor_stby           (motor_stby),
        .disturb_tau_pend_uNm (disturb_pend_uNm),
        .disturb_tau_arm_uNm  (disturb_arm_uNm),
        .sensor_noise_urad    (sensor_noise_urad),
        .state_load           (state_load),
        .init_alpha_q16       (init_alpha_q16),
        .init_dalpha_q16      (init_dalpha_q16),
        .init_theta_q16       (init_theta_q16),
        .init_dtheta_q16      (init_dtheta_q16),
        .raw_adc              (plant_raw_adc),
        .enc_a                (enc_a),
        .enc_b                (enc_b),
        .theta_q16            (plant_theta_q16),
        .dtheta_q16           (plant_dtheta_q16),
        .alpha_q16            (plant_alpha_q16),
        .dalpha_q16           (plant_dalpha_q16),
        .v_motor_mV           (plant_vmotor_mv),
        .energy_rel_mJ        (plant_energy_mj),
        .phys_step            (plant_phys_step),
        .ctrl_tick            (plant_ctrl_tick),
        .diverged             (plant_diverged),
        .rk4_step_count       (plant_rk4_steps)
    );

    // -------------------------------------------------------------------------
    // SPI 从机行为模型 (直接复用 j280_hw_top_tb.v 中已验证的移位发送逻辑)
    //   16 位帧, 12 位 ADC 数据置于低 12 位, 与 angle_sensor_reader.v 的
    //   adc_latch <= shift_reg[11:0] 提取方式配套; negedge adc_cs_n 处采样保持,
    //   等效真实 ADC 的 sample-and-hold, 一帧内码值恒定。
    // -------------------------------------------------------------------------
    reg [15:0] spi_tx_word;
    reg [15:0] spi_slave_shift;

    always @(negedge adc_cs_n) begin
        spi_tx_word     = {4'b0000, plant_raw_adc};
        adc_miso        <= spi_tx_word[15];
        spi_slave_shift <= {spi_tx_word[14:0], 1'b0};
    end

    always @(negedge adc_sclk) begin
        if (!adc_cs_n) begin
            adc_miso        <= spi_slave_shift[15];
            spi_slave_shift <= {spi_slave_shift[14:0], 1'b0};
        end
    end

    // -------------------------------------------------------------------------
    // 50MHz 系统时钟
    // -------------------------------------------------------------------------
    initial begin
        clk_50m = 1'b0;
        forever #10 clk_50m = ~clk_50m;
    end

    // -------------------------------------------------------------------------
    // 统计与判读变量
    // -------------------------------------------------------------------------
    reg     mon_en;
    integer tick_idx;             // 自 sw_motor_en=1 起累计的控制周期数 (~ ms)
    integer trace_div;
    reg [1:0] mon_prev_state;
    reg [1:0] mon_state_now;      // 监视器时间轴上的 fsm_state 快照 (主线程判读用, 免竞态)

    // dtheta 解卷绕假尖峰诊断
    integer dtheta_spike_n;       // 检测到的假尖峰次数 (上升沿计数)
    real    dtheta_spike_peak;    // 假尖峰 |raw_dtheta| 峰值 (rad/s)
    reg     spike_active;
    real    raw_dth_now, abs_raw_dth;

    integer catch_tick;           // 首次进入 BALANCE 的 tick (-1 = 从未)
    integer catch_count;          // 进入 BALANCE 的总次数 (1->2 跳变)
    integer fall_count;           // 跌落回退次数 (2->1 跳变)
    integer balance_ticks;        // 处于 BALANCE 的总 tick 数
    integer protect_ticks;        // 处于 PROTECT 的总 tick 数
    integer softlimit_ticks;      // soft_limit_err 有效的 tick 数
    integer max_abs_pwm;          // 全程 |final_pwm_duty| 峰值
    integer max_abs_pwm_bal;      // BALANCE 期间 |final_pwm_duty| 峰值
    integer swing_pwm_sum;
    integer swing_pwm_n;
    real    min_abs_th_swing_deg; // SWINGUP 期间 |theta| 最小值
    real    max_abs_th_bal_deg;   // BALANCE 期间 |theta| 峰值
    real    max_abs_th_all_deg;   // 全程 |theta| 峰值
    real    max_abs_alpha_deg;    // 全程 |alpha| 峰值
    integer min_energy_mj;        // 全程机械能最小值 (mJ)
    integer max_energy_mj;        // 全程机械能最大值 (mJ)

    // 瞬时量 (供 $display 使用)
    real    th_deg_now, al_deg_now, dth_now, dal_now, abs_th_now, abs_al_now;
    integer abs_pwm_now;

    // 抗扰窗口
    reg     dist_win_open;
    integer dist_push_end_tick;
    real    dist_max_deg;
    integer dist_recover_ms;

    // 轨迹窗口
    reg     traj_win_open;
    integer traj_start_tick;
    integer traj_tail_start;
    real    traj_max_deg;
    real    traj_max_pos_err_deg;
    real    traj_final_pos_err_deg;
    real    traj_pos_err_now;
    real    traj_ss_pos_err_deg;    // 最后 TAIL_AVG_MS 的 |alpha - alpha_ref| 均值
    real    traj_ss_th_deg;         // 最后 TAIL_AVG_MS 的 |theta| 均值
    real    traj_ss_sum_pos, traj_ss_sum_th;
    integer traj_ss_cnt;

    // 正弦动态跟踪窗口 (TEST E: 拓展要求 2)
    reg     sine_win_open;
    real    sine_pos_err_now;
    real    sine_max_pos_err_deg;
    real    sine_max_deg;
    real    sine_ss_sum_sq;
    integer sine_ss_cnt;
    real    sine_rms_deg;

    // -------------------------------------------------------------------------
    // 传感器一致性监测 (修复 M4)
    //
    // PHASE 0b 的 PASS 文本曾宣称「编码器与 PLANT 同步」，但当时唯一的判据是
    // fsm_state !== 0，同步性只打印不比对。后果: H1 造成的 16 脉冲 (1.42°)
    // 系统性偏置跑了整 8 秒、被打印 2 次、被 0 条判据检查，而它恰恰是
    // 定点伺服稳态误差的主因 —— 当时被误读为「控制器收敛慢」。
    //
    // 两类断言的容差故意不同，原因如下:
    //   alpha / pulse_count: 纯计数链，无滤波。plant 每 2us 刷新且单步逼近，
    //     DUT 侧仅 2 级同步器 + 8 拍消抖 ≈ 200ns 延迟；7.6 rad/s 下该延迟
    //     仅对应 0.011 脉冲。故容差取 ±2 脉冲 (0.18°) —— **强断言**。
    //   theta / theta_err_q16: DUT 侧有一阶 IIR (beta=0.35，时间常数约 2.4 个
    //     采样周期)，起摆期 dtheta 达 20 rad/s 时滤波滞后可达 0.048 rad。
    //     滤波滞后是设计特性而非失步，故容差取 ±20000 Q16 (0.3 rad = 17°)
    //     —— **弱断言**，目的是捕获解卷绕失效这类量级错误 (会产生 411775 Q16
    //     的 2pi 跳变)，而不是校验逐点精度。
    // -------------------------------------------------------------------------
    real    sync_exp_pulse_r;
    integer sync_exp_pulse;
    integer sync_pulse_err;
    integer sync_th_diff;
    // 以下四个计数器**故意不放进 reset_stats**: 传感器失步是系统性问题，
    // 必须跨 PHASE 累积统计；若每个 PHASE 前的 reset_stats 将其清零，
    // 起摆阶段发生的失步证据会在进入后续 PHASE 时被抹除。
    // 用声明时初始化，仅在仿真开始时生效一次。
    integer sync_pulse_peak    = 0;
    integer sync_desync_n      = 0;
    integer sync_th_peak       = 0;
    integer sync_th_desync_n   = 0;
    localparam integer SYNC_PULSE_TOL  = 2;      // ±2 脉冲 = 0.18°
    localparam integer SYNC_TH_Q16_TOL = 20000;  // ±0.3 rad，容忍 IIR 滞后
    integer ramp_guard;   // TEST D 斜坡轮询上限计数器 (修复 B2，与 RAMP_STEP_Q16 解耦)
    integer settle_guard; // TEST D-ret 沉降轮询上限计数器
    // ±180° 穿越计数 (修复 M2)。用于证明解卷绕路径被**真实覆盖**:
    // 若起摆期 theta 从未接近 ±180°, 则"0 次假尖峰"是平凡结果，不构成证据。
    integer theta_wrap_n       = 0;
    integer mon_prev_th_q16    = 0;

    // -------------------------------------------------------------------------
    // 在线监视器: 每个控制周期采样一次并累计统计量 + 打印轨迹
    //   注: 这里的层次引用 (u_dut.*) 仅用于**观测与判读**, 不构成闭环路径;
    //       闭环只经由 adc_miso / enc_a / enc_b / motor_in* 引脚。
    // -------------------------------------------------------------------------
    always @(posedge clk_50m) begin
        // ---- 每个时钟都监测未滤波角速度, 捕获 1 拍宽的解卷绕假尖峰 ----
        if (rst_n === 1'b1 && mon_en === 1'b1) begin
            raw_dth_now = u_dut.u_angle_sensor.raw_dtheta_q16 / 65536.0;
            abs_raw_dth = (raw_dth_now < 0.0) ? -raw_dth_now : raw_dth_now;
            if (abs_raw_dth > DTH_SPIKE_RAD_S) begin
                if (spike_active !== 1'b1) begin
                    dtheta_spike_n = dtheta_spike_n + 1;
                    spike_active   = 1'b1;
                end
                if (abs_raw_dth > dtheta_spike_peak) dtheta_spike_peak = abs_raw_dth;
            end else begin
                spike_active = 1'b0;
            end
        end

        if (rst_n === 1'b1 && plant_ctrl_tick === 1'b1 && mon_en === 1'b1) begin
            tick_idx = tick_idx + 1;

            th_deg_now  = plant_theta_q16  * (RAD2DEG / 65536.0);
            al_deg_now  = plant_alpha_q16  * (RAD2DEG / 65536.0);
            dth_now     = plant_dtheta_q16 / 65536.0;
            dal_now     = plant_dalpha_q16 / 65536.0;
            abs_th_now  = (th_deg_now < 0.0) ? -th_deg_now : th_deg_now;
            abs_al_now  = (al_deg_now < 0.0) ? -al_deg_now : al_deg_now;
            abs_pwm_now = (u_dut.final_pwm_duty < 16'sd0) ? -u_dut.final_pwm_duty
                                                          :  u_dut.final_pwm_duty;

            if (abs_th_now > max_abs_th_all_deg) max_abs_th_all_deg = abs_th_now;
            if (abs_al_now > max_abs_alpha_deg)  max_abs_alpha_deg  = abs_al_now;
            if (abs_pwm_now > max_abs_pwm)       max_abs_pwm        = abs_pwm_now;
            if (plant_energy_mj < min_energy_mj) min_energy_mj      = plant_energy_mj;
            if (plant_energy_mj > max_energy_mj) max_energy_mj      = plant_energy_mj;

            case (u_dut.fsm_state)
                2'd1: begin // SWINGUP
                    if (abs_th_now < min_abs_th_swing_deg) min_abs_th_swing_deg = abs_th_now;
                    swing_pwm_sum = swing_pwm_sum + abs_pwm_now;
                    swing_pwm_n   = swing_pwm_n + 1;
                end
                2'd2: begin // BALANCE
                    balance_ticks = balance_ticks + 1;
                    if (abs_th_now  > max_abs_th_bal_deg) max_abs_th_bal_deg = abs_th_now;
                    if (abs_pwm_now > max_abs_pwm_bal)    max_abs_pwm_bal    = abs_pwm_now;
                end
                2'd3: protect_ticks = protect_ticks + 1;
                default: ;
            endcase

            // 状态跳变计数 + 跳变日志 (诊断价值: 完整记录 fsm_state 序列)
            if (mon_prev_state != u_dut.fsm_state) begin
                $display("  [FSM] t=%6d ms  state %0d -> %0d  (th=%9.3f deg, dth=%8.3f rad/s, al=%9.3f deg, pwm=%5d)",
                         tick_idx, mon_prev_state, u_dut.fsm_state,
                         th_deg_now, dth_now, al_deg_now, u_dut.final_pwm_duty);
            end
            if (mon_prev_state != 2'd2 && u_dut.fsm_state == 2'd2) begin
                catch_count = catch_count + 1;
                if (catch_tick < 0) catch_tick = tick_idx;
            end
            if (mon_prev_state == 2'd2 && u_dut.fsm_state == 2'd1) fall_count = fall_count + 1;
            mon_prev_state = u_dut.fsm_state;
            mon_state_now  = u_dut.fsm_state;

            if (u_dut.soft_limit_err === 1'b1) softlimit_ticks = softlimit_ticks + 1;

            // ---- θ 跨越 ±180° 边界计数 (修复 M2) ----
            // 阈值 150000 Q16 = 2.29 rad = 131°: 仅当从 >131° 跳到 <-131°
            // (或反向) 才计为一次真跨越，避免把普通摆动误计。
            if ((mon_prev_th_q16 > 150000) && (plant_theta_q16 < -150000))
                theta_wrap_n = theta_wrap_n + 1;
            if ((mon_prev_th_q16 < -150000) && (plant_theta_q16 > 150000))
                theta_wrap_n = theta_wrap_n + 1;
            mon_prev_th_q16 = plant_theta_q16;

            // ---- 传感器一致性硬断言 (修复 M4，每个控制周期检查) ----
            // 期望脉冲数 = alpha(rad) * ENC_CPR/(2*pi) = alpha * 636.61977
            sync_exp_pulse_r = plant_alpha_q16 / 65536.0 * 636.61977;
            sync_exp_pulse   = (sync_exp_pulse_r >= 0.0) ?  $rtoi(sync_exp_pulse_r + 0.5)
                                                        : -$rtoi(-sync_exp_pulse_r + 0.5);
            sync_pulse_err   = u_dut.pulse_count - sync_exp_pulse;
            if (sync_pulse_err < 0) sync_pulse_err = -sync_pulse_err;
            if (sync_pulse_err > SYNC_PULSE_TOL) begin
                sync_desync_n = sync_desync_n + 1;
                if (sync_pulse_err > sync_pulse_peak) sync_pulse_peak = sync_pulse_err;
            end

            // 摆角一致性: 用最短角距离比对，避免 ±180° 处 +pi/-pi 两种表示造成假报
            sync_th_diff = plant_theta_q16 - u_dut.theta_err_q16;
            if (sync_th_diff >  205887) sync_th_diff = sync_th_diff - 411775;
            if (sync_th_diff < -205888) sync_th_diff = sync_th_diff + 411775;
            if (sync_th_diff < 0) sync_th_diff = -sync_th_diff;
            if (sync_th_diff > SYNC_TH_Q16_TOL) begin
                sync_th_desync_n = sync_th_desync_n + 1;
                if (sync_th_diff > sync_th_peak) sync_th_peak = sync_th_diff;
            end

            // 抗扰窗口统计
            if (dist_win_open === 1'b1) begin
                if (abs_th_now > dist_max_deg) dist_max_deg = abs_th_now;
                if (dist_recover_ms < 0 && tick_idx > dist_push_end_tick &&
                    abs_th_now < DIST_REC_TH && u_dut.fsm_state == 2'd2)
                    dist_recover_ms = tick_idx - dist_push_end_tick;
            end

            // 轨迹窗口统计
            if (traj_win_open === 1'b1) begin
                traj_pos_err_now = (plant_alpha_q16 - u_dut.alpha_ref_q16) * (RAD2DEG / 65536.0);
                if (traj_pos_err_now < 0.0) traj_pos_err_now = -traj_pos_err_now;
                if (traj_pos_err_now > traj_max_pos_err_deg) traj_max_pos_err_deg = traj_pos_err_now;
                traj_final_pos_err_deg = traj_pos_err_now;
                if (abs_th_now > traj_max_deg) traj_max_deg = abs_th_now;
                if (tick_idx >= traj_tail_start) begin
                    traj_ss_sum_pos = traj_ss_sum_pos + traj_pos_err_now;
                    traj_ss_sum_th  = traj_ss_sum_th  + abs_th_now;
                    traj_ss_cnt     = traj_ss_cnt + 1;
                end
            end

            // 正弦跟踪窗口统计 (TEST E: 拓展要求 2)
            if (sine_win_open === 1'b1) begin
                sine_pos_err_now = (plant_alpha_q16 - u_dut.alpha_ref_q16) * (RAD2DEG / 65536.0);
                if (sine_pos_err_now < 0.0) sine_pos_err_now = -sine_pos_err_now;
                if (sine_pos_err_now > sine_max_pos_err_deg) sine_max_pos_err_deg = sine_pos_err_now;
                if (abs_th_now > sine_max_deg) sine_max_deg = abs_th_now;
                sine_ss_sum_sq = sine_ss_sum_sq + (sine_pos_err_now * sine_pos_err_now);
                sine_ss_cnt    = sine_ss_cnt + 1;
            end

            // 轨迹打印
            if (trace_div > 0 && (tick_idx % trace_div) == 0) begin
                $display("  [TRACE] t=%6d ms st=%0d th=%9.3f deg dth=%8.3f rad/s al=%10.3f deg dal=%8.3f pwm=%5d V=%8.1f mV adc=%4d E=%7d mJ aref=%7d",
                         tick_idx, u_dut.fsm_state, th_deg_now, dth_now, al_deg_now, dal_now,
                         u_dut.final_pwm_duty, plant_vmotor_mv * 1.0, plant_raw_adc,
                         plant_energy_mj, u_dut.alpha_ref_q16);
            end
        end
    end

    // -------------------------------------------------------------------------
    // 时序控制任务
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // 主线程与监视器的时间轴同步 (关键: 消除 NBA 竞态)
    //
    //   plant 的 ctrl_tick 在 clk 上升沿 N 经非阻塞赋值拉高; 而监视器
    //   always @(posedge clk_50m) 在活跃区读到的是**上一拍**的 ctrl_tick,
    //   因此它是在沿 N+1 才累加 tick_idx / 更新 catch_tick / fall_count。
    //
    //   若主线程只写 @(posedge plant_ctrl_tick) 就立刻判读 u_dut.fsm_state,
    //   它会在沿 N 的 NBA 区读到 DUT 刚刚更新的 fsm_state, 比监视器**早一拍**:
    //   循环因 fsm_state==2 而退出, 但 catch_tick 仍是 -1 -> TEST A 假红。
    //   (本次调试实测: RTL 在 tick 3946 真实捕获, 却被判 FAIL, 即此竞态所致。)
    //
    //   解决: 每等一个控制周期, 再多等一个 clk 沿 + 1ns, 让监视器完成本周期记账;
    //   所有窗口判据一律改用监视器时间轴上的 tick_idx / catch_tick / mon_state_now。
    // -------------------------------------------------------------------------
    task sync_ctrl_tick;
        begin
            @(posedge plant_ctrl_tick);   // clk 沿 N   : ctrl_tick 拉高 (plant 推进 5 次 RK4)
            @(posedge clk_50m);           // clk 沿 N+1 : 监视器累加本控制周期
            #1;                           // 让监视器的阻塞赋值全部落定
        end
    endtask

    task wait_ctrl_ticks;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) sync_ctrl_tick;
        end
    endtask

    task reset_stats;
        begin
            tick_idx             = 0;
            mon_prev_state       = 2'd0;
            mon_state_now        = u_dut.fsm_state;
            dtheta_spike_n       = 0;
            dtheta_spike_peak    = 0.0;
            spike_active         = 1'b0;
            catch_tick           = -1;
            catch_count          = 0;
            fall_count           = 0;
            balance_ticks        = 0;
            protect_ticks        = 0;
            softlimit_ticks      = 0;
            max_abs_pwm          = 0;
            max_abs_pwm_bal      = 0;
            swing_pwm_sum        = 0;
            swing_pwm_n          = 0;
            min_abs_th_swing_deg = 1.0e9;
            max_abs_th_bal_deg   = 0.0;
            max_abs_th_all_deg   = 0.0;
            max_abs_alpha_deg    = 0.0;
            min_energy_mj        = 2000000000;
            max_energy_mj        = -2000000000;
            dist_win_open        = 1'b0;
            dist_push_end_tick   = 0;
            dist_max_deg         = 0.0;
            dist_recover_ms      = -1;
            traj_win_open        = 1'b0;
            traj_start_tick      = 0;
            traj_tail_start      = 1000000000;
            traj_max_deg         = 0.0;
            traj_max_pos_err_deg = 0.0;
            traj_final_pos_err_deg = 0.0;
            traj_ss_pos_err_deg  = 0.0;
            traj_ss_th_deg       = 0.0;
            traj_ss_sum_pos      = 0.0;
            traj_ss_sum_th       = 0.0;
            traj_ss_cnt          = 0;
            sine_win_open        = 1'b0;
            sine_max_deg         = 0.0;
            sine_max_pos_err_deg = 0.0;
            sine_ss_sum_sq       = 0.0;
            sine_ss_cnt          = 0;
            sine_rms_deg         = 0.0;
        end
    endtask

    // -------------------------------------------------------------------------
    // 运行预算与结果变量
    // -------------------------------------------------------------------------
    integer hil_ctrl_cycles;
    integer arg_i;
    integer setup_ticks;
    integer ticks_used;
    integer avail, n_ticks, elapsed, bal_held;
    integer phase_start_tick;   // 各阶段窗口起点 (tick_idx 全程连续, 不归零)
    integer trace_div_save;
    integer test_a_pass, test_b_pass, test_c_pass, test_d_pass, test_e_pass, test_cal_pass;
    integer test_b_run, test_c_run, test_d_run, test_e_run, test_rec_run;
    integer test_alt_used;        // 1 = 使用了诊断性回退投放 (意味着 TEST A 已失败)
    integer n_checks, n_fail;
    integer freeze_ticks;         // state_load 冻结期间经历的控制周期数 (不做 RK4)
    integer expected_rk4;
    real    swing_avg_pwm;
    real    wd_ns;

    // -------------------------------------------------------------------------
    // 主测试线程
    // -------------------------------------------------------------------------
    initial begin
        // ---------------- 运行参数 ----------------
        hil_ctrl_cycles = 20000;
        if ($value$plusargs("HIL_CTRL_CYCLES=%d", arg_i)) hil_ctrl_cycles = arg_i;
        trace_div = 100;
        if ($value$plusargs("HIL_TRACE_DIV=%d", arg_i)) trace_div = arg_i;

        test_a_pass = 0; test_b_pass = 0; test_c_pass = 0; test_d_pass = 0; test_e_pass = 0; test_cal_pass = 0;
        test_b_run  = 0; test_c_run  = 0; test_d_run  = 0; test_e_run  = 0; test_rec_run = 0;
        test_alt_used = 0;
        n_checks    = 0; n_fail      = 0;
        setup_ticks = 0;  ticks_used  = 0; bal_held = 0;
        phase_start_tick = 0;
        freeze_ticks = 0; expected_rk4 = 0;

        $display("=================================================================");
        $display("   J280 旋转倒立摆 闭环硬件在环 (HIL) 仿真   F13-b");
        $display("   DUT   : j280_hw_top  (TIMER_1MS_LIMIT = 50000, 真实 1ms 控制周期)");
        $display("   PLANT : furuta_plant_model (RK4, dt_phys = 0.2ms, 5 步/控制周期)");
        $display("   预算  : %0d 个控制周期 = %0d.%03d 秒物理时间",
                 hil_ctrl_cycles, hil_ctrl_cycles/1000, hil_ctrl_cycles%1000);
        $display("   拉长  : vsim ... work.furuta_hil_tb +HIL_CTRL_CYCLES=65000 (65s 完整验收)");
        $display("=================================================================");

        // ---------------- 初始化 ----------------
        rst_n          = 1'b0;
        sw_motor_en    = 1'b0;
        sw_brake_mode  = 1'b1;   // 能耗制动: 停机时 IN1=IN2=1 -> V=0, 与 b_emf 阻尼自洽
        key_zero_calib = 1'b1;
        key_pos_clear  = 1'b1;
        enc_z          = 1'b0;   // 严禁 Z 脉冲 (USE_Z_INDEX=1 会清零 pulse_count)
        adc_miso       = 1'b0;

        state_load        = 1'b1;
        init_alpha_q16    = 32'sd0;
        init_dalpha_q16   = 32'sd0;
        init_theta_q16    = 32'sd0;      // theta = 0 -> 倒立 (供零点标定)
        init_dtheta_q16   = 32'sd0;
        disturb_pend_uNm  = 32'sd0;
        disturb_arm_uNm   = 32'sd0;
        sensor_noise_urad = 32'sd0;      // 默认关闭噪声, 保证回归可复现

        reset_stats;
        mon_en = 1'b0;

        #200;
        rst_n = 1'b1;
        #40;

        // =====================================================================
        // PHASE 0: 一键垂直零点标定 (摆杆倒立 theta = 0, ADC = 2048)
        // =====================================================================
        $display("\n[PHASE 0] 摆杆置于倒立位 (theta=0, raw_adc=%0d), 执行一键零点标定...",
                 plant_raw_adc);
        wait_ctrl_ticks(3);              // 等 3 个控制周期, 确保 adc_has_sampled = 1
        setup_ticks = setup_ticks + 3;
        key_zero_calib = 1'b0;
        #KEY_PRESS_NS;                   // 1000 clk = 20us >> CALIB_DEB(100 clk)
        key_zero_calib = 1'b1;
        wait_ctrl_ticks(2);
        setup_ticks = setup_ticks + 2;

        $display("  zero_offset_reg = %0d (期望 2048), zero_calibrated = %b, led_calib_ok = %b (0=点亮)",
                 u_dut.zero_offset_reg, u_dut.zero_calibrated, led_calib_ok);
        $display("  DUT 解算 theta_err_q16 = %0d (期望 ~0), dtheta_q16 = %0d",
                 u_dut.theta_err_q16, u_dut.dtheta_q16);
        n_checks = n_checks + 1;
        if (led_calib_ok === 1'b0 && u_dut.zero_calibrated === 1'b1 &&
            u_dut.zero_offset_reg === 12'd2048) begin
            test_cal_pass = 1;
            $display("  [PASS] 零点标定成功, ctrl_fsm 的 calib_done 前置条件已满足");
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 零点标定失败 —— FSM 将永久停留在 HANGING, 后续测试无意义");
        end

        // =====================================================================
        // PHASE 0b: 摆杆搬迁到下垂位 (theta = pi) 并静置
        // =====================================================================
        $display("\n[PHASE 0b] 将摆杆搬迁至自由下垂位 (theta = pi, raw_adc = 0) 并静置...");
        init_theta_q16 = PI_Q16;
        state_load     = 1'b1;           // 仍冻结, 让 DUT 的 IIR 角速度收敛
        wait_ctrl_ticks(60);
        setup_ticks = setup_ticks + 60;
        freeze_ticks = setup_ticks;      // 至此为止的全部控制周期均处于冻结态 (不积分)
        state_load = 1'b0;               // 释放: 物理开始积分 (下垂为稳定平衡点)
        mon_en     = 1'b1;               // 使能监视器，采样待机状态与传感器同步性
        wait_ctrl_ticks(10);
        setup_ticks = setup_ticks + 10;
        $display("  静置后: PLANT theta=%0d Q16, DUT theta_err_q16=%0d (期望 ~-205888 = -180deg), dtheta_q16=%0d",
                 plant_theta_q16, u_dut.theta_err_q16, u_dut.dtheta_q16);
        $display("  静置后: PLANT alpha=%0d Q16, DUT pulse_count=%0d, alpha_rad_q16=%0d (三者应一致为 ~0)",
                 plant_alpha_q16, u_dut.pulse_count, u_dut.alpha_rad_q16);
        $display("  静置后: fsm_state=%0d (期望 0=HANGING), RK4 累计步数=%0d (期望 ~%0d)",
                 u_dut.fsm_state, plant_rk4_steps, (setup_ticks - freeze_ticks) * 5);

        // 判据 0a: FSM 待机态
        n_checks = n_checks + 1;
        if (u_dut.fsm_state !== 2'd0) begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 电机未使能时 FSM 不在 HANGING 态!");
        end else begin
            $display("  [PASS] 待机态正确 (HANGING), 摆杆静止下垂");
        end

        // 判据 0b: 传感器与 PLANT 真值同步 (修复 M4)
        // 原代码此处只打印 plant/DUT 对比值却不做任何比对，PASS 文本却宣称
        // 「编码器与 PLANT 同步」—— 属于为未执行的检查背书。H1 的 16 脉冲
        // 偏置正是因此逃过了整个 8 秒运行。
        n_checks = n_checks + 1;
        if (sync_desync_n == 0 && sync_th_desync_n == 0) begin
            $display("  [PASS] 传感器与 PLANT 同步 (pulse 偏差峰值 %0d <= %0d, theta 偏差峰值 %0d <= %0d Q16)",
                     sync_pulse_peak, SYNC_PULSE_TOL, sync_th_peak, SYNC_TH_Q16_TOL);
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 传感器与 PLANT 失步: pulse 失步 %0d 次 (峰值偏差 %0d, 容差 %0d), theta 失步 %0d 次 (峰值 %0d Q16, 容差 %0d)",
                     sync_desync_n, sync_pulse_peak, SYNC_PULSE_TOL,
                     sync_th_desync_n, sync_th_peak, SYNC_TH_Q16_TOL);
        end

        // =====================================================================
        // PHASE A: 起摆测试 (基础要求 1)
        // =====================================================================
        $display("\n[PHASE A / TEST A] 闭合电机使能, 观测能量泵起摆 -> 倒立捕获...");
        reset_stats;
        mon_en      = 1'b1;
        sw_motor_en = 1'b1;

        avail   = hil_ctrl_cycles - ticks_used - setup_ticks;
        n_ticks = (SWING_MAX < avail) ? SWING_MAX : avail;
        if (n_ticks < 0) n_ticks = 0;
        // 窗口一律用监视器时间轴上的 tick_idx 度量, 退出条件用监视器维护的
        // catch_tick, 避开与 DUT fsm_state 的 NBA 竞态 (详见 sync_ctrl_tick 注释)
        while (tick_idx < n_ticks && catch_tick < 0) sync_ctrl_tick;
        elapsed    = tick_idx;
        ticks_used = ticks_used + elapsed;

        swing_avg_pwm = (swing_pwm_n > 0) ? (swing_pwm_sum * 1.0 / swing_pwm_n) : 0.0;
        $display("  ---- TEST A 起摆结果 ----");
        $display("  观测窗口         : %0d ms (计划 %0d ms)", elapsed, n_ticks);
        $display("  是否进入 BALANCE : %s", (catch_tick >= 0) ? "YES" : "NO");
        $display("  起摆耗时         : %0d ms", (catch_tick >= 0) ? catch_tick : -1);
        $display("  SWINGUP 期 |th| 最小值 : %8.3f deg (需显著小于 180 才说明真的抬起来了)",
                 (min_abs_th_swing_deg > 1.0e8) ? -1.0 : min_abs_th_swing_deg);
        $display("  SWINGUP 期 |pwm| 平均值 : %8.2f (占空比计数, 满量程 1000)", swing_avg_pwm);
        $display("  全程 |pwm| 峰值  : %0d", max_abs_pwm);
        $display("  全程 |alpha| 峰值: %8.3f deg (软限位 ±720 deg)", max_abs_alpha_deg);
        $display("  机械能区间       : [%0d, %0d] mJ (下垂静止 ~ -98 mJ, 倒立静止 0 mJ)",
                 min_energy_mj, max_energy_mj);
        $display("  [诊断] raw_dtheta 解卷绕假尖峰: %0d 次, |峰值| = %0.1f rad/s (门限 %.0f rad/s)",
                 dtheta_spike_n, dtheta_spike_peak, DTH_SPIKE_RAD_S);
        $display("         该探测器验证 angle_sensor_reader.v 的最短角位移解卷绕是否生效:");
        $display("         若无解卷绕, theta 每跨越 ±180° 会注入 -2*pi*1000 = -6283 rad/s 假值;");
        $display("         本 PHASE 内 theta 实际跨越 ±180° 共 %0d 次 (跨越数为 0 则探测器未被覆盖)",
                 theta_wrap_n);

        n_checks = n_checks + 1;
        if (catch_tick >= 0) begin
            test_a_pass = 1;
            $display("  [PASS] TEST A: 静止下垂的摆杆被抬起并在 %0d ms 内切入倒立自平衡 (基础要求 1)",
                     catch_tick);
        end else begin
            n_fail = n_fail + 1;
            $display("  [FAIL] TEST A: 在 %0d ms 内未能切入 BALANCE —— 起摆失败 (基础要求 1 未达成)",
                     elapsed);
            $display("         诊断: SWINGUP 期 |th| 最小值 = %8.3f deg, |pwm| 平均 = %8.2f, 峰值 = %0d",
                     (min_abs_th_swing_deg > 1.0e8) ? -1.0 : min_abs_th_swing_deg,
                     swing_avg_pwm, max_abs_pwm);
        end

        // 判据 A2: 最短角位移解卷绕有效性 (修复 M2)
        // 原先 dtheta_spike_n 只打印不判决 —— 把 angle_sensor_reader.v 的解卷绕
        // 逻辑整段删掉也不会有任何一条判据变红，等于该修复没有回归保护。
        // 同时要求跨越数 >= 1，避免"从未接近 ±180° 所以 0 次尖峰"的平凡通过。
        n_checks = n_checks + 1;
        if (dtheta_spike_n != 0) begin
            n_fail = n_fail + 1;
            $display("  [FAIL] TEST A2: 检测到 %0d 次解卷绕假尖峰, |峰值| = %0.1f rad/s —— angle_sensor_reader.v 的最短角位移解卷绕可能已失效",
                     dtheta_spike_n, dtheta_spike_peak);
        end else if (theta_wrap_n == 0) begin
            n_fail = n_fail + 1;
            $display("  [FAIL] TEST A2: 全程未发生 ±180° 跨越, 解卷绕路径未被覆盖 —— 0 次尖峰不构成证据");
        end else begin
            $display("  [PASS] TEST A2: 最短角位移解卷绕有效 —— theta 跨越 ±180° 共 %0d 次, 假尖峰 0 次",
                     theta_wrap_n);
        end

        // =====================================================================
        // PHASE A-alt: 诊断性回退投放 (仅当 TEST A 失败时启用)
        //
        //   目的: TEST A 失败后, 若不采取任何措施, B/C/D 将全部无从执行, 本 TB
        //         就只能回答"起摆不行"而无法回答"平衡/抗扰/定点行不行"。
        //         为了把缺陷隔离到起摆环节, 这里把被控对象人工投放到 FSM 的
        //         **真实捕获窗口内** (theta = 15 deg < 22 deg, theta_dot = 0 < 4 rad/s,
        //         alpha = 0, alpha_dot = 0), 等价于“人手持摆杆到接近倒立位置后松手”,
        //         这也正是 test_suite.py 的 TEST3/TEST4 所用的初始化方式。
        //
        //   声明 (保持诚实):
        //     (1) 本回退 **不改变 TEST A 的 FAIL 结论**, TEST A 仍如实报告失败;
        //         且总结中会显著标注 B/C/D 的结果是在回退投放下取得的。
        //     (2) 这 **不是直写 DUT 内部信号** —— 被控对象状态仍然只经由
        //         SPI (adc_miso) 与正交编码器 (enc_a/enc_b) 引脚回注 DUT,
        //         量化/零点扣减/解卷绕/IIR/鉴相/消抖/LQI/FSM/PWM 全部真实参与。
        //     (3) FSM 必须 **自行** 依据捕获判据切入 BALANCE; 若仍不切入,
        //         则 B/C/D 同样跳过, 不做任何强制。
        //     (4) 投放同时长按 KEY2 (100us >= CALIB_DEB*35) 触发 clear_pos 将
        //         pulse_count 归零, 与投放后的 alpha = 0 保持一致 —— 这是真实
        //         操作员会做的动作, 不是绕过硬件。
        // =====================================================================
        if (catch_tick < 0) begin
            test_alt_used = 1;
            $display("\n[PHASE A-alt] 起摆失败 -> 启用**诊断性回退投放** (theta=15deg, theta_dot=0, alpha=0, alpha_dot=0)");
            $display("            注: 此举仅为隔离缺陷、继续采集 B/C/D 证据, **TEST A 仍为 FAIL**");
            $display("            注: 投放后 tick_idx 归零, 下方 [FSM]/TRACE 的时间轴以投放点为原点");
            state_load      = 1'b1;
            init_theta_q16  = CAPTURE_TH_Q16;
            init_dtheta_q16 = 32'sd0;
            init_alpha_q16  = 32'sd0;
            init_dalpha_q16 = 32'sd0;
            wait_ctrl_ticks(5);                 // 等 A/B 电平与 ADC 码值稳定
            ticks_used   = ticks_used + 5;
            freeze_ticks = freeze_ticks + 5;
            key_pos_clear = 1'b0;
            #KEY_LONG_NS;                       // 长按: 清零编码器 pulse_count
            key_pos_clear = 1'b1;
            wait_ctrl_ticks(60);                // 让 DUT 的 IIR 角速度从码值阶跃中收敛
            ticks_used   = ticks_used + 60;
            freeze_ticks = freeze_ticks + 60;
            $display("  投放后(仍冻结): PLANT theta=%0d Q16, DUT theta_err_q16=%0d, dtheta_q16=%0d",
                     plant_theta_q16, u_dut.theta_err_q16, u_dut.dtheta_q16);
            $display("  投放后(仍冻结): PLANT alpha=%0d Q16, DUT pulse_count=%0d, alpha_rad_q16=%0d, traj_mode=%0d",
                     plant_alpha_q16, u_dut.pulse_count, u_dut.alpha_rad_q16, u_dut.traj_mode);
            state_load = 1'b0;                  // 释放: 真实物理过程开始
            reset_stats;                        // B/C/D 的统计从投放点重新开始
            wait_ctrl_ticks(20);
            ticks_used = ticks_used + 20;
            $display("  释放 20ms 后: fsm_state=%0d (2=BALANCE 说明 FSM 已自行捕获), catch_tick=%0d ms",
                     u_dut.fsm_state, catch_tick);
            if (u_dut.fsm_state !== 2'd2) begin
                n_fail = n_fail + 1;
                n_checks = n_checks + 1;
                $display("  [FAIL] PHASE A-alt: 即使投放到捕获窗口内, FSM 仍未能切入 BALANCE —— 捕获逻辑或 LQI 存在额外缺陷");
            end else begin
                $display("  [INFO] PHASE A-alt: FSM 已自行捕获并切入 BALANCE, 继续执行 B/C/D");
            end
        end

        // =====================================================================
        // PHASE B: 平衡保持测试 (基础要求 3 前半)
        // =====================================================================
        if (catch_tick >= 0) begin
            test_b_run = 1;
            $display("\n[PHASE B / TEST B] 倒立平衡保持观测...");
            avail   = hil_ctrl_cycles - ticks_used - setup_ticks;
            n_ticks = (BAL_HOLD_PLAN < avail) ? BAL_HOLD_PLAN : avail;
            if (n_ticks < 0) n_ticks = 0;
            phase_start_tick = tick_idx;   // B 阶段窗口从当前 tick 起量 (tick_idx 不归零)
            while (tick_idx - phase_start_tick < n_ticks && mon_state_now === 2'd2)
                sync_ctrl_tick;
            elapsed    = tick_idx - phase_start_tick;
            ticks_used = ticks_used + elapsed;
            bal_held   = elapsed;

            $display("  ---- TEST B 平衡保持结果 ----");
            $display("  本次保持时长     : %0d ms (计划 %0d ms)", bal_held, n_ticks);
            $display("  累计 BALANCE 时长: %0d ms", balance_ticks);
            $display("  BALANCE 期 |th| 峰值 : %8.3f deg (判据 < %.1f deg)", max_abs_th_bal_deg, BAL_ANG_LIMIT);
            $display("  BALANCE 期 |pwm| 峰值: %0d", max_abs_pwm_bal);
            $display("  跌落回退次数 (2->1)  : %0d", fall_count);
            $display("  PROTECT 停留 tick 数 : %0d", protect_ticks);
            $display("  soft_limit_err tick 数: %0d (转臂软限位 ±720 deg)", softlimit_ticks);

            n_checks = n_checks + 1;
            if (fall_count == 0 && protect_ticks == 0 && max_abs_th_bal_deg < BAL_ANG_LIMIT &&
                bal_held >= n_ticks && n_ticks > 0) begin
                test_b_pass = 1;
                $display("  [PASS] TEST B: 平衡保持 %0d ms 无跌落, |th| 峰值 %8.3f deg < %.1f deg",
                         bal_held, max_abs_th_bal_deg, BAL_ANG_LIMIT);
            end else begin
                n_fail = n_fail + 1;
                $display("  [FAIL] TEST B: 平衡未能稳定保持 (跌落 %0d 次, PROTECT %0d tick, |th|峰值 %8.3f deg, 保持 %0d/%0d ms)",
                         fall_count, protect_ticks, max_abs_th_bal_deg, bal_held, n_ticks);
            end
        end else begin
            $display("\n[PHASE B / TEST B] 未执行 —— 未进入 BALANCE, 无从考察平衡保持 (前置 TEST A 已失败)");
        end

        // ---- 若曾跌落, 额外观察是否能自行重新起摆 (诊断价值) ----
        if (fall_count > 0) begin
            test_rec_run = 1;
            $display("\n[PHASE B'] 已发生跌落回退, 额外观察是否能自行重新起摆...");
            avail   = hil_ctrl_cycles - ticks_used - setup_ticks;
            n_ticks = (RECOVER_PLAN < avail) ? RECOVER_PLAN : avail;
            if (n_ticks < 0) n_ticks = 0;
            wait_ctrl_ticks(n_ticks);
            ticks_used = ticks_used + n_ticks;
            $display("  再观察 %0d ms 后: fsm_state=%0d, 累计进入 BALANCE 次数=%0d, 累计跌落次数=%0d",
                     n_ticks, u_dut.fsm_state, catch_count, fall_count);
        end

        // =====================================================================
        // PHASE C: 抗扰测试 (基础要求 3 后半)
        // =====================================================================
        if (u_dut.fsm_state === 2'd2) begin
            avail   = hil_ctrl_cycles - ticks_used - setup_ticks;
            n_ticks = (DIST_PLAN < avail) ? DIST_PLAN : avail;
            if (n_ticks >= DIST_PUSH_MS + 50) begin
                test_c_run = 1;
                $display("\n[PHASE C / TEST C] 施加外部推力扰动: disturb_tau_pendulum = +0.040 N*m, 持续 %0d ms",
                         DIST_PUSH_MS);
                trace_div_save = trace_div;
                trace_div      = 10;
                dist_max_deg    = 0.0;
                dist_recover_ms = -1;
                dist_win_open   = 1'b1;
                disturb_pend_uNm = DIST_TAU_uNm;
                wait_ctrl_ticks(DIST_PUSH_MS);
                disturb_pend_uNm = 32'sd0;
                dist_push_end_tick = tick_idx;
                wait_ctrl_ticks(n_ticks - DIST_PUSH_MS);
                dist_win_open = 1'b0;
                trace_div     = trace_div_save;
                ticks_used    = ticks_used + n_ticks;

                $display("  ---- TEST C 抗扰结果 ----");
                $display("  推力期间及恢复期 |th| 峰值 : %8.3f deg (判据 < %.1f deg)", dist_max_deg, DIST_DEF_LIMIT);
                $display("  恢复到 |th| < %.1f deg 耗时 : %0d ms (判据 < %0d ms, -1 = 未恢复)",
                         DIST_REC_TH, dist_recover_ms, DIST_REC_LIMIT);
                $display("  扰动后 fsm_state = %0d, 累计跌落次数 = %0d", u_dut.fsm_state, fall_count);

                n_checks = n_checks + 1;
                if (dist_max_deg < DIST_DEF_LIMIT && dist_recover_ms >= 0 &&
                    dist_recover_ms < DIST_REC_LIMIT && u_dut.fsm_state === 2'd2) begin
                    test_c_pass = 1;
                    $display("  [PASS] TEST C: 抵抗 0.04 N*m / %0d ms 推力, 最大偏角 %8.3f deg, %0d ms 内恢复直立 (基础要求 3)",
                             DIST_PUSH_MS, dist_max_deg, dist_recover_ms);
                end else begin
                    n_fail = n_fail + 1;
                    $display("  [FAIL] TEST C: 抗扰不达标 (峰值 %8.3f deg, 恢复 %0d ms, 末态 %0d)",
                             dist_max_deg, dist_recover_ms, u_dut.fsm_state);
                end
            end else begin
                $display("\n[PHASE C / TEST C] 跳过 —— 剩余预算 %0d ms 不足", avail);
            end
        end else begin
            $display("\n[PHASE C / TEST C] 跳过 —— 当前不在 BALANCE 态 (fsm_state=%0d)", u_dut.fsm_state);
        end

        // =====================================================================
        // PHASE D: 定点位置伺服 + 直立保持 (拓展要求 1 / 3)
        // =====================================================================
        if (u_dut.fsm_state === 2'd2) begin
            avail   = hil_ctrl_cycles - ticks_used - setup_ticks;
            n_ticks = (TRAJ_PLAN < avail) ? TRAJ_PLAN : avail;
            if (n_ticks >= 4700) begin   // 需容纳 904ms 斜坡 + 3600ms 沉降 + 200ms 尾窗 (解决 R1)
                test_d_run = 1;
                $display("\n[PHASE D / TEST D] 短按 KEY2 切换 traj_mode 0 -> 1 (alpha_ref = +45 deg), 观测位置伺服与直立保持...");
                traj_max_deg         = 0.0;
                traj_max_pos_err_deg = 0.0;
                traj_win_open        = 1'b1;
                traj_start_tick      = tick_idx;
                traj_ss_sum_pos      = 0.0;
                traj_ss_sum_th       = 0.0;
                traj_ss_cnt          = 0;

                key_pos_clear = 1'b0;
                #KEY_PRESS_NS;                       // 1000 clk, 落在短按窗口 [100,3500) 内
                key_pos_clear = 1'b1;

                // 轮询等待 alpha_ref 真正到达 +45°，不用固定等待时长 (修复 B2)。
                // 原写法 wait_ctrl_ticks(150) 隐含假设 RAMP_STEP_Q16=572 (90ms 走完)；
                // 该常数降为 57 (0.05deg/ms) 后斜坡需 904ms，150 拍时 alpha_ref 仅 8436，
                // 使尾窗 [T0+1300, T0+1500] 只在斜坡结束后 395ms 打开，正好罩住过冲
                // 衰减段 —— 实测把真实稳态误差 1.8° 高估成 10.155° (5.7 倍)。
                ramp_guard = 0;
                while (u_dut.alpha_ref_q16 != 32'sd51472 && ramp_guard < 3000) begin
                    wait_ctrl_ticks(1);
                    ramp_guard = ramp_guard + 1;
                end
                $display("  短按后 traj_mode = %0d (期望 1), alpha_ref_q16 = %0d (期望 51472 = +45deg), 斜坡耗时 %0d ms",
                         u_dut.traj_mode, u_dut.alpha_ref_q16, ramp_guard);

                // 尾窗锚定在「斜坡实际完成时刻 + TAIL_SETTLE_MS 沉降」之后，
                // 而不是相对 PHASE 起点的固定偏移。这样无论斜坡速率如何调整都成立。
                wait_ctrl_ticks(TAIL_SETTLE_MS);
                traj_tail_start = tick_idx;
                $display("  沉降 %0d ms 后开启 %0d ms 稳态尾窗 (tick %0d)",
                         TAIL_SETTLE_MS, TAIL_AVG_MS, traj_tail_start);
                wait_ctrl_ticks(TAIL_AVG_MS);
                traj_win_open = 1'b0;
                ticks_used    = ticks_used + (tick_idx - traj_start_tick);

                traj_ss_pos_err_deg = (traj_ss_cnt > 0) ? (traj_ss_sum_pos / traj_ss_cnt) : -1.0;
                traj_ss_th_deg      = (traj_ss_cnt > 0) ? (traj_ss_sum_th  / traj_ss_cnt) : -1.0;

                $display("  ---- TEST D 定点伺服结果 ----");
                $display("  traj_mode          : %0d", u_dut.traj_mode);
                $display("  alpha_ref_q16      : %0d (%8.3f deg)", u_dut.alpha_ref_q16,
                         u_dut.alpha_ref_q16 * (RAD2DEG / 65536.0));
                $display("  PLANT alpha        : %0d (%8.3f deg)", plant_alpha_q16, al_deg_now);
                $display("  DUT alpha_rad_q16  : %0d (%8.3f deg)", u_dut.alpha_rad_q16,
                         u_dut.alpha_rad_q16 * (RAD2DEG / 65536.0));
                $display("  稳态位置误差(后%0dms均值): %8.4f deg (判据 < %.2f deg, 采样数 %0d)",
                         TAIL_AVG_MS, traj_ss_pos_err_deg, TRAJ_POS_LIMIT, traj_ss_cnt);
                $display("  末点位置误差(瞬时值)  : %8.4f deg", traj_final_pos_err_deg);
                $display("  过程最大位置误差      : %8.4f deg", traj_max_pos_err_deg);
                $display("  过程 |th| 峰值        : %8.3f deg (判据 < %.1f deg, 拓展要求 3: 位置控制中保持直立)",
                         traj_max_deg, TRAJ_TH_LIMIT);
                $display("  稳态 |th| 均值(后%0dms) : %8.4f deg (判据 < %.2f deg)",
                         TAIL_AVG_MS, traj_ss_th_deg, TRAJ_TH_SS_LIMIT);
                $display("  alpha_int_q16      : %0d (LQI 积分器, 消除静差)", u_dut.alpha_int_q16);

                n_checks = n_checks + 1;
                if (u_dut.traj_mode === 2'd1 && traj_ss_pos_err_deg >= 0.0 &&
                    traj_ss_pos_err_deg < TRAJ_POS_LIMIT && traj_ss_th_deg < TRAJ_TH_SS_LIMIT &&
                    traj_max_deg < TRAJ_TH_LIMIT && u_dut.fsm_state === 2'd2) begin
                    test_d_pass = 1;
                    $display("  [PASS] TEST D: 转臂跟随至 +45 deg (稳态静差 %8.4f deg), 全过程摆杆直立 (|th|峰值 %8.3f deg, 稳态 %8.4f deg) (拓展要求 1/3)",
                             traj_ss_pos_err_deg, traj_max_deg, traj_ss_th_deg);
                end else begin
                    n_fail = n_fail + 1;
                    $display("  [FAIL] TEST D: 定点伺服或直立保持不达标 (mode=%0d, 稳态静差 %8.4f deg, 稳态|th| %8.4f deg, |th|峰值 %8.3f deg, 末态 %0d)",
                             u_dut.traj_mode, traj_ss_pos_err_deg, traj_ss_th_deg, traj_max_deg, u_dut.fsm_state);
                end
            end else begin
                $display("\n[PHASE D / TEST D] 跳过 —— 剩余预算 %0d ms 不足", avail);
            end
        end else begin
            $display("\n[PHASE D / TEST D] 跳过 —— 当前不在 BALANCE 态 (fsm_state=%0d)", u_dut.fsm_state);
        end

        // =====================================================================
        // PHASE D-ret: 从 +45° 平滑回正至 0° 定点
        // =====================================================================
        if (u_dut.fsm_state === 2'd2) begin
            phase_start_tick = tick_idx;
            $display("\n[PHASE D-ret] 按键切换 traj_mode 1 -> 2 -> 3 -> 0，平滑回正至 0° 定点...");
            // 连按 3 次 KEY2，切回 mode 0 (0° 定点)
            repeat (3) begin
                key_pos_clear = 1'b0;
                #KEY_PRESS_NS;
                key_pos_clear = 1'b1;
                wait_ctrl_ticks(5);
            end
            if (u_dut.traj_mode !== 2'd0) begin
                $display("  [WARN] 连按后 traj_mode = %0d != 0, 步进切换至 0...", u_dut.traj_mode);
                while (u_dut.traj_mode !== 2'd0) begin
                    key_pos_clear = 1'b0;
                    #KEY_PRESS_NS;
                    key_pos_clear = 1'b1;
                    wait_ctrl_ticks(5);
                end
            end
            $display("  已切回 mode 0 (当前 traj_mode = %0d), 等待斜坡降为 0°...", u_dut.traj_mode);
            ramp_guard = 0;
            while (u_dut.alpha_ref_q16 != 32'sd0 && ramp_guard < 3000) begin
                wait_ctrl_ticks(1);
                ramp_guard = ramp_guard + 1;
            end
            settle_guard = 0;
            while ((abs_al_now >= 1.0) && settle_guard < 3000) begin
                wait_ctrl_ticks(1);
                settle_guard = settle_guard + 1;
            end
            $display("  回正至 0° 完成，斜坡耗时 %0d ms，转臂沉降耗时 %0d ms (al=%0.3f°)，额外沉降 500 ms...",
                     ramp_guard, settle_guard, al_deg_now);
            wait_ctrl_ticks(500);
            ticks_used = ticks_used + (tick_idx - phase_start_tick);
        end

        // =====================================================================
        // PHASE E: 0.2Hz 正弦连续动态轨迹跟踪 + 倒立自平衡 (拓展要求 2)
        // =====================================================================
        if (u_dut.fsm_state === 2'd2) begin
            avail = hil_ctrl_cycles - ticks_used - setup_ticks;
            if (avail >= SINE_PLAN) begin
                test_e_run = 1;
                phase_start_tick = tick_idx;
                $display("\n[PHASE E / TEST E] 切换 traj_mode 0 -> 1 -> 2 -> 3，启动 0.2Hz 正弦动态跟踪 (幅值 25°, 周期 5000ms)...");
                sine_max_deg         = 0.0;
                sine_max_pos_err_deg = 0.0;
                sine_ss_sum_sq       = 0.0;
                sine_ss_cnt          = 0;

                // 连按 3 次 KEY2 切入 mode 3 (正弦模式)
                repeat (3) begin
                    key_pos_clear = 1'b0;
                    #KEY_PRESS_NS;
                    key_pos_clear = 1'b1;
                    wait_ctrl_ticks(5);
                end
                $display("  切入完成: traj_mode = %0d (期望 3)", u_dut.traj_mode);

                sine_win_open = 1'b1;
                wait_ctrl_ticks(SINE_PLAN);
                sine_win_open = 1'b0;
                ticks_used = ticks_used + (tick_idx - phase_start_tick);

                sine_rms_deg = (sine_ss_cnt > 0) ? $sqrt(sine_ss_sum_sq / sine_ss_cnt) : -1.0;

                $display("  ---- TEST E 正弦跟踪结果 ----");
                $display("  traj_mode             : %0d", u_dut.traj_mode);
                $display("  观测时长              : %0d ms (完整 1 周期)", SINE_PLAN);
                $display("  正弦跟踪 RMS 误差     : %8.4f deg (判据 < %.1f deg)", sine_rms_deg, SINE_RMS_LIMIT);
                $display("  过程最大位置跟踪误差  : %8.4f deg", sine_max_pos_err_deg);
                $display("  过程 |th| 峰值        : %8.3f deg (判据 < %.1f deg, 拓展要求 2: 动态过程中始终倒立自平衡)",
                         sine_max_deg, SINE_TH_LIMIT);
                $display("  跌落回退次数          : %0d", fall_count);
                $display("  PROTECT 停留 tick 数  : %0d", protect_ticks);
                $display("  FSM 末态              : %0d", u_dut.fsm_state);

                n_checks = n_checks + 1;
                if (u_dut.traj_mode === 2'd3 && sine_rms_deg >= 0.0 &&
                    sine_rms_deg < SINE_RMS_LIMIT && sine_max_deg < SINE_TH_LIMIT &&
                    fall_count == 0 && protect_ticks == 0 && u_dut.fsm_state === 2'd2) begin
                    test_e_pass = 1;
                    $display("  [PASS] TEST E: 0.2Hz 正弦动态轨迹跟踪达成 (RMS 误差 %8.4f deg), 摆杆始终稳定倒立 (|th| 峰值 %8.3f deg) (拓展要求 2)",
                             sine_rms_deg, sine_max_deg);
                end else begin
                    n_fail = n_fail + 1;
                    $display("  [FAIL] TEST E: 正弦跟踪或直立保持不达标 (mode=%0d, RMS=%8.4f deg, |th|峰值=%8.3f deg, fall=%0d, 末态=%0d)",
                             u_dut.traj_mode, sine_rms_deg, sine_max_deg, fall_count, u_dut.fsm_state);
                end
            end else begin
                $display("\n[PHASE E / TEST E] 跳过 —— 剩余预算 %0d ms 不足 (需至少 %0d ms)", avail, SINE_PLAN);
            end
        end else begin
            $display("\n[PHASE E / TEST E] 跳过 —— 当前不在 BALANCE 态 (fsm_state=%0d)", u_dut.fsm_state);
        end

        // ---------------- 用尽剩余预算 ----------------
        avail = hil_ctrl_cycles - ticks_used - setup_ticks;
        if (avail > 0) begin
            $display("\n[TAIL] 继续运行剩余 %0d ms 预算...", avail);
            wait_ctrl_ticks(avail);
            ticks_used = ticks_used + avail;
        end

        // =====================================================================
        // 总结
        // =====================================================================
        $display("\n=================================================================");
        $display("   HIL 闭环仿真总结");
        $display("=================================================================");
        $display("  总仿真物理时长       : %0d ms (setup %0d ms + 观测 %0d ms, 其中冻结摆位 %0d ms)",
                 setup_ticks + ticks_used, setup_ticks, ticks_used, freeze_ticks);
        expected_rk4 = (setup_ticks + ticks_used - freeze_ticks) * 5;
        $display("  RK4 累计步数         : %0d (自检: 应 = 5 x 非冻结物理时长 ms = %0d)",
                 plant_rk4_steps, expected_rk4);
        $display("  数值发散标志 diverged: %b (必须为 0, 否则闭环结论无效)", plant_diverged);
        $display("  FSM 末态             : %0d (0=HANGING 1=SWINGUP 2=BALANCE 3=PROTECT)", u_dut.fsm_state);
        $display("  进入 BALANCE 次数    : %0d, 跌落回退次数: %0d, PROTECT tick: %0d, soft_limit tick: %0d",
                 catch_count, fall_count, protect_ticks, softlimit_ticks);
        $display("  raw_dtheta 解卷绕假尖峰 : %0d 次 (|峰值| %0.1f rad/s) —— 自上一次 reset_stats 起",
                 dtheta_spike_n, dtheta_spike_peak);
        if (test_alt_used == 0) begin
            $display("  首次起摆耗时         : %0d ms", catch_tick);
        end else begin
            $display("  *** 警示: 下方 TEST B/C/D 的结果是在【PHASE A-alt 诊断性回退投放】下取得的 ***");
            $display("  ***       即摆杆并非由能量泵自行抬起, TEST A (基础要求 1) 已判 FAIL       ***");
            $display("  首次起摆耗时         : N/A —— 能量泵在 %0d ms 窗口内未能把摆杆抬进捕获区", SWING_MAX);
            $display("                         下方 %0d ms 是人工投放后 FSM 自行捕获的相对时刻", catch_tick);
            $display("                         (A-alt 后时间轴归零), **不代表起摆能力**");
        end
        $display("  累计 BALANCE 时长    : %0d ms", balance_ticks);
        $display("  |theta| 峰值(全程)   : %8.3f deg", max_abs_th_all_deg);
        $display("  |theta| 峰值(BALANCE): %8.3f deg", max_abs_th_bal_deg);
        $display("  |alpha| 峰值(全程)   : %8.3f deg", max_abs_alpha_deg);
        $display("  |pwm_duty| 峰值      : %0d / 1000", max_abs_pwm);
        $display("  末端 PLANT 状态      : theta=%8.3f deg, dtheta=%8.3f rad/s, alpha=%8.3f deg, dalpha=%8.3f rad/s",
                 th_deg_now, dth_now, al_deg_now, dal_now);
        $display("  末端 V_motor         : %8.1f mV", plant_vmotor_mv * 1.0);
        $display("-----------------------------------------------------------------");
        $display("  TEST 0 零点标定      : %s", (test_cal_pass == 1) ? "PASS" : "FAIL");
        $display("  TEST A 起摆(基础1)   : %s%s", (test_a_pass == 1) ? "PASS" : "FAIL",
                 (test_alt_used == 1) ? "   <== 赛题基础要求 1 在当前 RTL 下未达成" : "");
        $display("  TEST B 平衡保持(基础3): %s",
                 (test_b_run == 0) ? "SKIPPED (依赖 TEST A)" : ((test_b_pass == 1) ? "PASS" : "FAIL"));
        $display("  TEST C 抗扰(基础3)   : %s",
                 (test_c_run == 0) ? "SKIPPED" : ((test_c_pass == 1) ? "PASS" : "FAIL"));
        $display("  TEST D 定点直立(拓展1/3): %s",
                 (test_d_run == 0) ? "SKIPPED" : ((test_d_pass == 1) ? "PASS" : "FAIL"));
        $display("  TEST E 正弦跟踪(拓展2): %s",
                 (test_e_run == 0) ? "SKIPPED" : ((test_e_pass == 1) ? "PASS" : "FAIL"));
        $display("-----------------------------------------------------------------");
        n_checks = n_checks + 1;
        if (plant_diverged !== 1'b0 || plant_rk4_steps != expected_rk4) begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 被控对象数值完整性检查未通过 (diverged=%b, rk4=%0d)",
                     plant_diverged, plant_rk4_steps);
        end else begin
            $display("  [PASS] 被控对象数值完整性检查通过 (无发散, RK4 步数与物理时长严格一致)");
        end

        // 闭环全程传感器一致性复查 (加固假绿防线)
        if (sync_desync_n != 0 || sync_th_desync_n != 0) begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 闭环全程存在传感器与 PLANT 失步: pulse 失步 %0d 次 (峰值偏差 %0d), theta 失步 %0d 次 (峰值 %0d Q16)",
                     sync_desync_n, sync_pulse_peak, sync_th_desync_n, sync_th_peak);
        end else begin
            $display("  [PASS] 闭环全程传感器与 PLANT 同步复查通过 (pulse 偏差峰值 %0d, theta 偏差峰值 %0d Q16)",
                     sync_pulse_peak, sync_th_peak);
        end

        // 完整性断言: 防止中途跳过用例导致假绿 ("跳过即失败")
        if (n_checks < 10 || test_b_run == 0 || test_c_run == 0 || test_d_run == 0 || test_e_run == 0) begin
            n_fail = n_fail + 1;
            $display("  [FAIL] 存在未执行/被跳过的测试项或检查项数不足 (n_checks=%0d, 期望 10; b_run=%0d, c_run=%0d, d_run=%0d, e_run=%0d)",
                     n_checks, test_b_run, test_c_run, test_d_run, test_e_run);
        end
        $display("=================================================================");
        if (n_fail == 0)
            $display("  HIL RESULT: PASS  (%0d/%0d 项判据通过)", n_checks - n_fail, n_checks);
        else
            $display("  HIL RESULT: FAIL  (%0d/%0d 项判据通过, %0d 项失败)",
                     n_checks - n_fail, n_checks, n_fail);
        $display("=================================================================");
        $finish;
    end

    // -------------------------------------------------------------------------
    // 看门狗: 防止任何情况下仿真挂死 (预算 + 3000ms 余量)
    // -------------------------------------------------------------------------
    initial begin
        #1;  // 确保主线程已读完 plusarg
        wd_ns = (hil_ctrl_cycles + 3000) * 1000000.0;   // 1 控制周期 = 1ms = 1e6 ns
        #wd_ns;
        $display("\n*** HIL WATCHDOG TIMEOUT: 仿真超出预算仍未正常结束, 强制终止 ***");
        $display("*** 已运行 tick_idx = %0d ms, fsm_state = %0d, catch_tick = %0d ***",
                 tick_idx, u_dut.fsm_state, catch_tick);
        $display("  HIL RESULT: FAIL  (watchdog timeout)");
        $finish;
    end

endmodule
