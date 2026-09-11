// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: j280_hw_top
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: J280 旋转倒立摆姿态控制系统完整硬件顶层整合模块
//           将电机驱动、编码器测速测角、摆杆角度传感器、LQI 姿态控制核互联
//           集成:
//           1. 1ms (1000Hz) 精确数字控制节拍定时发生器与采样对齐流水线 (D6)
//           2. 一键硬件垂直零点自适应标定逻辑 (扶直摆杆按键即校准，免重烧程序)
//           3. 复合按键交互: 短按轮换控制模式(0°/±45°定点/0.2Hz正弦跟踪)，长按清零转臂位置
//           4. 起摆能量泵 (P1) 与 4 态控制状态机 (STATE_HANGING / SWINGUP / BALANCE / PROTECT)
//           5. 解决跌落保护结构性冲突 (P1/D2): 起摆态放行大角度，仅平衡态触发跌落自锁
//           6. 转臂多圈软限位保护 (D3) 防止电机电缆绞断
//           7. 无除法定点数积分分离与防饱和累加器 (D2 / E4)
//           8. 完备的状态 LED 指示与 TB6612FNG 能耗制动电机驱动 (D1)
// =============================================================================

module j280_hw_top #(
    parameter integer CLK_FREQ_HZ           = 50_000_000,          // 系统时钟频率
    parameter integer TIMER_1MS_LIMIT       = 50_000_000 / 1000,   // 1ms 控制节拍分频上限
    parameter integer CALIB_DEBOUNCE_CYCLES = 1_000_000,           // 标定按键消抖阈值 (20ms)
    parameter signed [31:0] ARM_SOFT_LIMIT_Q16 = 32'sd823548       // 转臂软限位阈值 (默认 ±2 圈)
)(
    // 系统时钟与复位
    input  wire                   clk_50m,        // 板载 50MHz 有源晶振
    input  wire                   rst_n,          // 硬件复位按键 (低有效)

    // 用户交互控制开关与按键
    input  wire                   sw_motor_en,    // 拨码开关: 电机使能 (1: 运行, 0: 安全停机)
    input  wire                   sw_brake_mode,  // 拨码开关: 停机模式 (1: 动态能耗制动, 0: 滑行)
    input  wire                   key_zero_calib, // 按键: 摆杆垂直零点标定触发 (低有效)
    input  wire                   key_pos_clear,  // 复合按键: 短按切换轨迹模式(0/1/2/3)，长按清零转臂位置 (低有效)

    // 直流电机驱动接口引脚 (连接 H 桥驱动芯片，如 TB6612FNG / A4950)
    output wire                   motor_pwm,      // 单端 PWM 脉冲引脚
    output wire                   motor_dir,      // 转向 DIR 引脚
    output wire                   motor_in1,      // H 桥 IN1 引脚
    output wire                   motor_in2,      // H 桥 IN2 引脚
    output wire                   motor_stby,     // TB6612 STBY 待机引脚 (高电平使能)

    // 电机正交光电编码器接口引脚
    input  wire                   enc_a,          // 编码器 A 相输入
    input  wire                   enc_b,          // 编码器 B 相输入
    input  wire                   enc_z,          // 编码器 Z 相输入 (可选)

    // 摆杆角度传感器 (12 位 SPI ADC / 磁编码器) 接口引脚
    output wire                   adc_cs_n,       // SPI 片选信号 (低有效)
    output wire                   adc_sclk,       // SPI 串行同步时钟
    input  wire                   adc_miso,       // SPI 串行数据输入

    // 板载状态指示 LED
    output wire                   led_balance,    // 倒立摆处于直立自平衡区指示灯 (低电平点亮/起摆闪烁)
    output wire                   led_calib_ok,   // 零位已标定指示灯 (低电平点亮/未标定慢闪)
    output wire                   led_motor_run   // 电机动力输出与报警指示灯 (低电平点亮/保护快闪)
);

    // -------------------------------------------------------------------------
    // 1. 1ms (1000Hz) 控制主时钟节拍发生器 (D6 采样对齐基准)
    // -------------------------------------------------------------------------
    reg [23:0] timer_1ms_cnt;
    reg        calc_en_pulse;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            timer_1ms_cnt <= 24'd0;
            calc_en_pulse <= 1'b0;
        end else begin
            if (timer_1ms_cnt >= TIMER_1MS_LIMIT - 1) begin
                timer_1ms_cnt <= 24'd0;
                calc_en_pulse <= 1'b1;
            end else begin
                timer_1ms_cnt <= timer_1ms_cnt + 1'b1;
                calc_en_pulse <= 1'b0;
            end
        end
    end

    // 状态指示闪烁分频器
    reg [24:0] blink_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            blink_cnt <= 25'd0;
        else
            blink_cnt <= blink_cnt + 1'b1;
    end
    wire blink_1hz  = blink_cnt[24]; // ~1.5 Hz
    wire blink_4hz  = blink_cnt[22]; // ~6 Hz
    wire blink_10hz = blink_cnt[21]; // ~12 Hz

    // -------------------------------------------------------------------------
    // 2. 按键消抖与垂直绝对零位自适应锁存逻辑 (KEY1)
    // -------------------------------------------------------------------------
    reg [23:0] calib_debounce_cnt;
    reg        calib_pressed;
    reg [11:0] zero_offset_reg;
    reg        zero_calibrated;
    reg        adc_has_sampled;

    wire [11:0] raw_adc_data;
    wire        sample_done;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            adc_has_sampled <= 1'b0;
        end else if (sample_done) begin
            adc_has_sampled <= 1'b1;
        end
    end

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            calib_debounce_cnt <= 24'd0;
            calib_pressed      <= 1'b0;
            zero_offset_reg    <= 12'd2048; // 默认初始中位 2048
            zero_calibrated    <= 1'b0;
        end else begin
            if (key_zero_calib == 1'b0) begin
                if (calib_debounce_cnt < CALIB_DEBOUNCE_CYCLES) begin
                    calib_debounce_cnt <= calib_debounce_cnt + 1'b1;
                end else if (!calib_pressed && adc_has_sampled) begin
                    // 确认按下且已有有效采样: 立即锁存当前摆杆 ADC 值为绝对零位
                    zero_offset_reg <= raw_adc_data;
                    calib_pressed   <= 1'b1;
                    zero_calibrated <= 1'b1;
                end
            end else begin
                calib_debounce_cnt <= 24'd0;
                calib_pressed      <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. 复合按键 KEY2: 短按模式轮换，长按绝对位置清零
    // -------------------------------------------------------------------------
    reg [25:0] key2_cnt;
    reg        clear_pos_pulse;
    reg [1:0]  traj_mode; // 0: 0°定点, 1: +45°定点, 2: -45°定点, 3: 0.2Hz正弦跟踪

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            key2_cnt        <= 26'd0;
            clear_pos_pulse <= 1'b0;
            traj_mode       <= 2'b00;
        end else begin
            clear_pos_pulse <= 1'b0;
            if (key_pos_clear == 1'b0) begin
                if (key2_cnt < 26'd50_000_000) begin
                    key2_cnt <= key2_cnt + 1'b1;
                end
                // 长按超过阈值触发转臂位置归零
                if (key2_cnt == CALIB_DEBOUNCE_CYCLES * 35) begin
                    clear_pos_pulse <= 1'b1;
                end
            end else begin
                // 短按释放 (消抖时间 ~ 长按阈值) 切换轨迹模式
                if (key2_cnt >= CALIB_DEBOUNCE_CYCLES && key2_cnt < CALIB_DEBOUNCE_CYCLES * 35) begin
                    traj_mode <= traj_mode + 1'b1;
                end
                key2_cnt <= 26'd0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 4. 例化: 摆杆角度传感器采集解算模块 (D6: 1ms calc_en 触发 SPI)
    // -------------------------------------------------------------------------
    wire signed [31:0] theta_err_q16;
    wire signed [31:0] dtheta_q16;

    angle_sensor_reader #(
        .CLK_FREQ_HZ   (50_000_000),
        .SPI_SCLK_HZ   (2_500_000),
        .ADC_RESOLUTION(12),
        .INVERT_DIR    (0)
    ) u_angle_sensor (
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .calc_en        (calc_en_pulse),
        .spi_cs_n      (adc_cs_n),
        .spi_sclk      (adc_sclk),
        .spi_miso      (adc_miso),
        .zero_offset_raw(zero_offset_reg),
        .ext_raw_valid (1'b0),
        .ext_raw_data  (12'd0),
        .raw_adc_data  (raw_adc_data),
        .theta_err_q16 (theta_err_q16),
        .dtheta_q16    (dtheta_q16),
        .sample_done   (sample_done)
    );

    // -------------------------------------------------------------------------
    // 5. 例化: 电机编码器 4 倍频与数字滤波测速测角模块 (D4/D5/E4)
    // -------------------------------------------------------------------------
    wire signed [31:0] pulse_count;
    wire signed [31:0] alpha_rad_q16;
    wire signed [31:0] dalpha_rad_s_q16;

    encoder_quad_reader #(
        .CLK_FREQ_HZ   (50_000_000),
        .FILTER_CYCLES (8),
        .CPR           (4000),
        .USE_Z_INDEX   (1),
        .REVERSE_DIR   (0)
    ) u_encoder (
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .enc_a_raw     (enc_a),
        .enc_b_raw     (enc_b),
        .enc_z_raw     (enc_z),
        .clear_pos      (clear_pos_pulse),
        .calc_en        (sample_done),       // 修复 F9: 统一由 sample_done 触发，严格与角度采样对齐
        .pulse_count    (pulse_count),
        .alpha_rad_q16  (alpha_rad_q16),
        .dalpha_rad_s_q16(dalpha_rad_s_q16),
        .speed_pps      (),
        .dir_flag       ()
    );

    // -------------------------------------------------------------------------
    // 6. 转臂软限位保护判定 (D3) 防止连续单向旋转绞断电机引线
    // -------------------------------------------------------------------------
    wire soft_limit_err = (alpha_rad_q16 > ARM_SOFT_LIMIT_Q16) || (alpha_rad_q16 < -ARM_SOFT_LIMIT_Q16);

    // -------------------------------------------------------------------------
    // 7. 例化: 定点与正弦轨迹发生器 (P2 / 拓展要求 1 & 2 / F8 平滑与同步)
    // -------------------------------------------------------------------------
    wire signed [31:0] alpha_ref_q16;
    wire signed [31:0] dalpha_ref_q16;
    wire               traj_sync_pulse;

    traj_gen #(
        .CLK_FREQ_HZ(50_000_000),
        .UPDATE_HZ  (1000)
    ) u_traj_gen (
        .clk          (clk_50m),
        .rst_n        (rst_n),
        .calc_en      (sample_done),
        .sync_phase   (traj_sync_pulse),    // 修复 F8: 平衡切入瞬间复位正弦相位
        .mode_sel     (traj_mode),
        .alpha_ref_q16(alpha_ref_q16),
        .dalpha_ref_q16(dalpha_ref_q16)
    );

    // 状态误差解算 (引入轨迹位置与速度前馈偏差)
    wire signed [31:0] alpha_err_q16  = alpha_rad_q16 - alpha_ref_q16;
    wire signed [31:0] dalpha_err_q16 = dalpha_rad_s_q16 - dalpha_ref_q16;

    // -------------------------------------------------------------------------
    // 8. 声明内部连线并例化子模块
    // -------------------------------------------------------------------------
    wire [1:0]         fsm_state;
    wire               lqr_en;
    wire               swing_en;
    wire               fsm_motor_active;
    wire               reset_integral_pulse;
    wire               is_in_balance_zone;
    wire signed [15:0] final_pwm_duty;

    wire signed [15:0] swing_pwm_duty;
    wire               swing_calc_done;
    wire               swing_energy_deficit;

    wire signed [15:0] lqr_pwm_duty;
    wire               lqr_calc_done;

    // -------------------------------------------------------------------------
    // 9. LQI 转臂位置积分器 (D2 积分分离与 E4 无除法防饱和累加)
    // -------------------------------------------------------------------------
    // 积分饱和限幅: [-0.25, +0.25] rad -> Q16: 16384
    localparam signed [31:0] INT_LIMIT_Q16      = 32'sd16384;
    // 积分分离窗口: 仅当处于平衡区且转臂误差在 ±10° (0.1745 rad -> Q16: 11439) 内累加
    localparam signed [31:0] INT_ERR_THRESH_Q16 = 32'sd11439;
    reg signed [31:0] alpha_int_q16;

    // 消除乘法器与除法器: 1ms 积分步长通过高精度无乘法移位累加实现 (1/1024 + 1/65536 + 1/131072 ~= 0.001000, 误差 0.05%)
    wire signed [31:0] int_step_q16 = (alpha_err_q16 >>> 10) + (alpha_err_q16 >>> 16) + (alpha_err_q16 >>> 17);

    wire int_accum_en = lqr_en && (alpha_err_q16 > -INT_ERR_THRESH_Q16) && (alpha_err_q16 < INT_ERR_THRESH_Q16);

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            alpha_int_q16 <= 32'sd0;
        end else if (!fsm_motor_active || !lqr_en || reset_integral_pulse || soft_limit_err) begin
            alpha_int_q16 <= 32'sd0; // 停机、非平衡态、捕获切入瞬间或超限时清零积分 (D2)
        end else if (sample_done) begin
            if (int_accum_en) begin
                if (alpha_int_q16 + int_step_q16 > INT_LIMIT_Q16)
                    alpha_int_q16 <= INT_LIMIT_Q16;
                else if (alpha_int_q16 + int_step_q16 < -INT_LIMIT_Q16)
                    alpha_int_q16 <= -INT_LIMIT_Q16;
                else
                    alpha_int_q16 <= alpha_int_q16 + int_step_q16;
            end else begin
                alpha_int_q16 <= 32'sd0; // 误差偏大时分离积分，彻底消除饱和超调 (D2)
            end
        end
    end

    // -------------------------------------------------------------------------
    // 10. 例化: 起摆能量泵控制器 (P1 / F1 / F2 / F3 / F4)
    // -------------------------------------------------------------------------
    swing_up_ctrl u_swing_ctrl (
        .clk             (clk_50m),
        .rst_n           (rst_n),
        .calc_en         (sample_done),
        .theta_rad_q16   (theta_err_q16),
        .dtheta_rad_s_q16(dtheta_q16),
        .alpha_rad_q16   (alpha_rad_q16),
        .dalpha_rad_s_q16(dalpha_rad_s_q16),
        .pwm_swing_duty  (swing_pwm_duty),
        .energy_deficit  (swing_energy_deficit), // 修复 F2: 引出机械能指示
        .calc_done       (swing_calc_done)
    );

    // -------------------------------------------------------------------------
    // 11. 例化: LQR / LQI 定点数硬件乘加流水线计算核 (F7: 常开计算消除捕获空窗)
    // -------------------------------------------------------------------------
    furuta_lqr_ctrl u_lqr_core (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .calc_en    (sample_done),               // 修复 F7: 去掉 && lqr_en，常开流水，切入瞬间当拍即可输出有效占空比
        .theta_err  (theta_err_q16),
        .dtheta     (dtheta_q16),
        .alpha_err  (alpha_err_q16),
        .dalpha     (dalpha_err_q16),
        .alpha_int  (alpha_int_q16),
        .pwm_duty   (lqr_pwm_duty),
        .calc_done  (lqr_calc_done)
    );

    // -------------------------------------------------------------------------
    // 12. 例化: 四态主控制状态机 (P1 / F5 / F6 / F8)
    // -------------------------------------------------------------------------
    ctrl_fsm u_ctrl_fsm (
        .clk                 (clk_50m),
        .rst_n               (rst_n),
        .calc_en             (sample_done),
        .motor_en_sw         (sw_motor_en),
        .calib_done          (zero_calibrated),
        .soft_limit_err      (soft_limit_err),
        .theta_err_q16       (theta_err_q16),
        .dtheta_q16          (dtheta_q16),
        .pwm_swing_duty      (swing_pwm_duty),
        .pwm_lqr_duty        (lqr_pwm_duty),
        .current_state       (fsm_state),
        .final_pwm_duty      (final_pwm_duty),
        .reset_integral_pulse(reset_integral_pulse),
        .traj_sync_pulse     (traj_sync_pulse),  // 修复 F8: 捕获切入瞬间同步轨迹发生器
        .is_in_balance_zone  (is_in_balance_zone),
        .lqr_en              (lqr_en),
        .swing_en            (swing_en),
        .motor_active        (fsm_motor_active)
    );

    // -------------------------------------------------------------------------
    // 13. 例化: 电机 PWM 驱动发生器与 H 桥接口模块 (D1: 能耗制动 STBY 常保)
    // -------------------------------------------------------------------------
    motor_pwm_driver #(
        .CLK_FREQ_HZ   (50_000_000),
        .PWM_FREQ_HZ   (20_000),
        .DUTY_MAX_VAL  (1000),
        .DEAD_BAND_VAL (0)
    ) u_motor_driver (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .motor_en   (fsm_motor_active),
        .brake_mode (sw_brake_mode),
        .pwm_duty   (final_pwm_duty),
        .pwm_out    (motor_pwm),
        .dir_out    (motor_dir),
        .in1_out    (motor_in1),
        .in2_out    (motor_in2),
        .stby_out   (motor_stby)
    );

    // -------------------------------------------------------------------------
    // 14. 板载状态指示 LED 输出 (低电平点亮)
    // -------------------------------------------------------------------------
    // led_balance: BALANCE 态常亮(0)，SWINGUP 态快闪，其它熄灭(1)
    assign led_balance   = (fsm_state == 2'd2) ? 1'b0 :
                           (fsm_state == 2'd1) ? blink_4hz : 1'b1;

    // led_calib_ok: 标定成功常亮(0)，未标定慢闪提示(1.5Hz)
    assign led_calib_ok  = zero_calibrated ? 1'b0 : blink_1hz;

    // led_motor_run: 保护自锁态警报快闪(10Hz)，正常工作有动力输出常亮(0)，其余熄灭(1)
    assign led_motor_run = (fsm_state == 2'd3) ? blink_10hz :
                           (fsm_motor_active && (final_pwm_duty != 16'sd0)) ? 1'b0 : 1'b1;

endmodule
