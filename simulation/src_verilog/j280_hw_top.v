// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: j280_hw_top
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: J280 旋转倒立摆姿态控制系统完整硬件顶层整合模块
//           将电机驱动、编码器测速测角、摆杆角度传感器、LQI 姿态控制核互联
//           集成:
//           1. 1ms (1000Hz) 精确数字控制节拍定时发生器
//           2. 一键硬件垂直零点自适应标定逻辑 (扶直摆杆按键即校准，免重烧程序)
//           3. 转臂位置积分器 (LQI 消除死区静差，含积分分离与防饱和限幅)
//           4. 倾角超限跌落保护自锁机制 (超限自动封锁 PWM，杜绝甩杆打手)
//           5. 状态 LED 与引脚驱动接口
// =============================================================================

module j280_hw_top #(
    parameter integer CLK_FREQ_HZ           = 50_000_000,          // 系统时钟频率
    parameter integer TIMER_1MS_LIMIT       = 50_000_000 / 1000,   // 1ms 控制节拍分频上限
    parameter integer CALIB_DEBOUNCE_CYCLES = 1_000_000            // 标定按键消抖阈值 (20ms)
)(
    // 系统时钟与复位
    input  wire                   clk_50m,        // 板载 50MHz 有源晶振
    input  wire                   rst_n,          // 硬件复位按键 (低有效)

    // 用户交互控制开关与按键
    input  wire                   sw_motor_en,    // 拨码开关: 电机使能 (1: 运行, 0: 安全停机)
    input  wire                   sw_brake_mode,  // 拨码开关: 停机模式 (1: 动态能耗制动, 0: 滑行)
    input  wire                   key_zero_calib, // 按键: 摆杆垂直零点标定触发 (低有效)
    input  wire                   key_pos_clear,  // 按键: 转臂绝对位置清零 (低有效)

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
    output wire                   led_balance,    // 倒立摆处于直立自平衡区指示灯 (低电平点亮)
    output wire                   led_calib_ok,   // 零位已标定指示灯 (低电平点亮)
    output wire                   led_motor_run   // 电机正在输出动力指示灯 (低电平点亮)
);

    // -------------------------------------------------------------------------
    // 1. 1ms (1000Hz) 控制主时钟节拍发生器
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

    // -------------------------------------------------------------------------
    // 2. 按键消抖与垂直绝对零位自适应锁存逻辑
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
    // 3. 例化: 摆杆角度传感器采集解算模块
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
    // 4. 例化: 电机编码器 4 倍频与数字滤波测速测角模块
    // -------------------------------------------------------------------------
    wire signed [31:0] pulse_count;
    wire signed [31:0] alpha_rad_q16;
    wire signed [31:0] dalpha_rad_s_q16;

    encoder_quad_reader #(
        .CLK_FREQ_HZ   (50_000_000),
        .FILTER_CYCLES (8),
        .CPR           (4000),
        .REVERSE_DIR   (0)
    ) u_encoder (
        .clk            (clk_50m),
        .rst_n          (rst_n),
        .enc_a_raw     (enc_a),
        .enc_b_raw     (enc_b),
        .enc_z_raw     (enc_z),
        .clear_pos      (~key_pos_clear),
        .calc_en        (calc_en_pulse),
        .pulse_count    (pulse_count),
        .alpha_rad_q16  (alpha_rad_q16),
        .dalpha_rad_s_q16(dalpha_rad_s_q16),
        .speed_pps      (),
        .dir_flag       ()
    );

    // -------------------------------------------------------------------------
    // 5. 转臂目标位置设定与积分消除静差逻辑
    // -------------------------------------------------------------------------
    // 目标位置: 默认为 0.0 rad (垂直中位)。若做拓展要求 1，可从外部拔码开关/串口注入
    wire signed [31:0] alpha_target_q16 = 32'sd0;
    wire signed [31:0] alpha_err_q16    = alpha_rad_q16 - alpha_target_q16;

    // LQI 积分项限幅与分离:
    // 积分饱和限幅: [-0.25, +0.25] rad -> Q16: 0.25 * 65536 = 16384
    localparam signed [31:0] INT_LIMIT_Q16 = 32'sd16384;
    reg signed [31:0] alpha_int_q16;

    // 倒立自平衡区域判定: 摆角误差在 22 度以内 (约 0.384 rad -> Q16: 25166)
    localparam signed [31:0] BAL_ZONE_Q16 = 32'sd25166;
    wire is_in_balance = (theta_err_q16 > -BAL_ZONE_Q16) && (theta_err_q16 < BAL_ZONE_Q16);

    // 跌落安全保护判定: 摆角误差绝对值超过 45 度 (约 0.785 rad -> Q16: 51472) 则切断动力
    localparam signed [31:0] FALL_ZONE_Q16 = 32'sd51472;
    wire is_fall_down = (theta_err_q16 > FALL_ZONE_Q16) || (theta_err_q16 < -FALL_ZONE_Q16);

    // 控制计算触发脉冲: 当摆杆角度传感器 SPI 采样完成时立即启动 LQI 计算与积分累加 (零延迟控制)
    wire ctrl_calc_pulse = sample_done;

    // 积分分离累加: 仅在小倾角平衡区间内累加转臂位置误差，消除机械死区静差
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            alpha_int_q16 <= 32'sd0;
        end else if (!sw_motor_en || is_fall_down) begin
            alpha_int_q16 <= 32'sd0; // 停机或倒下时清零积分
        end else if (ctrl_calc_pulse && is_in_balance) begin
            // 累加步长 dt = 0.001s
            if (alpha_int_q16 + (alpha_err_q16 / 32'sd1000) > INT_LIMIT_Q16)
                alpha_int_q16 <= INT_LIMIT_Q16;
            else if (alpha_int_q16 + (alpha_err_q16 / 32'sd1000) < -INT_LIMIT_Q16)
                alpha_int_q16 <= -INT_LIMIT_Q16;
            else
                alpha_int_q16 <= alpha_int_q16 + (alpha_err_q16 / 32'sd1000);
        end
    end

    // -------------------------------------------------------------------------
    // 6. 例化: LQR / LQI 定点数硬件乘加流水线计算核
    // -------------------------------------------------------------------------
    wire signed [15:0] lqr_pwm_duty;
    wire               calc_done;

    furuta_lqr_ctrl u_lqr_core (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .calc_en    (ctrl_calc_pulse),
        .theta_err  (theta_err_q16),
        .dtheta     (dtheta_q16),
        .alpha_err  (alpha_err_q16),
        .dalpha     (dalpha_rad_s_q16),
        .alpha_int  (alpha_int_q16),
        .pwm_duty   (lqr_pwm_duty),
        .calc_done  (calc_done)
    );

    // 安全联锁门控: 仅在电机使能且未严重跌落翻倒时放行控制占空比
    wire signed [15:0] safe_pwm_duty = (sw_motor_en && !is_fall_down) ? lqr_pwm_duty : 16'sd0;
    wire               motor_active  = sw_motor_en && !is_fall_down;

    // -------------------------------------------------------------------------
    // 7. 例化: 电机 PWM 驱动发生器与 H 桥接口模块
    // -------------------------------------------------------------------------
    motor_pwm_driver #(
        .CLK_FREQ_HZ   (50_000_000),
        .PWM_FREQ_HZ   (20_000),
        .DUTY_MAX_VAL  (1000),
        .DEAD_BAND_VAL (0)
    ) u_motor_driver (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .motor_en   (motor_active),
        .brake_mode (sw_brake_mode),
        .pwm_duty   (safe_pwm_duty),
        .pwm_out    (motor_pwm),
        .dir_out    (motor_dir),
        .in1_out    (motor_in1),
        .in2_out    (motor_in2),
        .stby_out   (motor_stby)
    );

    // -------------------------------------------------------------------------
    // 8. 状态指示 LED 输出 (低电平点亮)
    // -------------------------------------------------------------------------
    assign led_balance   = ~is_in_balance;
    assign led_calib_ok  = ~zero_calibrated;
    assign led_motor_run = ~(motor_active && (safe_pwm_duty != 0));

endmodule
