// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: motor_pwm_driver
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 直流电机 PWM 驱动发生器与 H 桥接口控制器
//           1. 支持有符号 16 位占空比输入 [-1000, +1000] (与 LQR 控制器无缝对接)
//           2. 生成 20kHz 超音频高频 PWM (彻底消除人耳可闻的高频啸叫蜂鸣声)
//           3. 提供双模式 H 桥输出:
//              - 模式 A: PWM + 方向信号 (DIR / PWM)
//              - 模式 B: 互补双相信号 (IN1 / IN2, 兼容 TB6612FNG, A4950, DRV8833, L298N)
//           4. 包含硬件电机急停保护使能与刹车 (Brake) / 滑行 (Coast) 模式切换
// =============================================================================

module motor_pwm_driver #(
    parameter integer CLK_FREQ_HZ   = 50_000_000, // 系统主时钟频率 (默认 50MHz)
    parameter integer PWM_FREQ_HZ   = 20_000,     // PWM 载波频率 (默认 20kHz)
    parameter integer DUTY_MAX_VAL  = 1000,       // 占空比最大计数值 (对应 100% 满占空比)
    parameter integer DEAD_BAND_VAL = 0           // 硬件死区补偿值 (单位: 占空比计数值)
)(
    input  wire                   clk,            // 系统时钟 (50MHz)
    input  wire                   rst_n,          // 异步低电平复位
    input  wire                   motor_en,       // 电机使能 (1: 运行, 0: 紧急停机)
    input  wire                   brake_mode,     // 停机模式 (1: 动态能耗制动/刹车, 0: 自由滑行)
    input  wire signed [15:0]     pwm_duty,       // 有符号占空比 [-DUTY_MAX_VAL, +DUTY_MAX_VAL]

    // 输出信号: 兼容单信号 PWM+DIR 驱动器
    output reg                    pwm_out,        // 单端 PWM 脉冲输出
    output reg                    dir_out,        // 方向控制 (0: 正转, 1: 反转)

    // 输出信号: 兼容典型 H 桥芯片 (TB6612FNG, A4950, DRV8833)
    output reg                    in1_out,        // H 桥输入引脚 1
    output reg                    in2_out,        // H 桥输入引脚 2
    output wire                   stby_out        // TB6612 专用待机休眠引脚 (高电平工作)
);

    // -------------------------------------------------------------------------
    // 计数周期计算:
    // 50MHz / 20kHz = 2500 个时钟周期
    // -------------------------------------------------------------------------
    localparam integer TIMER_PERIOD = CLK_FREQ_HZ / PWM_FREQ_HZ;
    // 占空比换算比例: 2500 / 1000 = 2.5 (即 duty_abs * 2500 / 1000 = (duty_abs * 5) / 2)
    localparam integer TIMER_WIDTH  = 16;

    reg [TIMER_WIDTH-1:0] pwm_counter;
    reg [TIMER_WIDTH-1:0] compare_val;
    reg                   dir_reg;
    reg                   pwm_active;

    // TB6612FNG 待机使能逻辑:
    // 当系统使能时 (motor_en=1) 置 1 正常运行;
    // 当停机 (motor_en=0) 且要求动态能耗制动时 (brake_mode=1), 必须保持 STBY=1, 使得 IN1=IN2=1 短路刹车生效;
    // 仅在停机且选择自由滑行 (brake_mode=0) 时拉低 STBY=0 进入高阻待机状态。
    assign stby_out = motor_en | brake_mode;

    // -------------------------------------------------------------------------
    // 1. 输入占空比预处理与限幅 (去除负号，提取方向)
    // -------------------------------------------------------------------------
    wire [15:0] duty_abs = (pwm_duty < 0) ? (-pwm_duty) : pwm_duty;
    wire [15:0] duty_clamped = (duty_abs > DUTY_MAX_VAL) ? DUTY_MAX_VAL : duty_abs;
    wire        dir_detected = (pwm_duty < 0) ? 1'b1 : 1'b0;

    // 死区补偿计算
    wire [15:0] duty_compensated = (duty_clamped == 16'd0) ? 16'd0 :
                                   ((duty_clamped + DEAD_BAND_VAL > DUTY_MAX_VAL) ?
                                    DUTY_MAX_VAL : (duty_clamped + DEAD_BAND_VAL));

    // 计算实际比较门限值: 消除硬件运行时除法器 (E4 整改)
    // 采用编译期常数 Q16 比例系数 SCALE_Q16 = (TIMER_PERIOD * 65536) / DUTY_MAX_VAL
    // 在 50MHz / 20kHz (TIMER_PERIOD=2500, DUTY_MAX=1000) 下, SCALE_Q16 = 163840 (即精准 2.5)
    localparam [31:0] SCALE_FACTOR_Q16 = (TIMER_PERIOD * 65536) / DUTY_MAX_VAL;
    wire [31:0] compare_mult = duty_compensated * SCALE_FACTOR_Q16;
    wire [31:0] raw_compare  = compare_mult >> 16;

    // -------------------------------------------------------------------------
    // 2. 无毛刺影子寄存器更新 (在 PWM 周期归零时刻同步更新比较阈值)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= {TIMER_WIDTH{1'b0}};
            compare_val <= {TIMER_WIDTH{1'b0}};
            dir_reg     <= 1'b0;
        end else begin
            if (pwm_counter >= TIMER_PERIOD - 1) begin
                pwm_counter <= {TIMER_WIDTH{1'b0}};
                // 周期边界更新比较值，避免占空比调整时产生中途斩波毛刺
                compare_val <= raw_compare[TIMER_WIDTH-1:0];
                dir_reg     <= dir_detected;
            end else begin
                pwm_counter <= pwm_counter + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. PWM 脉冲生成逻辑
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_active <= 1'b0;
        end else begin
            if (compare_val == 0) begin
                pwm_active <= 1'b0; // 0 占空比输出完全低电平
            end else if (pwm_counter < compare_val) begin
                pwm_active <= 1'b1;
            end else begin
                pwm_active <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 4. 驱动引脚电平映射逻辑 (涵盖停机、刹车、正转、反转)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_out <= 1'b0;
            dir_out <= 1'b0;
            in1_out <= 1'b0;
            in2_out <= 1'b0;
        end else if (!motor_en) begin
            // 电机停机保护状态
            pwm_out <= 1'b0;
            dir_out <= 1'b0;
            if (brake_mode) begin
                // 刹车模式: 下管全部导通接地 (IN1=1, IN2=1 配合 TB6612 或短接电机端子形成闭环反电动势阻尼)
                in1_out <= 1'b1;
                in2_out <= 1'b1;
            end else begin
                // 滑行模式: 全部 MOS 管截止 (高阻态)
                in1_out <= 1'b0;
                in2_out <= 1'b0;
            end
        end else begin
            // 正常输出状态
            pwm_out <= pwm_active;
            dir_out <= dir_reg;

            if (dir_reg == 1'b0) begin
                // 正转: IN1 输出 PWM，IN2 置 0
                in1_out <= pwm_active;
                in2_out <= 1'b0;
            end else begin
                // 反转: IN1 置 0，IN2 输出 PWM
                in1_out <= 1'b0;
                in2_out <= pwm_active;
            end
        end
    end

endmodule
