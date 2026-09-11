// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一
// 模块名称: furuta_lqr_ctrl
// 芯片型号: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 旋转倒立摆姿态自平衡 LQI/LQR 定点数硬件乘加流水线计算核
// 算术精度: 32-bit 有符号定点数 (Q12.16 格式)
// 计算时钟: 50MHz 系统主频下 3 级流水线，确定性延迟 60ns (0 抖动)
// =============================================================================

module furuta_lqr_ctrl (
    input  wire        clk,            // 系统主时钟 (推荐 50MHz)
    input  wire        rst_n,          // 异步复位 (低电平有效)
    input  wire        calc_en,        // 1ms 控制节拍计算使能脉冲

    // 状态输入 (Q12.16 定点数，已由传感器采集与滤波模块处理好)
    input  wire signed [31:0] theta_err,   // 摆杆倾角偏差 (rad)
    input  wire signed [31:0] dtheta,      // 摆杆角速度 (rad/s)
    input  wire signed [31:0] alpha_err,   // 转臂位置偏差 (rad, alpha - alpha_target)
    input  wire signed [31:0] dalpha,      // 转臂角速度 (rad/s)
    input  wire signed [31:0] alpha_int,   // 转臂积分误差 (rad*s, 消除静差)

    // 控制输出
    output reg  signed [15:0] pwm_duty,    // 输出至电机驱动模块的 PWM 占空比 [-1000, +1000]
    output reg                calc_done    // 计算完成脉冲
);

    // -------------------------------------------------------------------------
    // LQR / LQI 状态反馈增益常量 (Q12.16 格式)
    // -------------------------------------------------------------------------
    localparam signed [31:0] K1 = -32'sd5388751; // -82.2258 (摆角增益)
    localparam signed [31:0] K2 = -32'sd625050; // -9.5375 (摆角速度增益)
    localparam signed [31:0] K3 = -32'sd422594; // -6.4483 (转臂位置增益)
    localparam signed [31:0] K4 = -32'sd296554; // -4.5250 (转臂速度增益)
    localparam signed [31:0] K5 = -32'sd131072; // -2.0000 (积分项增益)

    // 电压到 PWM 占空比缩放因子 (12V 对应 1000 计数值: 1000 / 12 ~= 83.33)
    // 在 Q16 格式下: 83.33 * 65536 ~= 5461163
    localparam signed [31:0] VOLT_TO_PWM = 32'sd5461163;
    localparam signed [15:0] PWM_MAX     = 16'sd1000;
    localparam signed [15:0] PWM_MIN     = -16'sd1000;

    // -------------------------------------------------------------------------
    // 流水线第 1 级: 多路并行乘法 (调用高云 GW2A DSP28x28 或 MULT18x18 硬核)
    // -------------------------------------------------------------------------
    reg signed [63:0] prod1, prod2, prod3, prod4, prod5;
    reg               stage1_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod1        <= 64'd0;
            prod2        <= 64'd0;
            prod3        <= 64'd0;
            prod4        <= 64'd0;
            prod5        <= 64'd0;
            stage1_valid <= 1'b0;
        end else if (calc_en) begin
            prod1        <= theta_err * K1;
            prod2        <= dtheta    * K2;
            prod3        <= alpha_err * K3;
            prod4        <= dalpha    * K4;
            prod5        <= - ($signed({{32{alpha_int[31]}}, alpha_int}) <<< 17); // K5 = -131072 = -2^17, 消除 32x32 乘法器
            stage1_valid <= 1'b1;
        end else begin
            stage1_valid <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线第 2 级: 累加求和并舍入右移 16 位恢复定点数尺度
    // V = -(K1*theta + K2*dtheta + K3*alpha + K4*dalpha + K5*alpha_int)
    // -------------------------------------------------------------------------
    reg signed [31:0] v_cmd_q16;
    reg               stage2_valid;

    wire signed [63:0] sum_prods = prod1 + prod2 + prod3 + prod4 + prod5;
    wire signed [63:0] v_cmd_64  = - (sum_prods >>> 16);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_cmd_q16    <= 32'sd0;
            stage2_valid <= 1'b0;
        end else if (stage1_valid) begin
            // 显式饱和保护与位宽截断规避 (消除 EX3791 警告与符号反转 D8)
            if (v_cmd_64 > 64'sd13107200)        // +200V in Q16
                v_cmd_q16 <= 32'sd13107200;
            else if (v_cmd_64 < -64'sd13107200) // -200V in Q16
                v_cmd_q16 <= -32'sd13107200;
            else
                v_cmd_q16 <= v_cmd_64[31:0];
            stage2_valid <= 1'b1;
        end else begin
            stage2_valid <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线第 3 级: 映射至 PWM 占空比并执行硬件饱和限幅 (Q16*Q16 -> Q32, 右移 32 位取整)
    // -------------------------------------------------------------------------
    wire signed [63:0] pwm_mult = $signed(v_cmd_q16) * $signed(VOLT_TO_PWM);
    wire signed [63:0] pwm_calc = pwm_mult >>> 32;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_duty  <= 16'd0;
            calc_done <= 1'b0;
        end else if (stage2_valid) begin
            // 饱和限幅保护电机与 H 桥 (避免溢出卷绕)
            if (pwm_calc > 64'sd1000)
                pwm_duty <= PWM_MAX;
            else if (pwm_calc < -64'sd1000)
                pwm_duty <= PWM_MIN;
            else
                pwm_duty <= pwm_calc[15:0];
            calc_done <= 1'b1;
        end else begin
            calc_done <= 1'b0;
        end
    end

endmodule
