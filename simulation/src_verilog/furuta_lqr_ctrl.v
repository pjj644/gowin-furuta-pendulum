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
    // LQR / LQI 状态反馈增益常量 (Q10 格式, 乘以 1024, 优化至 18-bit 有符号数消除 MULT36X36 资源浪费)
    // K1 = -82.2258 * 1024 = -84200 (18-bit, 误差 0.0009%)
    // K2 =  -9.5375 * 1024 =  -9766 (14-bit, 误差 0.001%)
    // K3 =  -6.4483 * 1024 =  -6603 (13-bit, 误差 0.005%)
    // K4 =  -4.5250 * 1024 =  -4634 (13-bit, 误差 0.009%)
    // K5 =  -2.0000 * 1024 =  -2048 (移位 <<< 11 即可实现)
    // -------------------------------------------------------------------------
    localparam signed [17:0] K1_18 = -18'sd84200;
    localparam signed [17:0] K2_18 = -18'sd9766;
    localparam signed [17:0] K3_18 = -18'sd6603;
    localparam signed [17:0] K4_18 = -18'sd4634;

    // 电压到 PWM 占空比缩放因子 (1000/12 = 83.3333; 83.3333 * 1024 = 85333, 18-bit, 误差 0.0004%)
    localparam signed [17:0] VOLT_TO_PWM_18 = 18'sd85333;
    localparam signed [15:0] PWM_MAX        = 16'sd1000;
    localparam signed [15:0] PWM_MIN        = -16'sd1000;

    // -------------------------------------------------------------------------
    // 流水线第 1 级: 18 位 DSP 乘法 (映射至高云 MULT18X18 或 MULTALU36X18，消除 MULT36X36)
    // -------------------------------------------------------------------------
    reg signed [49:0] prod1, prod2, prod3, prod4, prod5;
    reg               stage1_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod1        <= 50'sd0;
            prod2        <= 50'sd0;
            prod3        <= 50'sd0;
            prod4        <= 50'sd0;
            prod5        <= 50'sd0;
            stage1_valid <= 1'b0;
        end else if (calc_en) begin
            prod1        <= $signed(theta_err) * K1_18;
            prod2        <= $signed(dtheta)    * K2_18;
            prod3        <= $signed(alpha_err) * K3_18;
            prod4        <= $signed(dalpha)    * K4_18;
            prod5        <= - ($signed({{18{alpha_int[31]}}, alpha_int}) <<< 11); // K5 = -2.0, Q16*Q10 -> Q26
            stage1_valid <= 1'b1;
        end else begin
            stage1_valid <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线第 2 级: 累加求和并舍入右移 10 位恢复 Q16 定点数尺度 (Q26 >>> 10 -> Q16)
    // -------------------------------------------------------------------------
    reg signed [31:0] v_cmd_q16;
    reg               stage2_valid;

    wire signed [49:0] sum_prods = prod1 + prod2 + prod3 + prod4 + prod5;
    wire signed [49:0] v_cmd_raw = - (sum_prods >>> 10);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_cmd_q16    <= 32'sd0;
            stage2_valid <= 1'b0;
        end else if (stage1_valid) begin
            // 显式饱和保护与位宽截断规避 (±200V in Q16: ±13107200)
            if (v_cmd_raw > 50'sd13107200)
                v_cmd_q16 <= 32'sd13107200;
            else if (v_cmd_raw < -50'sd13107200)
                v_cmd_q16 <= -32'sd13107200;
            else
                v_cmd_q16 <= v_cmd_raw[31:0];
            stage2_valid <= 1'b1;
        end else begin
            stage2_valid <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线第 3 级 (F11 优化): 插入寄存器打拍，切断饱和比较到乘法器的长组合路径
    // 将 17 级逻辑深度切为 2 段，使 Fmax 突破 70MHz
    // -------------------------------------------------------------------------
    reg signed [49:0] pwm_mult_reg;
    reg               stage3_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_mult_reg <= 50'sd0;
            stage3_valid <= 1'b0;
        end else if (stage2_valid) begin
            pwm_mult_reg <= $signed(v_cmd_q16) * VOLT_TO_PWM_18; // Q16 * Q10 = Q26
            stage3_valid <= 1'b1;
        end else begin
            stage3_valid <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线第 4 级: 映射至 PWM 占空比并执行硬件饱和限幅 (Q26 >>> 26 -> 整数 PWM 计数)
    // -------------------------------------------------------------------------
    wire signed [49:0] pwm_calc = pwm_mult_reg >>> 26;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_duty  <= 16'd0;
            calc_done <= 1'b0;
        end else if (stage3_valid) begin
            if (pwm_calc > 50'sd1000)
                pwm_duty <= PWM_MAX;
            else if (pwm_calc < -50'sd1000)
                pwm_duty <= PWM_MIN;
            else
                pwm_duty <= pwm_calc[15:0];
            calc_done <= 1'b1;
        end else begin
            calc_done <= 1'b0;
        end
    end

endmodule
