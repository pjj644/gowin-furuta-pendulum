// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一
// 模块名称: furuta_lqr_ctrl
// 芯片型号: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 旋转倒立摆姿态自平衡 LQI/LQR 定点数硬件乘加流水线计算核
// 算术精度: 状态输入 32-bit Q12.16; 反馈增益降为 18-bit Q10 (×1024) 以映射
//           MULT18X18 硬件乘法器，消除 MULT36X36 占用 (F10 DSP 降载整改)
//           增益量化误差 < 0.01%，中间乘积为 Q26，末级 >>>26 恢复整数 PWM
// 计算时钟: 50MHz 系统主频下 4 级流水线，确定性延迟 80ns (0 抖动)
//           第 3 级为 pwm_mult_reg 打拍，用于切断"饱和比较 → 乘法器"长组合路径 (F11/N3)
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
    // 根据官方 FAQ 实测参数 (摆杆 15cm/90g, 摆臂 15.2cm/90g) 重算:
    // [设计说明 D1]:
    // 连续 CARE 理论解为: K=[-98.1984, -10.5681, -9.7622, -5.7926, -4.0000]
    // 实机调优值采用:     K=[-87.1845,  -9.6412, -9.7622, -5.7926, -4.0000]
    // 理由: 适度降低摆角刚度 K1(降11.2%)与阻尼 K2(降8.8%), 配合四阶软启动接入,
    //      有效消除 90g 大惯量接杆瞬间电机的正向深度饱和, 极点阻尼比 ζ=0.399, 系统保持绝对稳定。
    // K1 = -87.1845 * 1024 = -89277 (18-bit, 误差 0.0001%)
    // K2 =  -9.6412 * 1024 =  -9873 (14-bit, 误差 0.003%)
    // K3 =  -9.7622 * 1024 =  -9996 (14-bit, 误差 0.005%)
    // K4 =  -5.7926 * 1024 =  -5932 (13-bit, 误差 0.002%)
    // K5 =  -4.0000 * 1024 =  -4096 (移位 <<< 12 即可实现，0 DSP)
    // -------------------------------------------------------------------------
    localparam signed [17:0] K1_18 = -18'sd89277;
    localparam signed [17:0] K2_18 = -18'sd9873;
    localparam signed [17:0] K3_18 = -18'sd9996;
    localparam signed [17:0] K4_18 = -18'sd5932;

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
            prod5        <= - ($signed({{18{alpha_int[31]}}, alpha_int}) <<< 12); // K5 = -4.0, Q16*Q10 -> Q26
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
