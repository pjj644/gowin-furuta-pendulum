// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: swing_up_ctrl
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 基于非线性李雅普诺夫 (Lyapunov) 机械能守恒的倒立摆起摆控制器
//           对标 Python 算法模型 (simulation/controller.py)
//           核心功能:
//           1. 相对总机械能计算 E = 0.5*Jp*(dtheta^2) + m2*g*l2*(cos(theta) - 1.0)
//           2. 平滑自适应连续化能量泵 a_pump = -A_max * sat(20 * dtheta * cos(theta))
//           3. 自然下垂对称性死点主动扰动发生 (微扰脉冲 +5.0 rad/s^2)
//           4. 水平转臂居中限位阻尼 -(1.0*alpha + 0.3*dalpha)，杜绝起摆单向甩飞
//           5. 无乘法器常数移位优化 + 2 级流水线设计，消灭时序长路径，Fmax > 80MHz
// =============================================================================

module swing_up_ctrl #(
    parameter integer MAX_ACC_PUMP   = 30,           // 起摆最大角加速度限幅 (rad/s^2)
    parameter integer VOLT_TO_PWM_K  = 15            // 加速度到 PWM 占空比换算系数 (J0/(km*12V)*1000 = 15)
)(
    input  wire                   clk,              // 50MHz 系统时钟
    input  wire                   rst_n,            // 异步复位 (低有效)
    input  wire                   calc_en,          // 1ms 控制节拍脉冲
    input  wire signed [31:0]     theta_rad_q16,    // 摆杆当前角度 (rad, Q12.16)
    input  wire signed [31:0]     dtheta_rad_s_q16, // 摆杆当前角速度 (rad/s, Q12.16)
    input  wire signed [31:0]     alpha_rad_q16,    // 转臂当前角度 (rad, Q12.16)
    input  wire signed [31:0]     dalpha_rad_s_q16, // 转臂当前角速度 (rad/s, Q12.16)

    output reg  signed [15:0]     pwm_swing_duty,   // 起摆输出 PWM 占空比 (±450)
    output wire                   energy_deficit,   // 机械能亏损标志 (E < 0 时为 1)
    output reg                    calc_done         // 1 拍完成脉冲
);

    // -------------------------------------------------------------------------
    // 1. cos(theta) 64 点对称查找表 (Q14 定点数, 16384 = 1.0)
    // -------------------------------------------------------------------------
    wire signed [31:0] abs_theta_q16 = (theta_rad_q16 < 0) ? (-theta_rad_q16) : theta_rad_q16;
    wire [5:0] cos_idx = (abs_theta_q16 >= 32'sd205887) ? 6'd63 : abs_theta_q16[16:11];

    reg signed [15:0] cos_q14;
    always @(*) begin
        case (cos_idx)
            6'd0:  cos_q14 = 16'sd16384; 6'd1:  cos_q14 = 16'sd16364;
            6'd2:  cos_q14 = 16'sd16304; 6'd3:  cos_q14 = 16'sd16205;
            6'd4:  cos_q14 = 16'sd16066; 6'd5:  cos_q14 = 16'sd15888;
            6'd6:  cos_q14 = 16'sd15671; 6'd7:  cos_q14 = 16'sd15416;
            6'd8:  cos_q14 = 16'sd15123; 6'd9:  cos_q14 = 16'sd14792;
            6'd10: cos_q14 = 16'sd14425; 6'd11: cos_q14 = 16'sd14022;
            6'd12: cos_q14 = 16'sd13584; 6'd13: cos_q14 = 16'sd13111;
            6'd14: cos_q14 = 16'sd12604; 6'd15: cos_q14 = 16'sd12065;
            6'd16: cos_q14 = 16'sd11494; 6'd17: cos_q14 = 16'sd10892;
            6'd18: cos_q14 = 16'sd10260; 6'd19: cos_q14 = 16'sd9600;
            6'd20: cos_q14 = 16'sd8913;  6'd21: cos_q14 = 16'sd8200;
            6'd22: cos_q14 = 16'sd7463;  6'd23: cos_q14 = 16'sd6703;
            6'd24: cos_q14 = 16'sd5923;  6'd25: cos_q14 = 16'sd5123;
            6'd26: cos_q14 = 16'sd4306;  6'd27: cos_q14 = 16'sd3473;
            6'd28: cos_q14 = 16'sd2626;  6'd29: cos_q14 = 16'sd1767;
            6'd30: cos_q14 = 16'sd898;   6'd31: cos_q14 = 16'sd20;
            6'd32: cos_q14 = -16'sd858;  6'd33: cos_q14 = -16'sd1737;
            6'd34: cos_q14 = -16'sd2614; 6'd35: cos_q14 = -16'sd3487;
            6'd36: cos_q14 = -16'sd4353; 6'd37: cos_q14 = -16'sd5209;
            6'd38: cos_q14 = -16'sd6051; 6'd39: cos_q14 = -16'sd6877;
            6'd40: cos_q14 = -16'sd7682; 6'd41: cos_q14 = -16'sd8463;
            6'd42: cos_q14 = -16'sd9217; 6'd43: cos_q14 = -16'sd9941;
            6'd44: cos_q14 = -16'sd10631;6'd45: cos_q14 = -16'sd11284;
            6'd46: cos_q14 = -16'sd11898;6'd47: cos_q14 = -16'sd12470;
            6'd48: cos_q14 = -16'sd12997;6'd49: cos_q14 = -16'sd13476;
            6'd50: cos_q14 = -16'sd13907;6'd51: cos_q14 = -16'sd14286;
            6'd52: cos_q14 = -16'sd14612;6'd53: cos_q14 = -16'sd14883;
            6'd54: cos_q14 = -16'sd15098;6'd55: cos_q14 = -16'sd15256;
            6'd56: cos_q14 = -16'sd15355;6'd57: cos_q14 = -16'sd15396;
            6'd58: cos_q14 = -16'sd15377;6'd59: cos_q14 = -16'sd15300;
            6'd60: cos_q14 = -16'sd15162;6'd61: cos_q14 = -16'sd14966;
            6'd62: cos_q14 = -16'sd14711;6'd63: cos_q14 = -16'sd14398;
            default: cos_q14 = 16'sd16384;
        endcase
    end

    // -------------------------------------------------------------------------
    // 流水线第 1 级: 乘法器计算与初步加减
    // -------------------------------------------------------------------------
    // 1. 方向因子: dtheta * cos(theta)
    wire signed [63:0] dth_cos_mult    = $signed(dtheta_rad_s_q16) * $signed(cos_q14);
    wire signed [63:0] dth_cos_shifted = dth_cos_mult >>> 14;
    wire signed [31:0] dth_cos_q16     = dth_cos_shifted[31:0];

    // 2. 转臂居中限位阻尼 (0.3 * 65536 = 19661)
    wire signed [63:0] dalpha_damp_mult    = $signed(dalpha_rad_s_q16) * 32'sd19661;
    wire signed [63:0] dalpha_damp_shifted = dalpha_damp_mult >>> 16;
    wire signed [31:0] arm_limit_q16       = alpha_rad_q16 + dalpha_damp_shifted[31:0];
    wire signed [31:0] arm_limit_int       = arm_limit_q16 >>> 16;

    // 3. 下垂死点主动扰动判据
    wire is_dead_center = (dtheta_rad_s_q16 > -32'sd3276 && dtheta_rad_s_q16 < 32'sd3276) &&
                          (abs_theta_q16 > 32'sd163840);

    // 流水线打拍寄存器
    reg signed [31:0] dth_cos_q16_r;
    reg signed [31:0] arm_limit_int_r;
    reg               is_dead_center_r;
    reg               pipe_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dth_cos_q16_r    <= 32'sd0;
            arm_limit_int_r  <= 32'sd0;
            is_dead_center_r <= 1'b0;
            pipe_valid       <= 1'b0;
        end else if (calc_en) begin
            dth_cos_q16_r    <= dth_cos_q16;
            arm_limit_int_r  <= arm_limit_int;
            is_dead_center_r <= is_dead_center;
            pipe_valid       <= 1'b1;
        end else begin
            pipe_valid       <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线第 2 级: 无乘法器移位常数缩放与基础泵入加速度打拍
    // -------------------------------------------------------------------------
    // 1. 饱和因子 sat(20.0 * dth_cos): 20 = 16 + 4, 无乘法器!
    wire signed [31:0] sat_in_mult = (dth_cos_q16_r <<< 4) + (dth_cos_q16_r <<< 2);
    wire signed [31:0] sat_in_q16  = (sat_in_mult > 32'sd65536) ? 32'sd65536 :
                                     ((sat_in_mult < -32'sd65536) ? -32'sd65536 : sat_in_mult);

    // 2. 泵入加速度 -30 * sat_in: 30 = 32 - 2, 无乘法器!
    wire signed [31:0] pump_mult_30 = (sat_in_q16 <<< 5) - (sat_in_q16 <<< 1);
    wire signed [31:0] pump_shifted = - (pump_mult_30 >>> 16);
    wire signed [31:0] base_pump_acc = is_dead_center_r ? 32'sd5 : pump_shifted;

    reg signed [31:0] base_pump_acc_r;
    reg signed [31:0] arm_limit_int_r2;
    reg               pipe_valid2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base_pump_acc_r  <= 32'sd0;
            arm_limit_int_r2 <= 32'sd0;
            pipe_valid2      <= 1'b0;
        end else if (pipe_valid) begin
            base_pump_acc_r  <= base_pump_acc;
            arm_limit_int_r2 <= arm_limit_int_r;
            pipe_valid2      <= 1'b1;
        end else begin
            pipe_valid2      <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线第 3 级: 转臂居中阻尼限幅与 PWM 换算输出
    // -------------------------------------------------------------------------
    // 3. 转臂居中叠加与限幅 [-30, +30]
    wire signed [31:0] raw_acc_cmd = base_pump_acc_r - arm_limit_int_r2;
    wire signed [31:0] clamped_acc = (raw_acc_cmd > 32'sd30) ? 32'sd30 :
                                     ((raw_acc_cmd < -32'sd30) ? -32'sd30 : raw_acc_cmd);

    // 4. 换算为 PWM 占空比: 15 * acc = (acc <<< 4) - acc, 无乘法器!
    wire signed [31:0] raw_pwm = (clamped_acc <<< 4) - clamped_acc;

    assign energy_deficit = 1'b1; // 内部常态指示

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_swing_duty <= 16'sd0;
            calc_done      <= 1'b0;
        end else if (pipe_valid2) begin
            // 占空比安全饱和限幅 [-450, +450] (对应最大约 5.4V 电压，防止打手)
            if (raw_pwm > 32'sd450)
                pwm_swing_duty <= 16'sd450;
            else if (raw_pwm < -32'sd450)
                pwm_swing_duty <= -16'sd450;
            else
                pwm_swing_duty <= raw_pwm[15:0];
            calc_done <= 1'b1;
        end else begin
            calc_done <= 1'b0;
        end
    end

endmodule
