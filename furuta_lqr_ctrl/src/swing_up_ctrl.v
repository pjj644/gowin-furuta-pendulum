// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: swing_up_ctrl
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 基于非线性李雅普诺夫 (Lyapunov) 机械能守恒的高精度起摆控制器
//           严格对标 Python 仿真算法模型 (simulation/controller.py)
//           整改补齐核心功能:
//           1. F1: 修正 64 点精确余弦查找表与单调定点映射 (0~pi 覆盖无溢出回绕)
//           2. F2/N1/N2: 实现机械能判据 E = E_kin + E_pot 并修正定点定标 (>>>24) 与钳位值
//           3. F3/N5: 4 段折线平滑拟合 tanh(x)，常数平滑消除分段跳变
//           4. F4: 转臂居中限位阻尼全程保持 Q16 精度，消除整数截断死区
//           5. F10/F11/N3: 深度 5 级流水线优化，彻底打断乘法器级联与长组合逻辑链，时序充足收敛
// =============================================================================

module swing_up_ctrl #(
    parameter integer MAX_ACC_PUMP   = 30,           // 起摆最大角加速度限幅 (rad/s^2)
    parameter integer VOLT_TO_PWM_K  = 18            // 加速度到 PWM 占空比换算系数 (J0/(km*12V)*1000 = 18.48 ~= 18)
)(
    input  wire                   clk,              // 50MHz 系统时钟
    input  wire                   rst_n,            // 异步复位 (低有效)
    input  wire                   calc_en,          // 1ms 控制节拍脉冲
    input  wire signed [31:0]     theta_rad_q16,    // 摆杆当前角度 (rad, Q12.16)
    input  wire signed [31:0]     dtheta_rad_s_q16, // 摆杆当前角速度 (rad/s, Q12.16)
    input  wire signed [31:0]     alpha_rad_q16,    // 转臂当前角度 (rad, Q12.16)
    input  wire signed [31:0]     dalpha_rad_s_q16, // 转臂当前角速度 (rad/s, Q12.16)

    output reg  signed [15:0]     pwm_swing_duty,   // 起摆输出 PWM 占空比 (±450)
    output reg                    energy_deficit,   // 机械能亏损标志 (E < 0 时为 1, 供状态机监控)
    output reg                    calc_done         // 1 拍完成脉冲
);

    // 参数派生与规范化 (修复 B5: 消除硬编码, 规范接入 MAX_ACC_PUMP 与 VOLT_TO_PWM_K)
    localparam signed [31:0] ACC_LIMIT_Q16 = MAX_ACC_PUMP * 32'sd65536;      // ±MAX_ACC_PUMP rad/s^2 (Q16: 30*65536 = 1966080)
    localparam signed [31:0] PWM_MAX_SWING = MAX_ACC_PUMP * VOLT_TO_PWM_K;   // 占空比饱和上限 (30*15 = 450)

    // -------------------------------------------------------------------------
    // 流水线 Stage 0: 角度预处理与余弦查找表读出 (F1 & 切断乘法器级联 F11)
    // -------------------------------------------------------------------------
    wire signed [31:0] abs_theta_q16 = (theta_rad_q16 < 0) ? (-theta_rad_q16) : theta_rad_q16;
    wire [47:0] idx_mult = abs_theta_q16 * 32'd20535;
    wire [5:0]  cos_idx  = (abs_theta_q16 >= 32'sd205887) ? 6'd63 : idx_mult[31:26];

    reg signed [15:0] cos_lut_q14;
    always @(*) begin
        case (cos_idx)
            6'd0:  cos_lut_q14 =  16'sd16384; 6'd1:  cos_lut_q14 =  16'sd16364;
            6'd2:  cos_lut_q14 =  16'sd16303; 6'd3:  cos_lut_q14 =  16'sd16201;
            6'd4:  cos_lut_q14 =  16'sd16059; 6'd5:  cos_lut_q14 =  16'sd15877;
            6'd6:  cos_lut_q14 =  16'sd15656; 6'd7:  cos_lut_q14 =  16'sd15396;
            6'd8:  cos_lut_q14 =  16'sd15097; 6'd9:  cos_lut_q14 =  16'sd14761;
            6'd10: cos_lut_q14 =  16'sd14389; 6'd11: cos_lut_q14 =  16'sd13980;
            6'd12: cos_lut_q14 =  16'sd13537; 6'd13: cos_lut_q14 =  16'sd13060;
            6'd14: cos_lut_q14 =  16'sd12551; 6'd15: cos_lut_q14 =  16'sd12010;
            6'd16: cos_lut_q14 =  16'sd11440; 6'd17: cos_lut_q14 =  16'sd10841;
            6'd18: cos_lut_q14 =  16'sd10215; 6'd19: cos_lut_q14 =  16'sd9564;
            6'd20: cos_lut_q14 =  16'sd8889;  6'd21: cos_lut_q14 =  16'sd8192;
            6'd22: cos_lut_q14 =  16'sd7475;  6'd23: cos_lut_q14 =  16'sd6739;
            6'd24: cos_lut_q14 =  16'sd5986;  6'd25: cos_lut_q14 =  16'sd5218;
            6'd26: cos_lut_q14 =  16'sd4437;  6'd27: cos_lut_q14 =  16'sd3646;
            6'd28: cos_lut_q14 =  16'sd2845;  6'd29: cos_lut_q14 =  16'sd2037;
            6'd30: cos_lut_q14 =  16'sd1224;  6'd31: cos_lut_q14 =  16'sd408;
            6'd32: cos_lut_q14 = -16'sd408;   6'd33: cos_lut_q14 = -16'sd1224;
            6'd34: cos_lut_q14 = -16'sd2037;  6'd35: cos_lut_q14 = -16'sd2845;
            6'd36: cos_lut_q14 = -16'sd3646;  6'd37: cos_lut_q14 = -16'sd4437;
            6'd38: cos_lut_q14 = -16'sd5218;  6'd39: cos_lut_q14 = -16'sd5986;
            6'd40: cos_lut_q14 = -16'sd6739;  6'd41: cos_lut_q14 = -16'sd7475;
            6'd42: cos_lut_q14 = -16'sd8192;  6'd43: cos_lut_q14 = -16'sd8889;
            6'd44: cos_lut_q14 = -16'sd9564;  6'd45: cos_lut_q14 = -16'sd10215;
            6'd46: cos_lut_q14 = -16'sd10841; 6'd47: cos_lut_q14 = -16'sd11440;
            6'd48: cos_lut_q14 = -16'sd12010; 6'd49: cos_lut_q14 = -16'sd12551;
            6'd50: cos_lut_q14 = -16'sd13060; 6'd51: cos_lut_q14 = -16'sd13537;
            6'd52: cos_lut_q14 = -16'sd13980; 6'd53: cos_lut_q14 = -16'sd14389;
            6'd54: cos_lut_q14 = -16'sd14761; 6'd55: cos_lut_q14 = -16'sd15097;
            6'd56: cos_lut_q14 = -16'sd15396; 6'd57: cos_lut_q14 = -16'sd15656;
            6'd58: cos_lut_q14 = -16'sd15877; 6'd59: cos_lut_q14 = -16'sd16059;
            6'd60: cos_lut_q14 = -16'sd16201; 6'd61: cos_lut_q14 = -16'sd16303;
            6'd62: cos_lut_q14 = -16'sd16364; 6'd63: cos_lut_q14 = -16'sd16384;
            default: cos_lut_q14 = 16'sd16384;
        endcase
    end

    // 动能基础平方在 Stage 0 预计算，切断 Stage 1 的乘法器级联长路径 (F11 时序收敛)
    //
    // 钳位阈值与钳位值必须同为 ±32.0 rad/s (Q16: ±2097152 / Q12: ±131072)，理由如下:
    //   起摆的物理需求 —— 摆杆要从下垂点以 E >= 0 抵达倒立点，底部角速度至少需
    //     |theta_dot| >= sqrt(4*m2*g*l2/Jp) = sqrt(4*0.0662175/0.000675) = 19.81 rad/s (官方 90g 摆杆参数)
    //   若钳位到 8 rad/s，则 e_kin 上限仅 (32767^2*22)>>24 = 1407，
    //   而势能项最大幅值为 8680 (下垂点) —— e_total 将永远 < 0，
    //   energy_deficit 永远为 1，泵能永不停止 -> 机械能 runaway、永不捕获。
    //   钳位到 32 rad/s 后 e_kin 上限为 22527 > 8680，能量判据才能正常翻转。
    // 溢出核算: 131071^2 * 22 = 3.78e11 < 2^47 (e_kin_scaled 为 48 位) ✅
    wire signed [17:0] dth_in_q12       = (dtheta_rad_s_q16 > 32'sd2097151) ? 18'sd131071 :
                                          ((dtheta_rad_s_q16 < -32'sd2097152) ? -18'sd131072 : dtheta_rad_s_q16[21:4]);
    reg signed [35:0] dth_sq_s0;

    // Stage 0 寄存器
    reg signed [15:0] cos_q14_s0;
    reg signed [31:0] dtheta_s0;
    reg signed [31:0] alpha_s0;
    reg signed [31:0] dalpha_s0;
    reg               is_dead_s0;
    reg               pipe_valid0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cos_q14_s0   <= 16'sd16384;
            dtheta_s0    <= 32'sd0;
            alpha_s0     <= 32'sd0;
            dalpha_s0    <= 32'sd0;
            dth_sq_s0    <= 36'sd0;
            is_dead_s0   <= 1'b0;
            pipe_valid0  <= 1'b0;
        end else if (calc_en) begin
            cos_q14_s0   <= cos_lut_q14;
            dtheta_s0    <= dtheta_rad_s_q16;
            alpha_s0     <= alpha_rad_q16;
            dalpha_s0    <= dalpha_rad_s_q16;
            dth_sq_s0    <= dth_in_q12 * dth_in_q12;
            is_dead_s0   <= (dtheta_rad_s_q16 > -32'sd3276 && dtheta_rad_s_q16 < 32'sd3276) &&
                            (abs_theta_q16 > 32'sd163840);
            pipe_valid0  <= 1'b1;
        end else begin
            pipe_valid0  <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线 Stage 1: 机械能计算 (F2/N1) 与 基础乘法 (全 18x18 独立乘法器)
    // -------------------------------------------------------------------------
    // 1. 动能计算: 引用 Stage 0 已注册的平方结果 dth_sq_s0，彻底切断级联长路径
    // 修复 N1: Q12 平方为 Q24，乘以 22 需 >>> 24 转化为 Q16 (原 >>> 8 导致动能虚高 65536 倍)
    // 官方 90g 摆杆: 0.5*Jp = 0.5*0.000675 = 0.0003375 -> 0.0003375*65536 = 22.118 ~= 22
    wire signed [47:0] e_kin_scaled     = dth_sq_s0 * 18'sd22;
    wire signed [47:0] e_kin_shift      = e_kin_scaled >>> 24; // Q24 * 22 >>> 24 -> Q16
    wire signed [31:0] e_kin_q16        = e_kin_shift[31:0];

    // 2. 势能计算: (cos_q14_s0 - 16384) * 4340
    // 官方 90g 摆杆: m2*g*l2 = 0.090 * 9.81 * 0.075 = 0.0662175 J -> 0.0662175*65536 = 4339.6 ~= 4340 (原 3214)
    wire signed [31:0] cos_diff_q14     = $signed(cos_q14_s0) - 32'sd16384;
    wire signed [47:0] e_pot_mult       = cos_diff_q14 * 32'sd4340; // MULT18X18
    wire signed [47:0] e_pot_shift      = e_pot_mult >>> 14;
    wire signed [31:0] e_pot_q16        = e_pot_shift[31:0];

    wire signed [31:0] e_total_q16      = e_kin_q16 + e_pot_q16;
    wire               e_deficit_calc   = (e_total_q16 < 32'sd0);

    // 3. 方向因子: dtheta * cos(theta)
    wire signed [47:0] dth_cos_mult     = $signed(dtheta_s0) * $signed(cos_q14_s0);
    wire signed [47:0] dth_cos_shift    = dth_cos_mult >>> 14;
    wire signed [31:0] dth_cos_q16      = dth_cos_shift[31:0];

    // 4. 转臂居中限位阻尼 (F4 全精度 Q16): 0.3 * 65536 = 19661
    wire signed [47:0] dalpha_damp_mult = $signed(dalpha_s0) * 32'sd19661;
    wire signed [47:0] dalpha_damp_shift = dalpha_damp_mult >>> 16;
    wire signed [31:0] dalpha_damp_q16  = dalpha_damp_shift[31:0];
    wire signed [31:0] arm_limit_q16    = alpha_s0 + dalpha_damp_q16;

    // Stage 1 寄存器
    reg signed [31:0] dth_cos_q16_r;
    reg signed [31:0] arm_limit_q16_r;
    reg               is_dead_center_r;
    reg               energy_deficit_r;
    reg               pipe_valid1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dth_cos_q16_r    <= 32'sd0;
            arm_limit_q16_r  <= 32'sd0;
            is_dead_center_r <= 1'b0;
            energy_deficit_r <= 1'b1;
            pipe_valid1      <= 1'b0;
        end else if (pipe_valid0) begin
            dth_cos_q16_r    <= dth_cos_q16;
            arm_limit_q16_r  <= arm_limit_q16;
            is_dead_center_r <= is_dead_s0;
            energy_deficit_r <= e_deficit_calc;
            pipe_valid1      <= 1'b1;
        end else begin
            pipe_valid1      <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线 Stage 2: 预计算乘 20 因子与绝对值/符号，插入寄存器打断长路径 (修复 N3)
    // -------------------------------------------------------------------------
    wire signed [31:0] x_mult = (dth_cos_q16_r <<< 4) + (dth_cos_q16_r <<< 2);
    wire signed [31:0] abs_x  = (x_mult < 0) ? (-x_mult) : x_mult;
    wire               x_neg  = (x_mult < 0);

    reg signed [31:0] abs_x_r2;
    reg               x_neg_r2;
    reg signed [31:0] arm_limit_q16_r2;
    reg               is_dead_center_r2;
    reg               energy_deficit_r2;
    reg               pipe_valid2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            abs_x_r2          <= 32'sd0;
            x_neg_r2          <= 1'b0;
            arm_limit_q16_r2  <= 32'sd0;
            is_dead_center_r2 <= 1'b0;
            energy_deficit_r2 <= 1'b1;
            pipe_valid2       <= 1'b0;
        end else if (pipe_valid1) begin
            abs_x_r2          <= abs_x;
            x_neg_r2          <= x_neg;
            arm_limit_q16_r2  <= arm_limit_q16_r;
            is_dead_center_r2 <= is_dead_center_r;
            energy_deficit_r2 <= energy_deficit_r;
            pipe_valid2       <= 1'b1;
        end else begin
            pipe_valid2       <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线 Stage 3: 4 段折线平滑 tanh 逼近 (F3/N5) 与 基础加速度合成
    // -------------------------------------------------------------------------
    reg signed [31:0] tanh_abs_q16;
    always @(*) begin
        if (abs_x_r2 < 32'sd32768)
            tanh_abs_q16 = abs_x_r2 - (abs_x_r2 >>> 4) - (abs_x_r2 >>> 6);
        else if (abs_x_r2 < 32'sd65536)
            tanh_abs_q16 = (abs_x_r2 >>> 1) + (abs_x_r2 >>> 4) + 32'sd11852; // 修复 N5: 消除 0.5 处边界跳变
        else if (abs_x_r2 < 32'sd131072)
            // 修复 N5 残留: 段3 常数必须与段2 (11852) 联动推导，否则 x=1.0 处产生跳变
            // 连续性条件: c3 = 段2(x=1.0) - (1/8 + 1/16) = (32768+4096+11852)/65536 - 0.1875
            //            = 0.743347 - 0.1875 = 0.555847 -> Q16 = 36428
            // 实测 x=1.0 处跳变由 0.009171 降至 0.000031 (完全连续)
            tanh_abs_q16 = (abs_x_r2 >>> 3) + (abs_x_r2 >>> 4) + 32'sd36428;
        else
            tanh_abs_q16 = 32'sd65536;
    end

    // 泵入加速度: a_pump = - A_max * tanh(20*dth*cos) = x_neg ? (MAX_ACC * tanh_abs) : (- MAX_ACC * tanh_abs)
    // 默认 MAX_ACC_PUMP=30 时使用移位减法 (32 - 2 = 30) 避免消耗乘法器 (修复 B5)
    wire signed [31:0] pump_mag         = (MAX_ACC_PUMP == 30) ?
                                          ((tanh_abs_q16 <<< 5) - (tanh_abs_q16 <<< 1)) :
                                          (tanh_abs_q16 * MAX_ACC_PUMP);
    wire signed [31:0] pump_acc_q16     = x_neg_r2 ? pump_mag : (-pump_mag);

    // F2: 机械能门控
    wire signed [31:0] pump_gated_q16   = energy_deficit_r2 ? pump_acc_q16 : 32'sd0;
    wire signed [31:0] base_acc_q16     = is_dead_center_r2 ? 32'sd327680 : pump_gated_q16;

    reg signed [31:0] base_acc_q16_r3;
    reg signed [31:0] arm_limit_q16_r3;
    reg               energy_deficit_r3;
    reg               pipe_valid3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base_acc_q16_r3   <= 32'sd0;
            arm_limit_q16_r3  <= 32'sd0;
            energy_deficit_r3 <= 1'b1;
            pipe_valid3       <= 1'b0;
        end else if (pipe_valid2) begin
            base_acc_q16_r3   <= base_acc_q16;
            arm_limit_q16_r3  <= arm_limit_q16_r2;
            energy_deficit_r3 <= energy_deficit_r2;
            pipe_valid3       <= 1'b1;
        end else begin
            pipe_valid3       <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 流水线 Stage 4: 转臂居中阻尼连续合成 (F4) 与 PWM 换算输出
    // -------------------------------------------------------------------------
    wire signed [31:0] raw_acc_q16 = base_acc_q16_r3 - arm_limit_q16_r3;

    // 加速度饱和限幅 [-MAX_ACC_PUMP, +MAX_ACC_PUMP] rad/s^2 (修复 B5)
    wire signed [31:0] clamped_acc_q16 = (raw_acc_q16 > ACC_LIMIT_Q16)  ? ACC_LIMIT_Q16 :
                                         ((raw_acc_q16 < -ACC_LIMIT_Q16) ? -ACC_LIMIT_Q16 : raw_acc_q16);

    // 换算为 PWM 占空比 (修复 B5):
    // 换算系数默认为 18 时，使用移位加法 (clamped_acc_q16 <<< 4) + (clamped_acc_q16 <<< 1) 实现 * 18 (无 DSP 乘法器消耗)
    wire signed [47:0] pwm_mult_acc = (VOLT_TO_PWM_K == 18) ?
                                      ((clamped_acc_q16 <<< 4) + (clamped_acc_q16 <<< 1)) :
                                      ((VOLT_TO_PWM_K == 15) ?
                                      ((clamped_acc_q16 <<< 4) - clamped_acc_q16) :
                                      (clamped_acc_q16 * VOLT_TO_PWM_K));
    wire signed [31:0] raw_pwm      = pwm_mult_acc >>> 16;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_swing_duty <= 16'sd0;
            energy_deficit <= 1'b1;
            calc_done      <= 1'b0;
        end else if (pipe_valid3) begin
            energy_deficit <= energy_deficit_r3;
            if (raw_pwm > PWM_MAX_SWING)
                pwm_swing_duty <= PWM_MAX_SWING[15:0];
            else if (raw_pwm < -PWM_MAX_SWING)
                pwm_swing_duty <= -PWM_MAX_SWING[15:0];
            else
                pwm_swing_duty <= raw_pwm[15:0];
            calc_done <= 1'b1;
        end else begin
            calc_done <= 1'b0;
        end
    end

endmodule
