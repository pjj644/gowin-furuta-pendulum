// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: encoder_quad_reader
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 电机正交光电/磁编码器 (A/B/Z 相) 4 倍频采集与数字滤波测速测角模块
//           1. 硬件级双重防亚稳态同步 + 多级数字低通消抖滤波 (滤除电机火花电磁干扰)
//           2. 严格四倍频 (4x Quadrature Decoding) 硬件状态判向鉴相
//           3. 32 位有符号绝对脉冲位置计数 (支持掉电/按键归零与 Z 相零位复位)
//           4. 1ms 定时硬件差分测速 + 一阶数字 IIR 低通平滑滤波 (完全抑制离散量化阶跃噪声)
//           5. 直接输出 Q12.16 定点数转臂角位移 alpha (rad) 与角速度 dalpha (rad/s)
// =============================================================================

module encoder_quad_reader #(
    parameter integer CLK_FREQ_HZ   = 50_000_000, // 主系统时钟频率 (50MHz)
    parameter integer FILTER_CYCLES = 8,          // 数字滤波防抖采样阈值 (8*20ns = 160ns)
    parameter integer CPR           = 4000,       // 一整圈脉冲数 (1000线编码器 x 4倍频 = 4000 CPR)
    parameter integer USE_Z_INDEX   = 0,          // 是否启用 Z 相零位复位功能 (D4 整改)
    parameter integer REVERSE_DIR   = 0           // 方向反转配置 (0: 正向, 1: 软反转)
)(
    input  wire                   clk,            // 系统主时钟 (50MHz)
    input  wire                   rst_n,          // 异步低电平复位
    input  wire                   enc_a_raw,      // 编码器 A 相原始输入引脚
    input  wire                   enc_b_raw,      // 编码器 B 相原始输入引脚
    input  wire                   enc_z_raw,      // 编码器 Z 相原始输入引脚 (圈零位脉冲, 可选)
    input  wire                   clear_pos,      // 软件位置清零脉冲 (按键或上电自动归零)
    input  wire                   calc_en,        // 1ms 控制节拍脉冲 (1000Hz, 用于计算角速度)

    // 状态输出接口
    output reg  signed [31:0]     pulse_count,    // 32 位有符号绝对位置脉冲计数值
    output reg  signed [31:0]     alpha_rad_q16,  // 转臂当前角位移 (rad, Q12.16 定点数)
    output reg  signed [31:0]     dalpha_rad_s_q16,// 转臂滤波后角速度 (rad/s, Q12.16 定点数)
    output reg  signed [31:0]     speed_pps,      // 当前瞬时脉冲速率 (PPS, 扩展为 32 位防止溢出截断 D5)
    output reg                    dir_flag        // 当前旋转方向 (0: 递增/顺时针, 1: 递减/逆时针)
);

    // -------------------------------------------------------------------------
    // 1. 硬件双级触发器同步 (跨时钟域防亚稳态处理)
    // -------------------------------------------------------------------------
    reg [1:0] a_sync_reg;
    reg [1:0] b_sync_reg;
    reg [1:0] z_sync_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync_reg <= 2'b00;
            b_sync_reg <= 2'b00;
            z_sync_reg <= 2'b00;
        end else begin
            a_sync_reg <= {a_sync_reg[0], enc_a_raw};
            b_sync_reg <= {b_sync_reg[0], enc_b_raw};
            z_sync_reg <= {z_sync_reg[0], enc_z_raw};
        end
    end

    // -------------------------------------------------------------------------
    // 2. 多级数字积分消抖低通滤波器 (去除电机电刷微火花及线缆感应毛刺)
    // -------------------------------------------------------------------------
    reg [7:0] a_filter_cnt;
    reg [7:0] b_filter_cnt;
    reg       a_filt;
    reg       b_filt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_filter_cnt <= 8'd0;
            b_filter_cnt <= 8'd0;
            a_filt       <= 1'b0;
            b_filt       <= 1'b0;
        end else begin
            // A 相滤波: 连续采样 FILTER_CYCLES 个时钟高电平才翻转为 1，反之翻转为 0
            if (a_sync_reg[1] == a_filt) begin
                a_filter_cnt <= 8'd0;
            end else if (a_filter_cnt >= FILTER_CYCLES - 1) begin
                a_filt       <= a_sync_reg[1];
                a_filter_cnt <= 8'd0;
            end else begin
                a_filter_cnt <= a_filter_cnt + 1'b1;
            end

            // B 相滤波
            if (b_sync_reg[1] == b_filt) begin
                b_filter_cnt <= 8'd0;
            end else if (b_filter_cnt >= FILTER_CYCLES - 1) begin
                b_filt       <= b_sync_reg[1];
                b_filter_cnt <= 8'd0;
            end else begin
                b_filter_cnt <= b_filter_cnt + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. 正交四倍频鉴相与方向解算 (AB 正交双边沿检测)
    // -------------------------------------------------------------------------
    reg a_filt_dly, b_filt_dly;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_filt_dly <= 1'b0;
            b_filt_dly <= 1'b0;
        end else begin
            a_filt_dly <= a_filt;
            b_filt_dly <= b_filt;
        end
    end

    // 边沿触发检测
    wire a_rise = (a_filt == 1'b1) && (a_filt_dly == 1'b0);
    wire a_fall = (a_filt == 1'b0) && (a_filt_dly == 1'b1);
    wire b_rise = (b_filt == 1'b1) && (b_filt_dly == 1'b0);
    wire b_fall = (b_filt == 1'b0) && (b_filt_dly == 1'b1);

    // 标准正交鉴相逻辑:
    // A 上升沿时若 B 为低，或 A 下降沿时若 B 为高，或 B 上升沿时若 A 为高，或 B 下降沿时若 A 为低 -> 正转(+1)
    // 其余有效跳变为反转(-1)
    reg signed [1:0] count_step;

    always @(*) begin
        case ({a_filt_dly, b_filt_dly, a_filt, b_filt})
            // 正转步进 (+1): A 相超前 B 相 (CW)
            4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: count_step =  2'sd1;
            // 反转步进 (-1): B 相超前 A 相 (CCW)
            4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: count_step = -2'sd1;
            // 未跳变或非法跳变 (如同时翻转)
            default: count_step = 2'sd0;
        endcase
    end

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // 4. 绝对脉冲累加器 (含 Z 相零位复位支持 D4)
    // -------------------------------------------------------------------------
    reg z_sync_dly;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) z_sync_dly <= 1'b0;
        else z_sync_dly <= z_sync_reg[1];
    end
    wire z_rise = (z_sync_reg[1] == 1'b1) && (z_sync_dly == 1'b0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_count <= 32'sd0;
            dir_flag    <= 1'b0;
        end else if (clear_pos || (USE_Z_INDEX && z_rise)) begin
            pulse_count <= 32'sd0;
        end else begin
            if (REVERSE_DIR == 0) begin
                pulse_count <= pulse_count + count_step;
                if (count_step > 0) dir_flag <= 1'b0;
                else if (count_step < 0) dir_flag <= 1'b1;
            end else begin
                pulse_count <= pulse_count - count_step;
                if (count_step > 0) dir_flag <= 1'b1;
                else if (count_step < 0) dir_flag <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 5. 1ms 周期差分测速与角速度/角位移 Q12.16 定点数转换 (完全消除除法器 E4)
    // -------------------------------------------------------------------------
    // 换算常数解释:
    // 4000 CPR 对应 2*pi rad
    // 定点数 Q16 放大 65536:
    // 角度换算: alpha_q16 = pulse_count * (2 * pi * 65536 / 4000)
    // 2 * pi * 65536 / 4000 = 102.943700948
    // 放大至 Q16: 102.943700948 * 65536 = 6746406
    // 则: alpha_q16 = (pulse_count * 6746406) >>> 16 (完全消除 64 位除法器 E4)
    //
    // 角速度换算 (在 1ms = 0.001s 控制周期内):
    // dalpha = delta_pulses / 0.001 * (2 * pi / 4000) = delta_pulses * (pi / 2)
    // 定点数 Q16: (pi / 2) * 65536 = 1.5707963 * 65536 = 102944
    //
    // 一阶 IIR 滤波参数: dalpha = dalpha + 0.35 * (raw - dalpha)
    // 0.35 * 65536 = 22938 (消除 32 位除法器 / 100)
    // 优化乘法位宽至 18x18 (F10): pulse_count 在软限位 ±2 圈下仅 ±8000 脉冲
    // 18 位有符号数范围 [-131072, +131071] 对应 ±32.7 圈，余量充足
    // 2*pi*65536/4000 = 102.9437009; 102.9437009 * 1024 = 105414 (18位有符号数)
    // alpha_q16 = (pulse_cnt_18 * 105414) >>> 10, 误差仅 0.00033%
    localparam signed [17:0] K_ANGLE_18        = 18'sd105414;
    localparam signed [17:0] K_VEL_18          = 18'sd102944; // (pi/2)*65536
    localparam signed [17:0] FILTER_ALPHA_18   = 18'sd22938;  // 0.35*65536

    reg signed [31:0] pulse_count_prev;
    reg signed [31:0] delta_pulse;
    reg signed [31:0] raw_dalpha_q16;

    wire signed [17:0] pulse_cnt_18 = (pulse_count > 32'sd131071) ? 18'sd131071 :
                                      ((pulse_count < -32'sd131072) ? -18'sd131072 : pulse_count[17:0]);
    wire signed [17:0] delta_cnt_18 = (delta_pulse > 32'sd131071) ? 18'sd131071 :
                                      ((delta_pulse < -32'sd131072) ? -18'sd131072 : delta_pulse[17:0]);

    // 拍 1: 角度与角速度基础乘法 (18x18 独立单核)
    wire signed [35:0] pulse_mult_36    = pulse_cnt_18 * K_ANGLE_18;
    wire signed [35:0] delta_mult_36    = delta_cnt_18 * K_VEL_18;
    wire signed [31:0] alpha_shifted    = pulse_mult_36 >>> 10;

    // 拍 2: 基于已打拍的 raw_dalpha_q16 计算差值并送乘法器 (拆分长组合路径)
    wire signed [31:0] diff_for_iir     = raw_dalpha_q16 - dalpha_rad_s_q16;
    wire signed [17:0] diff_iir_18      = (diff_for_iir > 32'sd131071) ? 18'sd131071 :
                                          ((diff_for_iir < -32'sd131072) ? -18'sd131072 : diff_for_iir[17:0]);
    wire signed [35:0] iir_diff_mult    = diff_iir_18 * FILTER_ALPHA_18;
    reg  signed [35:0] iir_diff_mult_r;
    wire signed [31:0] iir_diff_shifted = iir_diff_mult_r >>> 16;
    reg  [1:0]         filter_step;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_count_prev <= 32'sd0;
            delta_pulse      <= 32'sd0;
            raw_dalpha_q16   <= 32'sd0;
            alpha_rad_q16    <= 32'sd0;
            dalpha_rad_s_q16 <= 32'sd0;
            speed_pps        <= 32'sd0;
            iir_diff_mult_r  <= 36'sd0;
            filter_step      <= 2'd0;
        end else if (calc_en) begin
            // 拍 1: 更新角度与原始角速度寄存器
            delta_pulse      <= pulse_count - pulse_count_prev;
            pulse_count_prev <= pulse_count;
            speed_pps        <= delta_pulse * 32'sd1000;
            alpha_rad_q16    <= alpha_shifted[31:0];
            raw_dalpha_q16   <= delta_mult_36[31:0];
            filter_step      <= 2'd1;
        end else if (filter_step == 2'd1) begin
            // 拍 2: 寄存 IIR 乘法器输出，彻底切断 DSP 与累加器级联
            iir_diff_mult_r  <= iir_diff_mult;
            filter_step      <= 2'd2;
        end else if (filter_step == 2'd2) begin
            // 拍 3: 更新一阶数字滤波角速度
            dalpha_rad_s_q16 <= dalpha_rad_s_q16 + iir_diff_shifted[31:0];
            filter_step      <= 2'd0;
        end
    end

endmodule
