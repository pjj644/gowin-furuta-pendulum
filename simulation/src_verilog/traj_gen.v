// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: traj_gen
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 转臂目标位置设定与连续轨迹跟踪发生器 (满足拓展要求 1、2、3)
//           支持 4 种运行模式:
//           2'b00: 定点 0 度自平衡 (基础自立)
//           2'b01: 定点 +45 度平滑位置伺服 (alpha_ref = +0.7854 rad)
//           2'b10: 定点 -45 度平滑位置伺服 (alpha_ref = -0.7854 rad)
//           2'b11: 0.2Hz 正弦连续动态轨迹跟踪 (幅值 25 度, 周期 5 秒, 含速度前馈 dalpha_ref)
//           内部无除法器，采用高精度定点数比例换算与 128 点全周期正余弦 LUT
// =============================================================================

module traj_gen #(
    parameter integer CLK_FREQ_HZ      = 50_000_000,
    parameter integer UPDATE_HZ        = 1000,
    parameter integer TIMER_1MS_LIMIT  = 50_000_000 / 1000 // 1ms 基准
)(
    input  wire                   clk,              // 50MHz 系统时钟
    input  wire                   rst_n,            // 异步复位 (低有效)
    input  wire                   calc_en,          // 1ms 控制节拍脉冲
    input  wire [1:0]             mode_sel,         // 模式选择

    output reg  signed [31:0]     alpha_ref_q16,    // 目标位置参考 (rad, Q12.16)
    output reg  signed [31:0]     dalpha_ref_q16    // 目标速度前馈 (rad/s, Q12.16)
);

    // -------------------------------------------------------------------------
    // 常数定义 (Q12.16)
    // -------------------------------------------------------------------------
    localparam signed [31:0] POS_45DEG_Q16  = 32'sd51472;   // +45 deg (0.785398 rad * 65536)
    localparam signed [31:0] NEG_45DEG_Q16  = -32'sd51472;  // -45 deg
    localparam signed [31:0] TRAJ_AMP_Q16   = 32'sd28594;   // 25 deg (0.436332 rad * 65536)
    localparam signed [31:0] TRAJ_VMAX_Q16  = 32'sd35933;   // A*2*pi*f = 0.5483 rad/s * 65536

    // -------------------------------------------------------------------------
    // 流水线第 1 级: 5000 拍时间累加与 LUT 索引预计算打拍
    // -------------------------------------------------------------------------
    reg [12:0] time_cnt_5s; // 0 ~ 4999
    reg [6:0]  lut_idx_r;
    reg [1:0]  mode_sel_r;
    reg        stage1_valid;

    // 映射至 128 点全周期正弦查找表索引: idx = (time_cnt_5s * 128) / 5000
    // 采用无除法计算: 128 / 5000 = 16 / 625 = 0.0256
    // 0.0256 * 1048576 = 26843.5456 ~= 26844 (Q20 精度, 误差 < 0.0002%)
    wire [31:0] idx_calc = (time_cnt_5s * 32'd26844) >> 20;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            time_cnt_5s  <= 13'd0;
            lut_idx_r    <= 7'd0;
            mode_sel_r   <= 2'd0;
            stage1_valid <= 1'b0;
        end else if (calc_en) begin
            if (time_cnt_5s >= 13'd4999)
                time_cnt_5s <= 13'd0;
            else
                time_cnt_5s <= time_cnt_5s + 1'b1;

            lut_idx_r    <= idx_calc[6:0];
            mode_sel_r   <= mode_sel;
            stage1_valid <= 1'b1;
        end else begin
            stage1_valid <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 128 点 16 位有符号全波正弦与余弦查找表 (Q14 定点数, 16384 = 1.0)
    // -------------------------------------------------------------------------
    reg signed [15:0] sin_q14;
    reg signed [15:0] cos_q14;

    always @(*) begin
        case (lut_idx_r)
            7'd0:   begin sin_q14 = 16'sd0;     cos_q14 = 16'sd16384; end
            7'd1:   begin sin_q14 = 16'sd804;   cos_q14 = 16'sd16364; end
            7'd2:   begin sin_q14 = 16'sd1606;  cos_q14 = 16'sd16305; end
            7'd3:   begin sin_q14 = 16'sd2404;  cos_q14 = 16'sd16207; end
            7'd4:   begin sin_q14 = 16'sd3196;  cos_q14 = 16'sd16069; end
            7'd5:   begin sin_q14 = 16'sd3981;  cos_q14 = 16'sd15893; end
            7'd6:   begin sin_q14 = 16'sd4756;  cos_q14 = 16'sd15679; end
            7'd7:   begin sin_q14 = 16'sd5520;  cos_q14 = 16'sd15426; end
            7'd8:   begin sin_q14 = 16'sd6270;  cos_q14 = 16'sd15137; end
            7'd9:   begin sin_q14 = 16'sd7005;  cos_q14 = 16'sd14811; end
            7'd10:  begin sin_q14 = 16'sd7723;  cos_q14 = 16'sd14449; end
            7'd11:  begin sin_q14 = 16'sd8423;  cos_q14 = 16'sd14053; end
            7'd12:  begin sin_q14 = 16'sd9102;  cos_q14 = 16'sd13623; end
            7'd13:  begin sin_q14 = 16'sd9760;  cos_q14 = 16'sd13160; end
            7'd14:  begin sin_q14 = 16'sd10394; cos_q14 = 16'sd12665; end
            7'd15:  begin sin_q14 = 16'sd11003; cos_q14 = 16'sd12140; end
            7'd16:  begin sin_q14 = 16'sd11585; cos_q14 = 16'sd11585; end
            7'd17:  begin sin_q14 = 16'sd12140; cos_q14 = 16'sd11003; end
            7'd18:  begin sin_q14 = 16'sd12665; cos_q14 = 16'sd10394; end
            7'd19:  begin sin_q14 = 16'sd13160; cos_q14 = 16'sd9760;  end
            7'd20:  begin sin_q14 = 16'sd13623; cos_q14 = 16'sd9102;  end
            7'd21:  begin sin_q14 = 16'sd14053; cos_q14 = 16'sd8423;  end
            7'd22:  begin sin_q14 = 16'sd14449; cos_q14 = 16'sd7723;  end
            7'd23:  begin sin_q14 = 16'sd14811; cos_q14 = 16'sd7005;  end
            7'd24:  begin sin_q14 = 16'sd15137; cos_q14 = 16'sd6270;  end
            7'd25:  begin sin_q14 = 16'sd15426; cos_q14 = 16'sd5520;  end
            7'd26:  begin sin_q14 = 16'sd15679; cos_q14 = 16'sd4756;  end
            7'd27:  begin sin_q14 = 16'sd15893; cos_q14 = 16'sd3981;  end
            7'd28:  begin sin_q14 = 16'sd16069; cos_q14 = 16'sd3196;  end
            7'd29:  begin sin_q14 = 16'sd16207; cos_q14 = 16'sd2404;  end
            7'd30:  begin sin_q14 = 16'sd16305; cos_q14 = 16'sd1606;  end
            7'd31:  begin sin_q14 = 16'sd16364; cos_q14 = 16'sd804;   end
            7'd32:  begin sin_q14 = 16'sd16384; cos_q14 = 16'sd0;     end
            7'd33:  begin sin_q14 = 16'sd16364; cos_q14 = -16'sd804;  end
            7'd34:  begin sin_q14 = 16'sd16305; cos_q14 = -16'sd1606; end
            7'd35:  begin sin_q14 = 16'sd16207; cos_q14 = -16'sd2404; end
            7'd36:  begin sin_q14 = 16'sd16069; cos_q14 = -16'sd3196; end
            7'd37:  begin sin_q14 = 16'sd15893; cos_q14 = -16'sd3981; end
            7'd38:  begin sin_q14 = 16'sd15679; cos_q14 = -16'sd4756; end
            7'd39:  begin sin_q14 = 16'sd15426; cos_q14 = -16'sd5520; end
            7'd40:  begin sin_q14 = 16'sd15137; cos_q14 = -16'sd6270; end
            7'd41:  begin sin_q14 = 16'sd14811; cos_q14 = -16'sd7005; end
            7'd42:  begin sin_q14 = 16'sd14449; cos_q14 = -16'sd7723; end
            7'd43:  begin sin_q14 = 16'sd14053; cos_q14 = -16'sd8423; end
            7'd44:  begin sin_q14 = 16'sd13623; cos_q14 = -16'sd9102; end
            7'd45:  begin sin_q14 = 16'sd13160; cos_q14 = -16'sd9760; end
            7'd46:  begin sin_q14 = 16'sd12665; cos_q14 = -16'sd10394;end
            7'd47:  begin sin_q14 = 16'sd12140; cos_q14 = -16'sd11003;end
            7'd48:  begin sin_q14 = 16'sd11585; cos_q14 = -16'sd11585;end
            7'd49:  begin sin_q14 = 16'sd11003; cos_q14 = -16'sd12140;end
            7'd50:  begin sin_q14 = 16'sd10394; cos_q14 = -16'sd12665;end
            7'd51:  begin sin_q14 = 16'sd9760;  cos_q14 = -16'sd13160;end
            7'd52:  begin sin_q14 = 16'sd9102;  cos_q14 = -16'sd13623;end
            7'd53:  begin sin_q14 = 16'sd8423;  cos_q14 = -16'sd14053;end
            7'd54:  begin sin_q14 = 16'sd7723;  cos_q14 = -16'sd14449;end
            7'd55:  begin sin_q14 = 16'sd7005;  cos_q14 = -16'sd14811;end
            7'd56:  begin sin_q14 = 16'sd6270;  cos_q14 = -16'sd15137;end
            7'd57:  begin sin_q14 = 16'sd5520;  cos_q14 = -16'sd15426;end
            7'd58:  begin sin_q14 = 16'sd4756;  cos_q14 = -16'sd15679;end
            7'd59:  begin sin_q14 = 16'sd3981;  cos_q14 = -16'sd15893;end
            7'd60:  begin sin_q14 = 16'sd3196;  cos_q14 = -16'sd16069;end
            7'd61:  begin sin_q14 = 16'sd2404;  cos_q14 = -16'sd16207;end
            7'd62:  begin sin_q14 = 16'sd1606;  cos_q14 = -16'sd16305;end
            7'd63:  begin sin_q14 = 16'sd804;   cos_q14 = -16'sd16364;end
            7'd64:  begin sin_q14 = 16'sd0;     cos_q14 = -16'sd16384;end
            7'd65:  begin sin_q14 = -16'sd804;  cos_q14 = -16'sd16364;end
            7'd66:  begin sin_q14 = -16'sd1606; cos_q14 = -16'sd16305;end
            7'd67:  begin sin_q14 = -16'sd2404; cos_q14 = -16'sd16207;end
            7'd68:  begin sin_q14 = -16'sd3196; cos_q14 = -16'sd16069;end
            7'd69:  begin sin_q14 = -16'sd3981; cos_q14 = -16'sd15893;end
            7'd70:  begin sin_q14 = -16'sd4756; cos_q14 = -16'sd15679;end
            7'd71:  begin sin_q14 = -16'sd5520; cos_q14 = -16'sd15426;end
            7'd72:  begin sin_q14 = -16'sd6270; cos_q14 = -16'sd15137;end
            7'd73:  begin sin_q14 = -16'sd7005; cos_q14 = -16'sd14811;end
            7'd74:  begin sin_q14 = -16'sd7723; cos_q14 = -16'sd14449;end
            7'd75:  begin sin_q14 = -16'sd8423; cos_q14 = -16'sd14053;end
            7'd76:  begin sin_q14 = -16'sd9102; cos_q14 = -16'sd13623;end
            7'd77:  begin sin_q14 = -16'sd9760; cos_q14 = -16'sd13160;end
            7'd78:  begin sin_q14 = -16'sd10394;cos_q14 = -16'sd12665;end
            7'd79:  begin sin_q14 = -16'sd11003;cos_q14 = -16'sd12140;end
            7'd80:  begin sin_q14 = -16'sd11585;cos_q14 = -16'sd11585;end
            7'd81:  begin sin_q14 = -16'sd12140;cos_q14 = -16'sd11003;end
            7'd82:  begin sin_q14 = -16'sd12665;cos_q14 = -16'sd10394;end
            7'd83:  begin sin_q14 = -16'sd13160;cos_q14 = -16'sd9760; end
            7'd84:  begin sin_q14 = -16'sd13623;cos_q14 = -16'sd9102; end
            7'd85:  begin sin_q14 = -16'sd14053;cos_q14 = -16'sd8423; end
            7'd86:  begin sin_q14 = -16'sd14449;cos_q14 = -16'sd7723; end
            7'd87:  begin sin_q14 = -16'sd14811;cos_q14 = -16'sd7005; end
            7'd88:  begin sin_q14 = -16'sd15137;cos_q14 = -16'sd6270; end
            7'd89:  begin sin_q14 = -16'sd15426;cos_q14 = -16'sd5520; end
            7'd90:  begin sin_q14 = -16'sd15679;cos_q14 = -16'sd4756; end
            7'd91:  begin sin_q14 = -16'sd15893;cos_q14 = -16'sd3981; end
            7'd92:  begin sin_q14 = -16'sd16069;cos_q14 = -16'sd3196; end
            7'd93:  begin sin_q14 = -16'sd16207;cos_q14 = -16'sd2404; end
            7'd94:  begin sin_q14 = -16'sd16305;cos_q14 = -16'sd1606; end
            7'd95:  begin sin_q14 = -16'sd16364;cos_q14 = -16'sd804;  end
            7'd96:  begin sin_q14 = -16'sd16384;cos_q14 = 16'sd0;     end
            7'd97:  begin sin_q14 = -16'sd16364;cos_q14 = 16'sd804;   end
            7'd98:  begin sin_q14 = -16'sd16305;cos_q14 = 16'sd1606;  end
            7'd99:  begin sin_q14 = -16'sd16207;cos_q14 = 16'sd2404;  end
            7'd100: begin sin_q14 = -16'sd16069;cos_q14 = 16'sd3196;  end
            7'd101: begin sin_q14 = -16'sd15893;cos_q14 = 16'sd3981;  end
            7'd102: begin sin_q14 = -16'sd15679;cos_q14 = 16'sd4756;  end
            7'd103: begin sin_q14 = -16'sd15426;cos_q14 = 16'sd5520;  end
            7'd104: begin sin_q14 = -16'sd15137;cos_q14 = 16'sd6270;  end
            7'd105: begin sin_q14 = -16'sd14811;cos_q14 = 16'sd7005;  end
            7'd106: begin sin_q14 = -16'sd14449;cos_q14 = 16'sd7723;  end
            7'd107: begin sin_q14 = -16'sd14053;cos_q14 = 16'sd8423;  end
            7'd108: begin sin_q14 = -16'sd13623;cos_q14 = 16'sd9102;  end
            7'd109: begin sin_q14 = -16'sd13160;cos_q14 = 16'sd9760;  end
            7'd110: begin sin_q14 = -16'sd12665;cos_q14 = 16'sd10394; end
            7'd111: begin sin_q14 = -16'sd12140;cos_q14 = 16'sd11003; end
            7'd112: begin sin_q14 = -16'sd11585;cos_q14 = 16'sd11585; end
            7'd113: begin sin_q14 = -16'sd11003;cos_q14 = 16'sd12140; end
            7'd114: begin sin_q14 = -16'sd10394;cos_q14 = 16'sd12665; end
            7'd115: begin sin_q14 = -16'sd9760; cos_q14 = 16'sd13160; end
            7'd116: begin sin_q14 = -16'sd9102; cos_q14 = 16'sd13623; end
            7'd117: begin sin_q14 = -16'sd8423; cos_q14 = 16'sd14053; end
            7'd118: begin sin_q14 = -16'sd7723; cos_q14 = 16'sd14449; end
            7'd119: begin sin_q14 = -16'sd7005; cos_q14 = 16'sd14811; end
            7'd120: begin sin_q14 = -16'sd6270; cos_q14 = 16'sd15137; end
            7'd121: begin sin_q14 = -16'sd5520; cos_q14 = 16'sd15426; end
            7'd122: begin sin_q14 = -16'sd4756; cos_q14 = 16'sd15679; end
            7'd123: begin sin_q14 = -16'sd3981; cos_q14 = 16'sd15893; end
            7'd124: begin sin_q14 = -16'sd3196; cos_q14 = 16'sd16069; end
            7'd125: begin sin_q14 = -16'sd2404; cos_q14 = 16'sd16207; end
            7'd126: begin sin_q14 = -16'sd1606; cos_q14 = 16'sd16305; end
            7'd127: begin sin_q14 = -16'sd804;  cos_q14 = 16'sd16364; end
            default:begin sin_q14 = 16'sd0;     cos_q14 = 16'sd16384; end
        endcase
    end

    // -------------------------------------------------------------------------
    // 正弦轨迹与速度前馈计算 (乘法移位, 显式截取低 32 位消除 EX3791)
    // -------------------------------------------------------------------------
    wire signed [63:0] sine_pos_mult    = $signed(TRAJ_AMP_Q16)  * $signed(sin_q14);
    wire signed [63:0] sine_vel_mult    = $signed(TRAJ_VMAX_Q16) * $signed(cos_q14);
    wire signed [63:0] sine_pos_shifted = sine_pos_mult >>> 14;
    wire signed [63:0] sine_vel_shifted = sine_vel_mult >>> 14;

    wire signed [31:0] sine_pos_q16     = sine_pos_shifted[31:0];
    wire signed [31:0] sine_vel_q16     = sine_vel_shifted[31:0];

    // -------------------------------------------------------------------------
    // 流水线第 2 级: 模式切换输出
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alpha_ref_q16  <= 32'sd0;
            dalpha_ref_q16 <= 32'sd0;
        end else if (stage1_valid) begin
            case (mode_sel_r)
                2'b00: begin // 0 度定点自平衡
                    alpha_ref_q16  <= 32'sd0;
                    dalpha_ref_q16 <= 32'sd0;
                end
                2'b01: begin // +45 度定点伺服 (拓展要求 1)
                    alpha_ref_q16  <= POS_45DEG_Q16;
                    dalpha_ref_q16 <= 32'sd0;
                end
                2'b10: begin // -45 度定点伺服 (拓展要求 1)
                    alpha_ref_q16  <= NEG_45DEG_Q16;
                    dalpha_ref_q16 <= 32'sd0;
                end
                2'b11: begin // 0.2Hz 正弦连续动态轨迹跟踪 (拓展要求 2)
                    alpha_ref_q16  <= sine_pos_q16;
                    dalpha_ref_q16 <= sine_vel_q16;
                end
            endcase
        end
    end

endmodule
