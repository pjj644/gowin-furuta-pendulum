"""
FPGA Fixed-Point Arithmetic & Verilog HDL Mapping Guide
全国大学生嵌入式芯片与系统设计竞赛 - 选题一（高云 GW2A-55 FPGA）
提供：
  1. 浮点增益到定点数 (Q12.16) 的自动转换与精度量化分析 (支持 LQR 与 LQI)
  2. 高云 FPGA 云源软件可直接综合的 Verilog HDL 核心控制模块代码生成
"""

import os
import sys
import numpy as np

# 路径自适应兼容保护
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if base_dir not in sys.path:
    sys.path.insert(0, base_dir)

try:
    from simulation.config import PendulumConfig
    from simulation.controller import DiscreteFPGAController
except ImportError:
    from config import PendulumConfig
    from controller import DiscreteFPGAController


class FPGAFixedPointGuide:
    """
    负责将 Python 仿真出的浮点控制参数转换成适合 FPGA 硬件实现的定点数
    """

    def __init__(self, controller: DiscreteFPGAController = None):
        if controller is None:
            self.ctrl = DiscreteFPGAController()
        else:
            self.ctrl = controller

        # 使用 Q12.16 定点格式 (总长 28-bit，可直接装入 32-bit signed 寄存器)
        # 1 位符号位，11 位整数位，16 位小数位
        # 缩放因子 Scale = 2^16 = 65536
        self.FRACTION_BITS = 16
        self.SCALE = 1 << self.FRACTION_BITS

    def to_q_format(self, float_val: float) -> int:
        """将浮点数转换为 Q16 定点有符号整数"""
        return int(np.round(float_val * self.SCALE))

    def to_float(self, fixed_val: int) -> float:
        """将定点整数还原为浮点数"""
        return float(fixed_val) / self.SCALE

    def analyze_quantization(self):
        """对比浮点与定点增益的量化误差"""
        K_float = self.ctrl.K_gain
        if len(K_float) == 5:
            labels = [
                "k_theta (摆角)",
                "k_dtheta (摆角速度)",
                "k_alpha (转臂位置)",
                "k_dalpha (转臂角速度)",
                "k_int (转臂积分静差消除)",
            ]
        else:
            labels = [
                "k_theta (摆角)",
                "k_dtheta (摆角速度)",
                "k_alpha (转臂位置)",
                "k_dalpha (转臂角速度)",
            ]

        print("\n" + "=" * 70)
        print("          高云 FPGA 算法定点化 (Q12.16) 参数量化分析")
        print("=" * 70)
        print(f"{'增益项':<22} | {'浮点精确保留':<12} | {'Q16 定点整数':<14} | {'定点还原值':<12} | {'绝对误差'}")
        print("-" * 70)

        q_gains = []
        for i, val in enumerate(K_float):
            q_val = self.to_q_format(val)
            q_gains.append(q_val)
            recon = self.to_float(q_val)
            err = abs(recon - val)
            print(f"{labels[i]:<20} | {val:12.4f} | {q_val:14d} | {recon:12.4f} | {err:.6e}")
        print("=" * 70)
        return q_gains

    def to_verilog_literal(self, val: int, bits: int = 32) -> str:
        """生成标准 Verilog 有符号整数字面量 (负号必须位于位宽之前，例如 -32'sd5388751)"""
        if val < 0:
            return f"-{bits}'sd{abs(val)}"
        else:
            return f"{bits}'sd{val}"

    def generate_verilog_module(self, output_path: str = "simulation/furuta_lqr_ctrl.v"):
        """
        生成高云 FPGA 云源软件兼容的 Verilog HDL 硬件模块 (支持 LQI / LQR)
        """
        K = self.ctrl.K_gain
        is_lqi = (len(K) == 5)

        k1_q = self.to_q_format(K[0])
        k2_q = self.to_q_format(K[1])
        k3_q = self.to_q_format(K[2])
        k4_q = self.to_q_format(K[3])
        k5_q = self.to_q_format(K[4]) if is_lqi else 0

        k1_lit = self.to_verilog_literal(k1_q, 32)
        k2_lit = self.to_verilog_literal(k2_q, 32)
        k3_lit = self.to_verilog_literal(k3_q, 32)
        k4_lit = self.to_verilog_literal(k4_q, 32)
        k5_lit = self.to_verilog_literal(k5_q, 32)

        # 构建 Verilog 端口与流水线
        extra_input = "    input  wire signed [31:0] alpha_int,   // 转臂积分误差 (rad*s, 消除静差)\n" if is_lqi else ""
        extra_k_param = f"    localparam signed [31:0] K5 = {k5_lit}; // {K[4]:.4f} (积分项增益)\n" if is_lqi else ""
        extra_mult_reg = ", prod5" if is_lqi else ""
        extra_mult_rst = "            prod5        <= 64'd0;\n" if is_lqi else ""
        extra_mult_calc = "            prod5        <= alpha_int * K5;\n" if is_lqi else ""
        sum_terms = "prod1 + prod2 + prod3 + prod4" + (" + prod5" if is_lqi else "")

        verilog_code = f"""// =============================================================================
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
{extra_input}
    // 控制输出
    output reg  signed [15:0] pwm_duty,    // 输出至电机驱动模块的 PWM 占空比 [-1000, +1000]
    output reg                calc_done    // 计算完成脉冲
);

    // -------------------------------------------------------------------------
    // LQR / LQI 状态反馈增益常量 (Q12.16 格式)
    // -------------------------------------------------------------------------
    localparam signed [31:0] K1 = {k1_lit}; // {K[0]:.4f} (摆角增益)
    localparam signed [31:0] K2 = {k2_lit}; // {K[1]:.4f} (摆角速度增益)
    localparam signed [31:0] K3 = {k3_lit}; // {K[2]:.4f} (转臂位置增益)
    localparam signed [31:0] K4 = {k4_lit}; // {K[3]:.4f} (转臂速度增益)
{extra_k_param}
    // 电压到 PWM 占空比缩放因子 (12V 对应 1000 计数值: 1000 / 12 ~= 83.33)
    // 在 Q16 格式下: 83.33 * 65536 ~= 5461163
    localparam signed [31:0] VOLT_TO_PWM = 32'sd5461163;
    localparam signed [15:0] PWM_MAX     = 16'sd1000;
    localparam signed [15:0] PWM_MIN     = -16'sd1000;

    // -------------------------------------------------------------------------
    // 流水线第 1 级: 多路并行乘法 (调用高云 GW2A DSP28x28 或 MULT18x18 硬核)
    // -------------------------------------------------------------------------
    reg signed [63:0] prod1, prod2, prod3, prod4{extra_mult_reg};
    reg               stage1_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod1        <= 64'd0;
            prod2        <= 64'd0;
            prod3        <= 64'd0;
            prod4        <= 64'd0;
{extra_mult_rst}            stage1_valid <= 1'b0;
        end else if (calc_en) begin
            prod1        <= theta_err * K1;
            prod2        <= dtheta    * K2;
            prod3        <= alpha_err * K3;
            prod4        <= dalpha    * K4;
{extra_mult_calc}            stage1_valid <= 1'b1;
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

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_cmd_q16    <= 32'd0;
            stage2_valid <= 1'b0;
        end else if (stage1_valid) begin
            v_cmd_q16    <= - ( ({sum_terms}) >>> 16 );
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
"""
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(verilog_code)
        print(f"[FPGA Guide] 成功生成高云 FPGA 核心 Verilog 模块: {output_path}")


if __name__ == "__main__":
    guide = FPGAFixedPointGuide()
    guide.analyze_quantization()
    guide.generate_verilog_module("simulation/furuta_lqr_ctrl.v")
