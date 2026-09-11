// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: angle_sensor_reader
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 倒立摆摆杆角度传感器 (12 位 SPI ADC / 磁编码器 / 电位器) 采集与解算模块
//           1. 内嵌高精度硬件 SPI 主机状态机 (驱动 CS_N, SCLK, 接收 MISO 串行数据)
//           2. 支持外部直接输入原始数据旁路模式 (兼容板载并行 ADC / 仿真注入)
//           3. 绝对垂直平衡零点校准寄存器 (零点偏置 offset 扣减)
//           4. 360° 全圆周角度越界解卷绕逻辑 (消除 0° 与 360° 边界处的突跳发散)
//           5. 硬件尺度定点化: 将 12 位采样刻度精准线性映射至 Q12.16 定点弧度 (rad)
//           6. 1ms 定时差分测速 + 一阶数字 IIR 低通滤波器输出超平滑摆杆角速度 dtheta (rad/s)
// =============================================================================

module angle_sensor_reader #(
    parameter integer CLK_FREQ_HZ   = 50_000_000, // 主系统时钟 (50MHz)
    parameter integer SPI_SCLK_HZ   = 2_500_000,  // SPI 时钟频率 (2.5MHz, 周期 400ns)
    parameter integer ADC_RESOLUTION= 12,         // 采样位数 (默认 12 位: 0~4095)
    parameter integer INVERT_DIR    = 0           // 角度极性反转控制 (0: 正常, 1: 软取反)
)(
    input  wire                   clk,            // 系统时钟 (50MHz)
    input  wire                   rst_n,          // 异步低电平复位
    input  wire                   calc_en,        // 1ms 控制主节拍脉冲 (启动一次完整转换)

    // 硬件 SPI 接口引脚 (连接套件上的 12 位 ADC 芯片或磁编码器)
    output reg                    spi_cs_n,       // SPI 片选信号 (低电平有效)
    output reg                    spi_sclk,       // SPI 同步时钟 (Mode 0: 空闲为低)
    input  wire                   spi_miso,       // SPI 从机串行数据输入

    // 零位与校准接口
    input  wire [11:0]            zero_offset_raw,// 摆杆绝对垂直朝上时的 ADC 原始计数值 (0~4095)

    // 外部并行原始数据旁路接口 (测试/板载直接 ADC 模式)
    input  wire                   ext_raw_valid,  // 外部直接有效标志
    input  wire [11:0]            ext_raw_data,   // 外部 12 位原始 ADC 码值

    // 状态解算输出接口 (与 LQR 姿态控制算法直接互联)
    output reg  [11:0]            raw_adc_data,   // 经采样锁存的 12 位原始 ADC 码值
    output reg  signed [31:0]     theta_err_q16,  // 摆杆偏离垂直倒立位置的弧度偏差 (Q12.16, 0 代表垂直)
    output reg  signed [31:0]     dtheta_q16,     // 摆杆滤波后角速度 (Q12.16 rad/s)
    output reg                    sample_done     // 单次采样转换解算完成脉冲
);

    // -------------------------------------------------------------------------
    // 1. SPI 时钟分频器生成 (50MHz -> 2.5MHz, 分频比 = 50M / (2.5M * 2) = 10)
    // -------------------------------------------------------------------------
    localparam integer SCLK_DIV = CLK_FREQ_HZ / (SPI_SCLK_HZ * 2);
    reg [7:0] sclk_cnt;
    reg       sclk_pulse;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_cnt   <= 8'd0;
            sclk_pulse <= 1'b0;
        end else begin
            if (sclk_cnt >= SCLK_DIV - 1) begin
                sclk_cnt   <= 8'd0;
                sclk_pulse <= 1'b1;
            end else begin
                sclk_cnt   <= sclk_cnt + 1'b1;
                sclk_pulse <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 2. SPI 通信状态机 (标准 16 位传输时序，读取 12 位有效 ADC 转换数据)
    // -------------------------------------------------------------------------
    localparam STATE_IDLE    = 3'd0;
    localparam STATE_START   = 3'd1;
    localparam STATE_XFER    = 3'd2;
    localparam STATE_FINISH  = 3'd3;

    reg [2:0]  spi_state;
    reg [4:0]  bit_index;
    reg [15:0] shift_reg;
    reg [11:0] adc_latch;
    reg        adc_valid_internal;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_state          <= STATE_IDLE;
            spi_cs_n           <= 1'b1;
            spi_sclk           <= 1'b0;
            bit_index          <= 5'd0;
            shift_reg          <= 16'd0;
            adc_latch          <= 12'd0;
            adc_valid_internal <= 1'b0;
        end else begin
            adc_valid_internal <= 1'b0;

            case (spi_state)
                STATE_IDLE: begin
                    spi_cs_n <= 1'b1;
                    spi_sclk <= 1'b0;
                    // 当收到 1ms 控制脉冲时，启动一次新的 SPI 帧采样
                    if (calc_en) begin
                        spi_cs_n  <= 1'b0;
                        bit_index <= 5'd0;
                        shift_reg <= 16'd0;
                        spi_state <= STATE_START;
                    end
                end

                STATE_START: begin
                    if (sclk_pulse) begin
                        spi_sclk  <= 1'b0;
                        spi_state <= STATE_XFER;
                    end
                end

                STATE_XFER: begin
                    if (sclk_pulse) begin
                        spi_sclk <= ~spi_sclk;
                        // 在 SCLK 上升沿采样 MISO 输入引脚 (SPI Mode 0)
                        if (~spi_sclk == 1'b1) begin
                            shift_reg <= {shift_reg[14:0], spi_miso};
                            bit_index <= bit_index + 1'b1;
                            if (bit_index >= 5'd15) begin
                                spi_state <= STATE_FINISH;
                            end
                        end
                    end
                end

                STATE_FINISH: begin
                    if (sclk_pulse) begin
                        spi_cs_n           <= 1'b1;
                        spi_sclk           <= 1'b0;
                        // 提取 12 位转换数据 (典型 ADC 如 ADS7886/MCP3201 位于中间高位或低 12 位)
                        adc_latch          <= shift_reg[11:0];
                        adc_valid_internal <= 1'b1;
                        spi_state          <= STATE_IDLE;
                    end
                end

                default: spi_state <= STATE_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 3. 原始数据选择 (支持内部 SPI 采集或外部直接输入)
    // -------------------------------------------------------------------------
    wire        process_trigger = adc_valid_internal | ext_raw_valid;
    wire [11:0] active_raw_adc  = ext_raw_valid ? ext_raw_data : adc_latch;

    // -------------------------------------------------------------------------
    // 4. 角度绝对零点扣减与 360° 全圆周越界解卷绕 (Unwrapping)
    // -------------------------------------------------------------------------
    // 12 位 ADC 刻度范围: [0, 4095]
    // 设零点为 zero_offset_raw，则相对偏离量 diff = active_raw_adc - zero_offset_raw
    // 若 diff > 2047，说明从逆时针转过了半周边界，将其平移 -4096 (映射到 [-2048, 2047])
    // 若 diff < -2048，说明从顺时针转过了半周边界，将其平移 +4096
    // 这样摆杆在垂直向上 (theta = 0) 附近做微小左右摆动时，diff 严格围绕 0 线性连续变化！
    // -------------------------------------------------------------------------
    reg signed [12:0] diff_raw;
    reg signed [12:0] diff_unwrapped;

    always @(*) begin
        diff_raw = $signed({1'b0, active_raw_adc}) - $signed({1'b0, zero_offset_raw});
        if (diff_raw > 13'sd2047) begin
            diff_unwrapped = diff_raw - 13'sd4096;
        end else if (diff_raw < -13'sd2048) begin
            diff_unwrapped = diff_raw + 13'sd4096;
        end else begin
            diff_unwrapped = diff_raw;
        end
    end

    // -------------------------------------------------------------------------
    // 5. 角度尺度变换 (刻度 -> Q12.16 弧度) 与差分角速度滤波
    // -------------------------------------------------------------------------
    // 4096 刻度对应 2*pi rad
    // 定点数 Q16: theta_q16 = diff * (2 * pi * 65536 / 4096)
    // 2 * pi * 65536 = 411774.8 ~= 411775
    // 变换公式: (diff * 411775) >>> 12 (相当于先乘 411775 再除以 4096)
    //
    // 角速度计算:
    // 在 1ms 周期内: dtheta_raw = (theta_err_q16 - theta_err_prev) * 1000
    // 一阶 IIR 滤波: dtheta = dtheta + 0.35 * (dtheta_raw - dtheta)
    // -------------------------------------------------------------------------
    localparam signed [31:0] SCALE_NUM_Q16    = 32'sd411775;
    localparam signed [31:0] FILTER_ALPHA_Q16 = 32'sd22938; // 滤波系数 0.35 * 65536 (消除除法器 E4)

    // 64 位中间乘法节点避免位宽截断与符号位混淆
    wire signed [63:0] diff_scaled_64     = $signed({{51{diff_unwrapped[12]}}, diff_unwrapped}) * $signed(SCALE_NUM_Q16);
    wire signed [63:0] diff_rad_shifted   = diff_scaled_64 >>> 12;
    wire signed [31:0] diff_rad_q16       = diff_rad_shifted[31:0];

    reg signed [31:0] theta_prev_q16;
    reg signed [31:0] raw_dtheta_q16;
    reg               filter_step;

    // 拍 1 组合逻辑: 角度与角速度差分解算
    wire signed [31:0] current_theta_q16  = (INVERT_DIR == 0) ? diff_rad_q16 : (-diff_rad_q16);
    wire signed [31:0] diff_theta_step    = current_theta_q16 - theta_prev_q16;
    wire signed [47:0] raw_dtheta_mult    = $signed(diff_theta_step) * 32'sd1000;
    wire signed [31:0] raw_dtheta_calc    = raw_dtheta_mult[31:0];

    // 拍 2 组合逻辑: 针对已寄存的 raw_dtheta_q16 执行 IIR 滤波乘法 (彻底切断两级乘法器长路径)
    wire signed [47:0] filter_diff_mult   = $signed(raw_dtheta_q16 - dtheta_q16) * $signed(FILTER_ALPHA_Q16);
    wire signed [47:0] filter_diff_shift  = filter_diff_mult >>> 16;
    wire signed [31:0] filter_diff_q16    = filter_diff_shift[31:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            raw_adc_data     <= 12'd0;
            theta_err_q16    <= 32'sd0;
            theta_prev_q16   <= 32'sd0;
            raw_dtheta_q16   <= 32'sd0;
            dtheta_q16       <= 32'sd0;
            sample_done      <= 1'b0;
            filter_step      <= 1'b0;
        end else if (process_trigger) begin
            // 流水拍 1: 锁存 ADC 码值并更新角度与未滤波角速度寄存器
            raw_adc_data   <= active_raw_adc;
            theta_err_q16  <= current_theta_q16;
            theta_prev_q16 <= current_theta_q16;
            raw_dtheta_q16 <= raw_dtheta_calc;
            filter_step    <= 1'b1;
            sample_done    <= 1'b0;
        end else if (filter_step) begin
            // 流水拍 2: 基于寄存器执行低通滤波乘法，并产生 sample_done 脉冲
            dtheta_q16     <= dtheta_q16 + filter_diff_q16;
            filter_step    <= 1'b0;
            sample_done    <= 1'b1;
        end else begin
            sample_done    <= 1'b0;
        end
    end

endmodule
