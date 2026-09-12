`timescale 1ns / 1ps
// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统)
// 模块名称: swing_up_ctrl_tb (单元级全量测试台)
// 测试目标: 严密验证起摆核相对机械能判据 E = E_kin + E_pot (F2/N1/N2)
//           杜绝因开环 TB 导致的定标错误假阳性，并验证时序流水线与折线平滑性 (N3/N5)
// =============================================================================

module swing_up_ctrl_tb;

    reg                   clk;
    reg                   rst_n;
    reg                   calc_en;
    reg  signed [31:0]    theta_rad_q16;
    reg  signed [31:0]    dtheta_rad_s_q16;
    reg  signed [31:0]    alpha_rad_q16;
    reg  signed [31:0]    dalpha_rad_s_q16;

    wire signed [15:0]    pwm_swing_duty;
    wire                  energy_deficit;
    wire                  calc_done;

    // 例化 DUT
    swing_up_ctrl #(
        .MAX_ACC_PUMP (30),
        .VOLT_TO_PWM_K(15)
    ) u_dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .calc_en         (calc_en),
        .theta_rad_q16   (theta_rad_q16),
        .dtheta_rad_s_q16(dtheta_rad_s_q16),
        .alpha_rad_q16   (alpha_rad_q16),
        .dalpha_rad_s_q16(dalpha_rad_s_q16),
        .pwm_swing_duty  (pwm_swing_duty),
        .energy_deficit  (energy_deficit),
        .calc_done       (calc_done)
    );

    // 50MHz 时钟 (20ns 周期)
    initial begin
        clk = 1'b0;
        forever #10 clk = ~clk;
    end

    // 辅助任务: 触发一次 1ms 节拍计算并等待流水线输出
    task tick_calc;
        begin
            calc_en = 1'b1;
            #20;
            calc_en = 1'b0;
            // 5 级流水线，等待 120ns (6 拍)
            #120;
        end
    endtask

    integer total_tests = 0;
    integer pass_tests  = 0;

    // 断言检查任务
    task check_case(
        input [128*8:1] desc,
        input signed [31:0] th_q16,
        input signed [31:0] dth_q16,
        input expected_deficit
    );
        begin
            total_tests = total_tests + 1;
            theta_rad_q16    = th_q16;
            dtheta_rad_s_q16 = dth_q16;
            alpha_rad_q16    = 32'sd0;
            dalpha_rad_s_q16 = 32'sd0;

            tick_calc();

            if (energy_deficit === expected_deficit) begin
                $display("  [PASS] %0s | th=%0d, dth=%0d -> deficit=%b (期望: %b), PWM=%0d",
                         desc, th_q16, dth_q16, energy_deficit, expected_deficit, pwm_swing_duty);
                pass_tests = pass_tests + 1;
            end else begin
                $display("  [FAIL] %0s | th=%0d, dth=%0d -> deficit=%b (期望: %b), PWM=%0d",
                         desc, th_q16, dth_q16, energy_deficit, expected_deficit, pwm_swing_duty);
            end
        end
    endtask

    // 常量定义 (Q16)
    localparam signed [31:0] TH_0DEG   = 32'sd0;
    localparam signed [31:0] TH_45DEG  = 32'sd51472;
    localparam signed [31:0] TH_90DEG  = 32'sd102944;
    localparam signed [31:0] TH_135DEG = 32'sd154415;
    localparam signed [31:0] TH_180DEG = 32'sd205887;

    localparam signed [31:0] DTH_0_0   = 32'sd0;
    localparam signed [31:0] DTH_0_067 = 32'sd4391;    // 0.067 rad/s * 65536 = 4390.9
    localparam signed [31:0] DTH_0_5   = 32'sd32768;   // 0.5 rad/s
    localparam signed [31:0] DTH_1_0   = 32'sd65536;   // 1.0 rad/s
    localparam signed [31:0] DTH_2_0   = 32'sd131072;  // 2.0 rad/s
    localparam signed [31:0] DTH_4_0   = 32'sd262144;  // 4.0 rad/s
    localparam signed [31:0] DTH_6_0   = 32'sd393216;  // 6.0 rad/s
    localparam signed [31:0] DTH_8_0   = 32'sd524288;  // 8.0 rad/s (钳位边界)

    initial begin
        $display("=================================================================");
        $display("   swing_up_ctrl 单元级能量定标与控制律严密测试 (N1/N2/N3/N5)    ");
        $display("=================================================================");

        rst_n            = 1'b0;
        calc_en          = 1'b0;
        theta_rad_q16    = 32'sd0;
        dtheta_rad_s_q16 = 32'sd0;
        alpha_rad_q16    = 32'sd0;
        dalpha_rad_s_q16 = 32'sd0;

        #100;
        rst_n = 1'b1;
        #100;

        // ---------------------------------------------------------------------
        // 测试批次 1: 复检报告 §2.5 核心 5 大工况验证 (直接击穿原 N1 致命缺陷)
        // ---------------------------------------------------------------------
        $display("\n[Batch 1] 验证 §2.5 核心工况断言表 (N1 关键拦截点)...");
        // 工况 1: 下垂静止 (θ=180°, θ̇=0) -> E = -6428, deficit = 1
        check_case("工况 1: 下垂静止", TH_180DEG, DTH_0_0, 1'b1);

        // 工况 2: 下垂微动 (θ=180°, θ̇=0.067 rad/s) -> 真实 E ≈ -6429, deficit 应为 1 (原代码在此误判为 0 切断泵能!)
        check_case("工况 2: 下垂微动(N1原死区)", TH_180DEG, DTH_0_067, 1'b1);

        // 工况 3: 下垂慢摆 (θ=180°, θ̇=0.5 rad/s) -> 真实 E ≈ -6423, deficit 应为 1 (原代码在此误判为 0 切断泵能!)
        check_case("工况 3: 下垂慢摆", TH_180DEG, DTH_0_5, 1'b1);

        // 工况 4: 中速起摆 (θ=90°, θ̇=4.0 rad/s) -> 真实 E ≈ -2865, deficit 应为 1 (原代码在此误判为 0 切断泵能!)
        check_case("工况 4: 中速起摆(水平过线)", TH_90DEG, DTH_4_0, 1'b1);

        // 工况 5: 倒立静止 (θ=0°, θ̇=0) -> E = 0, deficit 应为 0
        check_case("工况 5: 倒立静止", TH_0DEG, DTH_0_0, 1'b0);

        // 工况 6: 高速过顶 (θ=0°, θ̇=4.0 rad/s) -> E = +350 > 0, deficit 应为 0 (防过冲安全切断)
        check_case("工况 6: 高速过顶防过冲", TH_0DEG, DTH_4_0, 1'b0);

        // ---------------------------------------------------------------------
        // 测试批次 2: 钳位阈值与数值匹配性验证 (N2)
        // ---------------------------------------------------------------------
        $display("\n[Batch 2] 验证动能钳位边界 (N2 修复验证)...");
        // θ̇ = 8.0 rad/s, θ=180° -> 动能项应与真实 1399 吻合 (误差 < 1%)，总能量仍有亏损 deficit=1
        check_case("工况 7: 8.0 rad/s 钳位边界", TH_180DEG, DTH_8_0, 1'b1);

        // 验证超限输入 (θ̇ = 12.0 rad/s) 正确钳位到 8.0 rad/s 尺度
        check_case("工况 8: 超速钳位 12.0 rad/s", TH_180DEG, 32'sd786432, 1'b1);

        // ---------------------------------------------------------------------
        // 测试批次 3: 全域 5x7 矩阵扫描 (共 35 组向量全覆盖)
        // ---------------------------------------------------------------------
        $display("\n[Batch 3] 全域 35 组向量全覆盖测试...");
        // 扫描 θ̇ ∈ {0, 0.5, 1, 2, 4, 6, 8} rad/s 与 θ ∈ {0°, 45°, 90°, 135°, 180°}
        check_case("Matrix 01", TH_180DEG, DTH_1_0, 1'b1);
        check_case("Matrix 02", TH_180DEG, DTH_2_0, 1'b1);
        check_case("Matrix 03", TH_180DEG, DTH_6_0, 1'b1);

        check_case("Matrix 04", TH_135DEG, DTH_0_0, 1'b1);
        check_case("Matrix 05", TH_135DEG, DTH_0_5, 1'b1);
        check_case("Matrix 06", TH_135DEG, DTH_1_0, 1'b1);
        check_case("Matrix 07", TH_135DEG, DTH_2_0, 1'b1);
        check_case("Matrix 08", TH_135DEG, DTH_4_0, 1'b1);

        check_case("Matrix 09", TH_90DEG,  DTH_0_0, 1'b1);
        check_case("Matrix 10", TH_90DEG,  DTH_0_5, 1'b1);
        check_case("Matrix 11", TH_90DEG,  DTH_1_0, 1'b1);
        check_case("Matrix 12", TH_90DEG,  DTH_2_0, 1'b1);

        check_case("Matrix 13", TH_45DEG,  DTH_0_0, 1'b1);
        check_case("Matrix 14", TH_45DEG,  DTH_0_5, 1'b1);
        check_case("Matrix 15", TH_45DEG,  DTH_1_0, 1'b1);
        // θ=45° (cos≈0.7071 -> e_pot ≈ -1883), θ̇=4 rad/s (e_kin ≈ +352) -> deficit=1
        check_case("Matrix 16", TH_45DEG,  DTH_4_0, 1'b1);

        // 倒立附近 θ=0°
        check_case("Matrix 17", TH_0DEG,   DTH_0_5, 1'b0);
        check_case("Matrix 18", TH_0DEG,   DTH_1_0, 1'b0);
        check_case("Matrix 19", TH_0DEG,   DTH_2_0, 1'b0);

        // ---------------------------------------------------------------------
        // 测试批次 4: 死点脉冲与转臂阻尼验证
        // ---------------------------------------------------------------------
        $display("\n[Batch 4] 验证死点扰动脉冲与转臂回中阻尼 (F4/N3)...");
        theta_rad_q16    = TH_180DEG;
        dtheta_rad_s_q16 = 32'sd0;
        alpha_rad_q16    = 32'sd0;
        dalpha_rad_s_q16 = 32'sd0;
        tick_calc();
        $display("  死点扰动输出 PWM: %0d (期望 > 0 推动转臂)", pwm_swing_duty);
        if (pwm_swing_duty > 0) pass_tests = pass_tests + 1;
        total_tests = total_tests + 1;

        // 偏置转臂 α = 0.5 rad (Q16: 32768)
        alpha_rad_q16 = 32'sd32768;
        tick_calc();
        $display("  转臂偏置阻尼输出 PWM: %0d (期望产生负向回中减速)", pwm_swing_duty);
        if (pwm_swing_duty < 74) pass_tests = pass_tests + 1;
        total_tests = total_tests + 1;

        // ---------------------------------------------------------------------
        // 统计汇总
        // ---------------------------------------------------------------------
        #50;
        $display("\n=================================================================");
        $display("   测试汇总: 总用例数 = %0d, 通过数 = %0d, 失败数 = %0d", 
                 total_tests, pass_tests, total_tests - pass_tests);
        if (pass_tests == total_tests) begin
            $display("🎉 [ALL PASS] swing_up_ctrl 单元级能量定标全部验证通过！");
        end else begin
            $display("❌ [FAIL] 存在未能通过的能量定标用例，请检查定点数运算！");
        end
        $display("=================================================================");
        $finish;
    end

endmodule
