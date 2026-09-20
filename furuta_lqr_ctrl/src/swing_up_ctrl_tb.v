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
        .VOLT_TO_PWM_K(18)
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

    // -------------------------------------------------------------------------
    // 带数值断言的用例检查 (修复 M3)
    //
    // check_case 只比对 energy_deficit 这 1 bit，判据强度不足:
    // 由于 e_pot 的最大幅值仅 6428，任何 e_kin > 6428 都会使 deficit=0，
    // 因此钳位值在 [76200, 131071] 这个 41% 宽的区间内写错都不会被发现
    // (例如误写 92682 -> e_kin=11264，deficit 仍为 0)。实测验证:
    //   clamp=131071 -> e_kin=22527   deficit 无法区分
    //   clamp=110000 -> e_kin=15866   deficit 无法区分，但 e_kin 断言可检出
    //   clamp= 92682 -> e_kin=11264   同上
    //   clamp= 76201 -> e_kin= 7614   同上
    // 本 task 额外断言 e_kin_q16 与 e_pot_q16 的**精确数值**，
    // 使钳位值、缩放移位量、cos 表读数三者都被逐位约束。
    //
    // 可观测性: e_kin_q16 / e_pot_q16 是 Stage 1 的组合输出，其输入
    // dth_sq_s0 与 cos_q14_s0 是 Stage 0 寄存器，在 calc_en 之后保持到
    // 下一次 calc_en，故 tick_calc() 返回后可稳定采样 (纯观测，不构成反馈)。
    // -------------------------------------------------------------------------
    task check_case_full(
        input [128*8:1] desc,
        input signed [31:0] th_q16,
        input signed [31:0] dth_q16,
        input expected_deficit,
        input signed [31:0] exp_ekin,
        input signed [31:0] exp_epot
    );
        integer ekin_act, epot_act;
        integer bad;
        begin
            total_tests = total_tests + 1;
            theta_rad_q16    = th_q16;
            dtheta_rad_s_q16 = dth_q16;
            alpha_rad_q16    = 32'sd0;
            dalpha_rad_s_q16 = 32'sd0;

            tick_calc();

            ekin_act = u_dut.e_kin_q16;
            epot_act = u_dut.e_pot_q16;
            bad = 0;
            if (energy_deficit !== expected_deficit) bad = bad + 1;
            if (ekin_act !== exp_ekin)               bad = bad + 2;
            if (epot_act !== exp_epot)               bad = bad + 4;

            if (bad == 0) begin
                $display("  [PASS] %0s | dth=%0d -> e_kin=%0d e_pot=%0d deficit=%b (三项逐位吻合)",
                         desc, dth_q16, ekin_act, epot_act, energy_deficit);
                pass_tests = pass_tests + 1;
            end else begin
                $display("  [FAIL] %0s | dth=%0d -> e_kin=%0d(期望%0d) e_pot=%0d(期望%0d) deficit=%b(期望%b) [badmask=%0d: 1=deficit 2=e_kin 4=e_pot]",
                         desc, dth_q16, ekin_act, exp_ekin, epot_act, exp_epot,
                         energy_deficit, expected_deficit, bad);
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
    localparam signed [31:0] DTH_8_0   = 32'sd524288;  // 8.0 rad/s
    // 钳位阈值已扩至 ±32.0 rad/s (Q16: ±2097152)，以下三个常量用于验证新阈值
    localparam signed [31:0] DTH_19_86 = 32'sd1301776; // 19.864 rad/s = sqrt(4*m2*g*l2/Jp)，E=0 物理临界点 (官方 90g 摆杆参数)
    localparam signed [31:0] DTH_32_0  = 32'sd2097152; // 32.0 rad/s，钳位边界
    localparam signed [31:0] DTH_40_0  = 32'sd2621440; // 40.0 rad/s，超限应被钳位至 32.0

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
        // 工况 1: 下垂静止 (θ=180°, θ̇=0) -> E = -8680, deficit = 1
        check_case("工况 1: 下垂静止", TH_180DEG, DTH_0_0, 1'b1);

        // 工况 2: 下垂微动 (θ=180°, θ̇=0.067 rad/s) -> 真实 E ≈ -8681, deficit 应为 1
        check_case("工况 2: 下垂微动(N1原死区)", TH_180DEG, DTH_0_067, 1'b1);

        // 工况 3: 下垂慢摆 (θ=180°, θ̇=0.5 rad/s) -> 真实 E ≈ -8675, deficit 应为 1
        check_case("工况 3: 下垂慢摆", TH_180DEG, DTH_0_5, 1'b1);

        // 工况 4: 中速起摆 (θ=90°, θ̇=4.0 rad/s) -> 真实 E ≈ -3880, deficit 应为 1
        check_case("工况 4: 中速起摆(水平过线)", TH_90DEG, DTH_4_0, 1'b1);

        // 工况 5: 倒立静止 (θ=0°, θ̇=0) -> E = 0, deficit 应为 0
        check_case("工况 5: 倒立静止", TH_0DEG, DTH_0_0, 1'b0);

        // 工况 6: 高速过顶 (θ=0°, θ̇=4.0 rad/s) -> E = +352 > 0, deficit 应为 0 (防过冲安全切断)
        check_case("工况 6: 高速过顶防过冲", TH_0DEG, DTH_4_0, 1'b0);

        // ---------------------------------------------------------------------
        // 测试批次 2: 动能钳位阈值与物理可达性验证 (N2 重修)
        //
        // 钳位阈值与钳位值必须同为 ±32.0 rad/s。官方 90g 摆杆起摆物理需求为
        //   |theta_dot| >= sqrt(4*m2*g*l2/Jp) = 19.81 rad/s
        // 钳位到 32 rad/s 后 e_kin 上限为 22527 > 8680，能量判据可正常翻转。
        // ---------------------------------------------------------------------
        $display("\n[Batch 2] 验证动能钳位阈值/钳位值与 E=0 物理临界点 (N2 重修 + M3 数值断言)...");
        // 正向系列
        check_case_full("工况 7: +8.0 阈值内",   TH_180DEG, DTH_8_0,      1'b1, 32'sd1408,  -32'sd8680);
        check_case_full("工况 8: +12.0 阈值内",  TH_180DEG, 32'sd786432,  1'b1, 32'sd3168,  -32'sd8680);
        // 工况 9: **关键用例** 19.864 rad/s = E=0 的物理临界点
        // e_kin=8680 刚平衡 e_pot=-8680 -> e_tot=0 -> deficit=0 (能量已足，应停止泵能)
        check_case_full("工况 9: +19.86 E=0临界", TH_180DEG, DTH_19_86,   1'b0, 32'sd8680,  -32'sd8680);
        // 工况 10/11: 正向钳位。+32.0 (q16=2097152) 满足 >2097151 故走钳位分支得 131071；
        // +40.0 同样钳位到 131071，两者 e_kin 必须完全相同
        check_case_full("工况10: +32.0 钳位边界", TH_180DEG, DTH_32_0,    1'b0, 32'sd22527, -32'sd8680);
        check_case_full("工况11: +40.0 超限钳位", TH_180DEG, DTH_40_0,    1'b0, 32'sd22527, -32'sd8680);

        // 负向镜像系列 (M3)
        check_case_full("工况12: -8.0 阈值内",   TH_180DEG, -DTH_8_0,     1'b1, 32'sd1408,  -32'sd8680);
        check_case_full("工况13: -19.86 E=0临界", TH_180DEG, -DTH_19_86,  1'b0, 32'sd8680,  -32'sd8680);
        check_case_full("工况14: -32.0 钳位边界", TH_180DEG, -DTH_32_0,   1'b0, 32'sd22528, -32'sd8680);
        check_case_full("工况15: -40.0 超限钳位", TH_180DEG, -DTH_40_0,   1'b0, 32'sd22528, -32'sd8680);

        // ---------------------------------------------------------------------
        // 测试批次 3: 全域 5x7 矩阵扫描 (完整 35 组向量全覆盖)
        // 期望值由 Python 逐位复现 RTL 定点行为算出 (Jp=0.000675, m2*g*l2=0.06622)
        // 其中 Matrix 27 (E=-367) 与 Matrix 28 (E=+249) 为能量过零临界点
        // ---------------------------------------------------------------------
        $display("\n[Batch 3] 全域 5x7 = 35 组向量全覆盖测试...");
        // 扫描 θ̇ ∈ {0, 0.5, 1, 2, 4, 6, 8} rad/s 与 θ ∈ {180°, 135°, 90°, 45°, 0°}
        check_case("Matrix 01 th=180 dth=0.0", TH_180DEG, DTH_0_0, 1'b1);  // E=-8680
        check_case("Matrix 02 th=180 dth=0.5", TH_180DEG, DTH_0_5, 1'b1);  // E=-8675
        check_case("Matrix 03 th=180 dth=1.0", TH_180DEG, DTH_1_0, 1'b1);  // E=-8658
        check_case("Matrix 04 th=180 dth=2.0", TH_180DEG, DTH_2_0, 1'b1);  // E=-8592
        check_case("Matrix 05 th=180 dth=4.0", TH_180DEG, DTH_4_0, 1'b1);  // E=-8328
        check_case("Matrix 06 th=180 dth=6.0", TH_180DEG, DTH_6_0, 1'b1);  // E=-7888
        check_case("Matrix 07 th=180 dth=8.0", TH_180DEG, DTH_8_0, 1'b1);  // E=-7272

        check_case("Matrix 08 th=135 dth=0.0", TH_135DEG, DTH_0_0, 1'b1);  // E=-7371
        check_case("Matrix 09 th=135 dth=0.5", TH_135DEG, DTH_0_5, 1'b1);  // E=-7366
        check_case("Matrix 10 th=135 dth=1.0", TH_135DEG, DTH_1_0, 1'b1);  // E=-7349
        check_case("Matrix 11 th=135 dth=2.0", TH_135DEG, DTH_2_0, 1'b1);  // E=-7283
        check_case("Matrix 12 th=135 dth=4.0", TH_135DEG, DTH_4_0, 1'b1);  // E=-7019
        check_case("Matrix 13 th=135 dth=6.0", TH_135DEG, DTH_6_0, 1'b1);  // E=-6579
        check_case("Matrix 14 th=135 dth=8.0", TH_135DEG, DTH_8_0, 1'b1);  // E=-5963

        check_case("Matrix 15 th=90 dth=0.0",  TH_90DEG,  DTH_0_0, 1'b1);  // E=-4232
        check_case("Matrix 16 th=90 dth=0.5",  TH_90DEG,  DTH_0_5, 1'b1);  // E=-4227
        check_case("Matrix 17 th=90 dth=1.0",  TH_90DEG,  DTH_1_0, 1'b1);  // E=-4210
        check_case("Matrix 18 th=90 dth=2.0",  TH_90DEG,  DTH_2_0, 1'b1);  // E=-4144
        check_case("Matrix 19 th=90 dth=4.0",  TH_90DEG,  DTH_4_0, 1'b1);  // E=-3880
        check_case("Matrix 20 th=90 dth=6.0",  TH_90DEG,  DTH_6_0, 1'b1);  // E=-3440
        check_case("Matrix 21 th=90 dth=8.0",  TH_90DEG,  DTH_8_0, 1'b1);  // E=-2824

        check_case("Matrix 22 th=45 dth=0.0",  TH_45DEG,  DTH_0_0, 1'b1);  // E=-1159
        check_case("Matrix 23 th=45 dth=0.5",  TH_45DEG,  DTH_0_5, 1'b1);  // E=-1154
        check_case("Matrix 24 th=45 dth=1.0",  TH_45DEG,  DTH_1_0, 1'b1);  // E=-1137
        check_case("Matrix 25 th=45 dth=2.0",  TH_45DEG,  DTH_2_0, 1'b1);  // E=-1071
        check_case("Matrix 26 th=45 dth=4.0",  TH_45DEG,  DTH_4_0, 1'b1);  // E=-807
        check_case("Matrix 27 th=45 dth=6.0",  TH_45DEG,  DTH_6_0, 1'b1);  // E=-367
        check_case("Matrix 28 th=45 dth=8.0",  TH_45DEG,  DTH_8_0, 1'b0);  // E=+249 ← 已越过零点

        check_case("Matrix 29 th=0 dth=0.0",   TH_0DEG,   DTH_0_0, 1'b0);  // E=0    (倒立静止)
        check_case("Matrix 30 th=0 dth=0.5",   TH_0DEG,   DTH_0_5, 1'b0);  // E=+5
        check_case("Matrix 31 th=0 dth=1.0",   TH_0DEG,   DTH_1_0, 1'b0);  // E=+22
        check_case("Matrix 32 th=0 dth=2.0",   TH_0DEG,   DTH_2_0, 1'b0);  // E=+88
        check_case("Matrix 33 th=0 dth=4.0",   TH_0DEG,   DTH_4_0, 1'b0);  // E=+352
        check_case("Matrix 34 th=0 dth=6.0",   TH_0DEG,   DTH_6_0, 1'b0);  // E=+792
        check_case("Matrix 35 th=0 dth=8.0",   TH_0DEG,   DTH_8_0, 1'b0);  // E=+1408

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
        $display("  转臂偏置阻尼输出 PWM: %0d (期望产生负向回中减速, 阻尼后 < 90)", pwm_swing_duty);
        if (pwm_swing_duty < 90) pass_tests = pass_tests + 1;
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
