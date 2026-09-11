`ifndef SYNTHESIS

`timescale 1ns / 1ps

module furuta_lqr_ctrl_tb;

    // -------------------------------------------------------------------------
    // 信号定义
    // -------------------------------------------------------------------------
    reg         clk;
    reg         rst_n;
    reg         calc_en;

    // Q12.16 定点数状态输入
    reg signed [31:0] theta_err;
    reg signed [31:0] dtheta;
    reg signed [31:0] alpha_err;
    reg signed [31:0] dalpha;
    reg signed [31:0] alpha_int;

    // 模块输出
    wire signed [15:0] pwm_duty;
    wire               calc_done;

    // -------------------------------------------------------------------------
    // 被测模块 (DUT) 例化
    // -------------------------------------------------------------------------
    furuta_lqr_ctrl uut (
        .clk        (clk),
        .rst_n      (rst_n),
        .calc_en    (calc_en),
        .theta_err  (theta_err),
        .dtheta     (dtheta),
        .alpha_err  (alpha_err),
        .dalpha     (dalpha),
        .alpha_int  (alpha_int),
        .pwm_duty   (pwm_duty),
        .calc_done  (calc_done)
    );

    // -------------------------------------------------------------------------
    // 50MHz 时钟生成 (周期 20ns)
    // -------------------------------------------------------------------------
    initial clk = 1'b0;
    always #10 clk = ~clk;

    // 计数器与统计
    integer pass_count = 0;
    integer fail_count = 0;

    // 辅助任务: 发送一次计算使能并等待结果输出 (验证 3 级流水线延时)
    task run_calc_check;
        input [8*32:1] test_name;
        input signed [15:0] exp_min;
        input signed [15:0] exp_max;
        integer cycle_delay;
        begin
            cycle_delay = 0;
            @(posedge clk);
            calc_en = 1'b1;
            @(posedge clk);
            calc_en = 1'b0;

            // 等待 calc_done 脉冲，并检测确定性延迟
            while (!calc_done && cycle_delay < 10) begin
                @(posedge clk);
                cycle_delay = cycle_delay + 1;
            end

            if (calc_done) begin
                $display("[TB 监控] %0s -> 延迟周期: %0d 拍 (理论: 3拍) | 输出 PWM: %0d", 
                         test_name, cycle_delay, pwm_duty);
                
                // 检查延迟是否严格为 3 拍 (60ns)
                if (cycle_delay != 3) begin
                    $display("  [ERROR] 流水线延迟异常! 期望 3 拍，实际得到 %0d 拍", cycle_delay);
                    fail_count = fail_count + 1;
                end
                // 检查输出范围是否在合理区间
                else if (pwm_duty >= exp_min && pwm_duty <= exp_max) begin
                    $display("  [PASS] 结果校验通过! 处于预期范围 [%0d, %0d]", exp_min, exp_max);
                    pass_count = pass_count + 1;
                end else begin
                    $display("  [ERROR] 输出占空比错误! 实际: %0d, 期望区间: [%0d, %0d]", 
                             pwm_duty, exp_min, exp_max);
                    fail_count = fail_count + 1;
                end
            end else begin
                $display("  [ERROR] 计算超时! 未检测到 calc_done 脉冲");
                fail_count = fail_count + 1;
            end
            @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // 测试向量激励序列
    // -------------------------------------------------------------------------
    initial begin
        $display("================================================================");
        $display("  高云 GW2A FPGA 倒立摆 LQI 硬件控制器 (furuta_lqr_ctrl) 功能仿真");
        $display("================================================================");

        // 1. 系统复位
        rst_n     = 1'b0;
        calc_en   = 1'b0;
        theta_err = 32'sd0;
        dtheta    = 32'sd0;
        alpha_err = 32'sd0;
        dalpha    = 32'sd0;
        alpha_int = 32'sd0;
        #100;
        rst_n = 1'b1;
        #40;

        // ---------------------------------------------------------------------
        // Case 1: 零输入稳态工况 (所有状态偏差为 0)
        // 期望: 输出 PWM 必须严格为 0
        // ---------------------------------------------------------------------
        theta_err = 32'sd0;
        dtheta    = 32'sd0;
        alpha_err = 32'sd0;
        dalpha    = 32'sd0;
        alpha_int = 32'sd0;
        run_calc_check("Case 1: 绝对平衡零偏差工况", 16'sd0, 16'sd0);

        // ---------------------------------------------------------------------
        // Case 2: 正向小倾角工况 (摆杆向正方向倾斜 +0.02 rad ≈ 1.15度)
        // 在 Q16 格式下: 0.02 * 65536 = 1311
        // 期望: 摆角增益 K1 为负，根据 V = -K*x，输出应为正电压驱动电机追赶摆杆
        // ---------------------------------------------------------------------
        theta_err = 32'sd1311;
        dtheta    = 32'sd0;
        alpha_err = 32'sd0;
        dalpha    = 32'sd0;
        alpha_int = 32'sd0;
        run_calc_check("Case 2: 正向微小倾角偏差 (+0.02 rad)", 16'sd120, 16'sd160);

        // ---------------------------------------------------------------------
        // Case 3: 负向小倾角工况 (摆杆向负方向倾斜 -0.02 rad ≈ -1.15度)
        // 在 Q16 格式下: -0.02 * 65536 = -1311
        // 验证有符号数补码运算与算术右移，期望输出完全对称的负电压
        // ---------------------------------------------------------------------
        theta_err = -32'sd1311;
        dtheta    = 32'sd0;
        alpha_err = 32'sd0;
        dalpha    = 32'sd0;
        alpha_int = 32'sd0;
        run_calc_check("Case 3: 负向微小倾角偏差 (-0.02 rad)", -16'sd160, -16'sd120);

        // ---------------------------------------------------------------------
        // Case 4: 超大正向扰动饱和工况 (摆杆倾角 +0.5 rad ≈ 28.6度)
        // 在 Q16 格式下: 0.5 * 65536 = 32768
        // 期望: 模块触发硬件上限饱和保护，输出严格限幅在 PWM_MAX (+1000)，不能溢出反转
        // ---------------------------------------------------------------------
        theta_err = 32'sd32768;
        dtheta    = 32'sd0;
        alpha_err = 32'sd0;
        dalpha    = 32'sd0;
        alpha_int = 32'sd0;
        run_calc_check("Case 4: 正向极大偏差硬件上限饱和", 16'sd1000, 16'sd1000);

        // ---------------------------------------------------------------------
        // Case 5: 超大负向扰动饱和工况 (摆杆倾角 -0.5 rad ≈ -28.6度)
        // 期望: 模块触发硬件下限饱和保护，输出严格限幅在 PWM_MIN (-1000)，不能溢出反转
        // ---------------------------------------------------------------------
        theta_err = -32'sd32768;
        dtheta    = 32'sd0;
        alpha_err = 32'sd0;
        dalpha    = 32'sd0;
        alpha_int = 32'sd0;
        run_calc_check("Case 5: 负向极大偏差硬件下限饱和", -16'sd1000, -16'sd1000);

        // ---------------------------------------------------------------------
        // Case 6: 多变量复合工况 (摆角 + 角速度 + 转臂位移 + 积分消除静差)
        // ---------------------------------------------------------------------
        theta_err = 32'sd655;   // +0.01 rad
        dtheta    = -32'sd1311; // -0.02 rad/s
        alpha_err = 32'sd1966;  // +0.03 rad
        dalpha    = 32'sd0;
        alpha_int = 32'sd655;   // +0.01 rad*s
        run_calc_check("Case 6: 多状态耦合与积分消除静差工况", 16'sd60, 16'sd110);

        // ---------------------------------------------------------------------
        // 总结报告
        // ---------------------------------------------------------------------
        #100;
        $display("================================================================");
        if (fail_count == 0) begin
            $display("🎉 恭喜! 全部 %0d 项 Testbench 测试用例 100%% 通过!", pass_count);
            $display("   -> 流水线延迟确定性 (60ns / 3 拍) 验证通过;");
            $display("   -> 有符号数符号扩展与 Q16 移位截断验证通过;");
            $display("   -> 极限大误差硬件饱和防溢出 (±1000) 验证通过。");
        end else begin
            $display("❌ 测试未通过! 成功: %0d, 失败: %0d", pass_count, fail_count);
        end
        $display("================================================================");
        $finish;
    end

endmodule

`endif
