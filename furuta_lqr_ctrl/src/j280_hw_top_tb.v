// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: j280_hw_top_tb
// 功能描述: J280 硬件顶层模块完整闭环仿真测试平台 (Testbench)
//           测试内容:
//           1. 50MHz 系统时钟与异步复位释放
//           2. 摆杆角度传感器 SPI Mode 0 接口时序与一键零位自适应标定验证 (TEST 1)
//           3. 电机正交编码器 A/B 相正向旋转信号注入与 4 倍频硬件计数验证 (TEST 2)
//           4. 起摆能量泵 (P1) 与四态主控制状态机转换 (HANGING -> SWINGUP -> BALANCE) (TEST 3)
//           5. 跌落超限 (>45度) 硬件安全自锁与平滑回退保护验证 (TEST 4)
//           6. 复合按键 KEY2 短按模式轮换 (0° -> +45° -> -45° -> 0.2Hz正弦跟踪) 验证 (TEST 5)
//           7. 转臂多圈旋转软限位保护动作验证 (D3 / TEST 6)
// =============================================================================

`timescale 1ns / 1ps

module j280_hw_top_tb;

    // 信号声明
    reg        clk_50m;
    reg        rst_n;
    reg        sw_motor_en;
    reg        sw_brake_mode;
    reg        key_zero_calib;
    reg        key_pos_clear;

    wire       motor_pwm;
    wire       motor_dir;
    wire       motor_in1;
    wire       motor_in2;
    wire       motor_stby;

    reg        enc_a;
    reg        enc_b;
    reg        enc_z;

    wire       adc_cs_n;
    wire       adc_sclk;
    reg        adc_miso;

    wire       led_balance;
    wire       led_calib_ok;
    wire       led_motor_run;

    // 例化待测顶层模块 (DUT)
    // 覆盖参数以加速仿真运行 (500 周期即产生 1 次控制节拍，消抖 10 周期)
    j280_hw_top #(
        .CLK_FREQ_HZ          (50_000_000),
        .TIMER_1MS_LIMIT      (500),         // 仿真加速: 10us 一个控制周期
        .CALIB_DEBOUNCE_CYCLES(10)           // 仿真加速: 10 拍按键消抖
        // 修复 F13-c: 采用硬件默认参数 ARM_SOFT_LIMIT_Q16 = 32'sd823548 验证真实阈值
    ) u_dut (
        .clk_50m       (clk_50m),
        .rst_n         (rst_n),
        .sw_motor_en   (sw_motor_en),
        .sw_brake_mode (sw_brake_mode),
        .key_zero_calib(key_zero_calib),
        .key_pos_clear (key_pos_clear),
        .motor_pwm     (motor_pwm),
        .motor_dir     (motor_dir),
        .motor_in1     (motor_in1),
        .motor_in2     (motor_in2),
        .motor_stby    (motor_stby),
        .enc_a         (enc_a),
        .enc_b         (enc_b),
        .enc_z         (enc_z),
        .adc_cs_n      (adc_cs_n),
        .adc_sclk      (adc_sclk),
        .adc_miso      (adc_miso),
        .led_balance   (led_balance),
        .led_calib_ok  (led_calib_ok),
        .led_motor_run (led_motor_run)
    );

    // 产生 50MHz 系统时钟 (周期 20ns)
    initial begin
        clk_50m = 1'b0;
        forever #10 clk_50m = ~clk_50m;
    end

    // 模拟 SPI 从机 (12 位 ADC / 磁编码器，SPI Mode 0 严谨时序)
    reg [11:0] simulated_adc_data;
    reg [15:0] spi_tx_word;
    reg [15:0] spi_slave_shift;

    always @(negedge adc_cs_n) begin
        // CS_N 下降沿载入待发送数据并在首个时钟前输出 bit 15
        spi_tx_word     = {4'b0000, simulated_adc_data};
        adc_miso        <= spi_tx_word[15];
        spi_slave_shift <= {spi_tx_word[14:0], 1'b0};
    end

    always @(negedge adc_sclk) begin
        if (!adc_cs_n) begin
            adc_miso        <= spi_slave_shift[15];
            spi_slave_shift <= {spi_slave_shift[14:0], 1'b0};
        end
    end

    integer test_pass = 1;
    integer ramp_guard;      // TEST 5 斜坡轮询上限计数器 (与 RAMP_STEP_Q16 解耦)

    // 主测试激励
    initial begin
        $display("=================================================================");
        $display("   全国大学生嵌入式芯片竞赛 - J280 硬件顶层闭环测试开始          ");
        $display("=================================================================");

        // 1. 初始化输入
        rst_n          = 1'b0;
        sw_motor_en    = 1'b0;
        sw_brake_mode  = 1'b1;
        key_zero_calib = 1'b1;
        key_pos_clear  = 1'b1;
        enc_a          = 1'b0;
        enc_b          = 1'b0;
        enc_z          = 1'b0;
        adc_miso       = 1'b0;
        simulated_adc_data = 12'd2048; // 模拟当前摆杆正好处于 2048 (垂直中位)

        // 复位释放
        #100;
        rst_n = 1'b1;
        #200;

        // ---------------------------------------------------------------------
        // TEST 1: 一键硬件零点自适应标定测试
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 测试一键垂直零点标定按键 (KEY1)...");
        // 先等待首次 ADC 采样帧彻底完成 (10us 发起 + 6.4us 传输 = 16.4us，等待 20us)
        #20000;
        // 模拟按下零点标定按键 (持续 400ns > 10 拍消抖时间)
        key_zero_calib = 1'b0;
        #400;
        key_zero_calib = 1'b1;
        #200;

        $display("  零位标定锁存值 zero_offset_reg: %0d (期望: 2048)", u_dut.zero_offset_reg);
        if (led_calib_ok == 1'b0 && u_dut.zero_offset_reg == 2048) begin // 低电平点亮
            $display("  [PASS] 垂直零点成功标定并锁存 (2048)! 指示灯 led_calib_ok 已点亮");
        end else begin
            $display("  [FAIL] 垂直零点标定指示灯未点亮或数值错误!");
            test_pass = 0;
        end

        // ---------------------------------------------------------------------
        // TEST 2: 正交编码器 4 倍频鉴相计数测试 (正转注入)
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 模拟电机光电编码器 A/B 相正向旋转 (A 超前 B)...");
        // 顺时针旋转 4 个机械周期 = 16 个脉冲，每拍 200ns > 160ns (8级数字滤波防抖阈值)
        repeat (4) begin
            #200 enc_a = 1'b1;
            #200 enc_b = 1'b1;
            #200 enc_a = 1'b0;
            #200 enc_b = 1'b0;
        end
        #500;
        $display("  当前编码器脉冲计数值: %0d", u_dut.u_encoder.pulse_count);
        if (u_dut.u_encoder.pulse_count == 16) begin
            $display("  [PASS] 编码器 4 倍频正转计数严格正确 (16 counts)!");
        end else begin
            $display("  [FAIL] 编码器计数异常! 期望 16，实际: %0d", u_dut.u_encoder.pulse_count);
            test_pass = 0;
        end

        // ---------------------------------------------------------------------
        // TEST 3: 状态机起摆与直立平衡捕获闭环响应测试 (P1 / F2 / F6 / F7)
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 开启电机使能，测试起摆到自平衡状态切换...");
        // 先设为下垂态 (ADC 码值 0 对应 ~180 度下垂)
        simulated_adc_data = 12'd0;
        // 等待 IIR 滤波器将码值阶跃引起的伪大角速度充分收敛至 0 (下垂静止)
        #250000;
        sw_motor_en = 1'b1;
        #5000; // 在首个控制节拍捕获 FSM 切入瞬间的 3.0V 初始扰动脉冲 (F6)
        $display("  FSM 切入首拍 current_state: %0d (期望: 1=STATE_SWINGUP)", u_dut.fsm_state);
        $display("  切入首拍初始扰动脉冲 PWM: %0d (期望严格等于 250 对应 3.0V F6)", u_dut.final_pwm_duty);

        if (u_dut.fsm_state == 2'd1 && u_dut.final_pwm_duty == 16'sd250) begin
            $display("  [PASS] FSM 切入 SWINGUP 成功注入 3.0V(250) 起动死区突破脉冲 (F6 修复成功)!");
        end else begin
            $display("  [FAIL] 初始扰动脉冲异常! 实际 PWM: %0d", u_dut.final_pwm_duty);
            test_pass = 0;
        end

        #20000; // 进入后续起摆节拍，此时选通起摆核连续动力输出
        $display("  连续起摆态主状态机 current_state: %0d, 能量泵输出 PWM: %0d", u_dut.fsm_state, u_dut.final_pwm_duty);
        $display("  相对机械能亏损指示 energy_deficit: %b (期望: 1, 机械能亏损需要泵能 F2)", u_dut.u_swing_ctrl.energy_deficit);
        
        if (u_dut.fsm_state == 2'd1 && u_dut.final_pwm_duty != 0 && u_dut.u_swing_ctrl.energy_deficit == 1'b1) begin
            $display("  [PASS] 能量泵持续放行动力，机械能亏损判据有效 (F2 修复成功)!");
        end else begin
            $display("  [FAIL] 连续起摆动力或机械能判据异常!");
            test_pass = 0;
        end

        // 模拟摆杆摆至直立平衡区 (ADC 码值 2060，偏离零点 12 码值，倾角 ~1.05度 < 22度)
        simulated_adc_data = 12'd2060;
        // 等待控制节拍让滤波后角速度充分收敛至 4.0 rad/s 捕获窗口内
        #250000;
        $display("  摆杆入区滤波稳定后: dtheta = %0d, current_state = %0d (期望: 2=STATE_BALANCE)", 
                 u_dut.dtheta_q16, u_dut.fsm_state);
        $display("  LQR 平衡控制器输出 PWM: %0d (后台常开流水，捕获切入瞬间无 0 输出空窗 F7)", u_dut.final_pwm_duty);
        if (u_dut.fsm_state == 2'd2 && led_balance == 1'b0 && u_dut.final_pwm_duty != 0) begin
            $display("  [PASS] 倒立摆成功捕获切入自平衡区 (STATE_BALANCE)，LQR 控制量实时生效 (F7 修复成功)!");
        end else begin
            $display("  [FAIL] 未切入自平衡态或捕获空窗输出为零!");
            test_pass = 0;
        end

        // ---------------------------------------------------------------------
        // TEST 4: 跌落超限 (>45度) 保护测试 (P1 状态回退)
        // ---------------------------------------------------------------------
        $display("\n[TEST 4] 模拟平衡态下受外力推倒 (>45度, ADC 码值设为 2600)...");
        simulated_adc_data = 12'd2600; // 偏离零点 552 个码值 (约 48.5度 > 45度)
        #25000;

        $display("  跌落后主状态机 current_state: %0d (期望回退至: 1=STATE_SWINGUP)", u_dut.fsm_state);
        if (u_dut.fsm_state == 2'd1) begin
            $display("  [PASS] 平衡态跌落后自动回退至起摆态重新拉起，系统具备自恢复鲁棒性!");
        end else begin
            $display("  [FAIL] 跌落状态未正确转换!");
            test_pass = 0;
        end

        // ---------------------------------------------------------------------
        // TEST 5: 复合按键 KEY2 短按模式轮换测试 (P2 / 拓展要求 1 & 2 / F8 平滑斜坡)
        // ---------------------------------------------------------------------
        $display("\n[TEST 5] 模拟短按 KEY2 切换控制轨迹模式并验证斜坡平滑器 (F8)...");
        // 恢复至平衡态
        simulated_adc_data = 12'd2048;
        #100000;

        // 模拟短按 KEY2: 按下 300ns 后释放
        key_pos_clear = 1'b0;
        #300;
        key_pos_clear = 1'b1;
        #5000; // 5us 后观察斜坡平滑上升状态
        $display("  切换第 1 次后 traj_mode: %0d, 目标过渡中: alpha_ref: %0d (平滑步进中)", u_dut.traj_mode, u_dut.alpha_ref_q16);

        // 轮询等待斜坡走完，不使用固定等待时长。
        // 原写法为 #1200000 (120 个控制周期，DUT 用 TIMER_1MS_LIMIT=500 即 10us/周期)，
        // 隐含假设 traj_gen.v 的 RAMP_STEP_Q16=572（90 拍走完 45 度）。该常数降为
        // 57 (0.05 度/ms) 后需 904 拍，固定等待立即失效并报假 FAIL。
        // 轮询 + 上限保护彻底解耦：无论斜坡速率如何调整都成立。
        // 上限 3000 拍 = 30ms 仿真时间，对 904 拍需求有 3.3 倍余量。
        ramp_guard = 0;
        while (u_dut.alpha_ref_q16 != 32'sd51472 && ramp_guard < 3000) begin
            #10000;              // 10us = 1 个控制周期 (TIMER_1MS_LIMIT=500)
            ramp_guard = ramp_guard + 1;
        end
        $display("  斜坡走完后最终 alpha_ref: %0d (期望: 51472 (+45度)), 耗时 %0d 个控制周期",
                 u_dut.alpha_ref_q16, ramp_guard);
        if (u_dut.traj_mode == 2'd1 && u_dut.alpha_ref_q16 == 32'sd51472 && ramp_guard < 3000) begin
            $display("  [PASS] KEY2 短按成功切换至 +45 度定点伺服模式，且具备平滑斜坡过渡 (F8 修复成功)!");
        end else begin
            $display("  [FAIL] 模式切换或斜坡过渡失败! (traj_mode=%0d, alpha_ref=%0d, 轮询拍数=%0d)",
                     u_dut.traj_mode, u_dut.alpha_ref_q16, ramp_guard);
            test_pass = 0;
        end

        // ---------------------------------------------------------------------
        // TEST 6: 转臂软限位保护与故障自恢复测试 (D3 / F5 / F13-c 真实硬件阈值验证)
        // ---------------------------------------------------------------------
        $display("\n[TEST 6] 模拟转臂多圈超限保护触发 (F13-c: 验证真实阈值 823548) 与清除后自动恢复 (F5)...");
        // 先将摆杆设为下垂态
        simulated_adc_data = 12'd0;
        #100000;
        $display("  注入 2010 个完整正交机械周期 (8040 脉冲 > 8000 真实阈值)...");
        repeat (2010) begin
            #100 enc_a = 1'b1;
            #100 enc_b = 1'b1;
            #100 enc_a = 1'b0;
            #100 enc_b = 1'b0;
        end
        #40000;
        $display("  当前转臂角度 alpha_rad_q16: %0d (硬件设定软限位阈值: %0d)", 
                 u_dut.alpha_rad_q16, u_dut.ARM_SOFT_LIMIT_Q16);
        $display("  超限报警 soft_limit_err: %b, current_state: %0d (期望: 3=STATE_PROTECT)", 
                 u_dut.soft_limit_err, u_dut.fsm_state);

        if (u_dut.soft_limit_err == 1'b1 && u_dut.fsm_state == 2'd3 && u_dut.final_pwm_duty == 0) begin
            $display("  [PASS] 转臂软限位保护生效，动力安全切断!");
        end else begin
            $display("  [FAIL] 软限位保护未正常触发!");
            test_pass = 0;
        end

        // 模拟长按 KEY2 清除转臂编码器位置
        $display("  模拟操作者长按 KEY2 清零转臂编码器...");
        key_pos_clear = 1'b0;
        #150000; // 长按触发 clear_pos_pulse
        key_pos_clear = 1'b1;
        #30000;
        $display("  清除位置后 soft_limit_err: %b, current_state: %0d (期望自动恢复至待机起摆态)", 
                 u_dut.soft_limit_err, u_dut.fsm_state);

        if (u_dut.soft_limit_err == 1'b0 && (u_dut.fsm_state == 2'd0 || u_dut.fsm_state == 2'd1)) begin
            $display("  [PASS] 软限位故障消除后状态机成功自恢复至待机起摆态 (F5 修复成功)!");
        end else begin
            $display("  [FAIL] 故障消除后未能自恢复!");
            test_pass = 0;
        end

        // ---------------------------------------------------------------------
        // 总结
        // ---------------------------------------------------------------------
        #5000;
        $display("\n=================================================================");
        if (test_pass) begin
            $display("🎉 J280 姿态控制系统硬件顶层集成仿真测试 ALL PASSED 全部通过！");
        end else begin
            $display("❌ 存在未通过的测试项，请检查仿真波形日志！");
        end
        $display("=================================================================");
        $finish;
    end

endmodule
