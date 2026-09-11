// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: j280_hw_top_tb
// 功能描述: J280 硬件顶层模块完整闭环仿真测试平台 (Testbench)
//           测试内容:
//           1. 50MHz 系统时钟与异步复位释放
//           2. 摆杆角度传感器 SPI Mode 0 接口时序与一键零位标定验证
//           3. 电机正交编码器 A/B 相正向/反向旋转信号注入与 4 倍频计数
//           4. LQR 最优姿态自平衡控制核运算响应与 20kHz 电机 PWM 驱动波形验证
//           5. 跌落超限 (>45度) 硬件急停自锁保护动作验证
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
        simulated_adc_data = 12'd2048; // 模拟当前摆杆正好处于 2048

        // 复位释放
        #100;
        rst_n = 1'b1;
        #200;

        // ---------------------------------------------------------------------
        // TEST 1: 一键硬件零点自适应标定测试
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 测试一键垂直零点标定按键...");
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
        // TEST 3: 电机使能与 LQI 控制闭环响应测试
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 开启电机使能，模拟摆杆轻微倾斜 (2060, 偏角约 +1.05度)...");
        sw_motor_en = 1'b1;
        simulated_adc_data = 12'd2060; // 偏离标定的 2048 零点 +12 个码值

        // 等待下一个完整控制节拍触发 SPI 采样与 LQR 计算
        #25000;

        $display("  ADC 采样码值 raw_adc: %0d, 零位偏差 diff: %0d", 
                 u_dut.raw_adc_data, u_dut.u_angle_sensor.diff_unwrapped);
        $display("  当前状态量 -> theta_err: %0d (Q16), alpha_err: %0d (Q16)", 
                 u_dut.theta_err_q16, u_dut.alpha_err_q16);
        $display("  LQR 控制器输出占空比 pwm_duty: %0d", u_dut.lqr_pwm_duty);
        $display("  电机驱动接口输出 -> PWM: %b, DIR: %b, IN1: %b, IN2: %b, STBY: %b", 
                 motor_pwm, motor_dir, motor_in1, motor_in2, motor_stby);

        if (motor_stby == 1'b1 && u_dut.lqr_pwm_duty != 0 && u_dut.raw_adc_data == 2060) begin
            $display("  [PASS] LQI 控制器闭环运算与电机驱动输出正常响应 (PWM 占空比输出正常)!");
        end else begin
            $display("  [FAIL] 控制响应异常!");
            test_pass = 0;
        end

        // ---------------------------------------------------------------------
        // TEST 4: 跌落超限 (>45度) 硬件安全自锁急停保护测试
        // ---------------------------------------------------------------------
        $display("\n[TEST 4] 模拟摆杆失衡倾倒 (>45度, ADC 码值设为 2600)...");
        simulated_adc_data = 12'd2600; // 偏离零点 552 个码值 (约 48.5度 > 45度)
        #25000;

        $display("  ADC 采样码值: %0d, theta_err: %0d (Q16)", u_dut.raw_adc_data, u_dut.theta_err_q16);
        $display("  跌落保护触发标志 is_fall_down: %b", u_dut.is_fall_down);
        $display("  安全门控输出占空比 safe_pwm_duty: %0d", u_dut.safe_pwm_duty);
        $display("  驱动输出引脚 -> PWM: %b, IN1: %b, IN2: %b", motor_pwm, motor_in1, motor_in2);

        if (u_dut.is_fall_down == 1'b1 && u_dut.safe_pwm_duty == 0) begin
            $display("  [PASS] 倾角超限跌落保护自锁功能完美触发，PWM 已被硬件完全切断!");
        end else begin
            $display("  [FAIL] 跌落安全自锁未按预期切断动力!");
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
