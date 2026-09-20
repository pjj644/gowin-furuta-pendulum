// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: furuta_plant_model
// 目标芯片: 无 (纯 Testbench 行为级组件, 整体由 `ifndef SYNTHESIS 包裹, 且
//           在 furuta_lqr_ctrl.gprj 中以 enable="0" 注册, 永不参与综合)
//
// 功能描述: 旋转倒立摆 (Furuta Pendulum) 被控对象非线性动力学行为级模型
//           与 j280_hw_top 构成完整硬件在环 (HIL) 闭环, 用于验证赛题的
//           闭环验收指标 (基础要求 1/3, 拓展要求 3), 弥补 F13-b 方法论缺口。
//
// 方程来源: 逐式移植自 C:\Users\28399\Desktop\GoWin\simulation\dynamics.py
//           FurutaDynamics.derivatives() / step_rk4() / normalize_theta()
//           物理参数逐条取自同目录 config.py 的 PendulumConfig (含 __post_init__
//           预计算的 J0 / km / b_emf 复合常数), 未做任何重新推导。
//
// 状态向量: [alpha, dalpha, theta, dtheta]
//           alpha  : 水平转臂角度 (rad), 多圈连续, 不做卷绕 (对应绝对编码器计数)
//           theta  : 摆杆角度 (rad), theta = 0 为垂直倒立平衡点,
//                    theta = +/-pi 为自由下垂稳定点 (与 dynamics.py 完全一致)
//
// 积分器  : 四阶龙格-库塔 (RK4), dt_phys = 0.2ms, 每 1ms 控制周期步进 5 次,
//           与 config.py 的 dt_phys / T_s 严格一致。
//
// -----------------------------------------------------------------------------
// 闭环连接方式 (全部经由真实外设引脚, 绝不直写 DUT 内部信号):
//
//   执行机构 (正算): DUT 的 motor_in1 / motor_in2 / motor_stby 引脚
//       -> 本模型在一个控制周期 (CTRL_CYCLES = 50000 clk = 1ms) 窗口内对
//          motor_in1 / motor_in2 的高电平时钟数分别计数 (in1_hi_cnt / in2_hi_cnt),
//          窗口长度 50000 = 20 x 2500 (PWM 载波周期) 为整数倍, 因此计数结果
//          恰好等于 20 个完整 PWM 周期的平均占空比, 无相位截断误差。
//       -> V_motor = V_SUPPLY * (in1_hi_cnt - in2_hi_cnt) / CTRL_CYCLES
//          (duty 满量程 1000 对应 in1 常高 -> +12.0V; in2 常高 -> -12.0V)
//       -> motor_stby = 0 (高阻待机) 时强制 V_motor = 0。
//       说明: 能耗制动 (motor_en=0 且 brake_mode=1) 时 IN1=IN2=1, 差分占空比为 0
//             -> V_motor = 0, 与"电枢两端短接"的物理情形一致; 而本模型的
//             tau_motor 中恒定含有 -b_emf*dalpha 反电动势阻尼项, 正是短接端
//             的电阻尼, 因此 TB 应将 sw_brake_mode 置 1 以保持物理自洽。
//             自由滑行 (IN1=IN2=0) 同样得到 V_motor = 0, 本模型不区分二者
//             (见文末"已知局限")。
//
//   摆杆角度传感器 (反算): theta -> 12-bit ADC 码值 -> SPI 从机 -> adc_miso 引脚
//       raw_adc = (ADC_ZERO_CODE + round(theta * ADC_COUNTS / (2*pi))) mod ADC_COUNTS
//               = (2048 + round(theta * 651.8873)) mod 4096
//       自检: theta=0 (倒立) -> 2048; theta=+pi (下垂) -> 4096 mod 4096 = 0;
//             theta=-pi      -> 0。与 j280_hw_top_tb.v 中用 12'd0 表示下垂一致。
//       该码值是 angle_sensor_reader.v 正变换
//           diff_unwrapped = raw_adc - zero_offset_raw (含 360° 解卷绕)
//           theta_err_q16  = diff_unwrapped * 411775 >>> 12
//       的精确逆映射 (651.8873 * 411775 / 4096 = 65535.6 ~= 65536)。
//       码值经 TB 中的 SPI 从机行为模型 (16 位帧, 数据置于低 12 位) 在
//       negedge adc_cs_n 处采样保持后经 adc_miso 串行回注 DUT, 因此 DUT 的
//       12-bit 量化、零点扣减、解卷绕、差分测速与一阶 IIR 滤波全部真实参与闭环。
//
//   转臂角度编码器 (反算): alpha -> 正交 A/B 电平 -> enc_a / enc_b 引脚
//       pulse = round(alpha * ENC_CPR / (2*pi)) = round(alpha * 636.6198)
//       s     = pulse mod 4  (负数归一到 [0,3])
//       A = (s==1)||(s==2) ;  B = (s==2)||(s==3)
//       序列核对 encoder_quad_reader.v 第 118~125 行鉴相表:
//           {a_dly,b_dly,a,b} = 4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00 -> +1
//       对应 (A,B) 依次为 00 -> 10 -> 11 -> 01 -> 00, 与本映射逐项吻合,
//       故 alpha 递增 -> pulse_count 递增 -> alpha_rad_q16 为正, 极性正确。
//       A/B 电平每 OUT_REFRESH_DIV (默认 100 clk = 2us) 刷新一次, 远大于
//       encoder_quad_reader.v 的 8 拍 (160ns) 数字消抖阈值 + 2 级同步延迟;
//       同时 2us 刷新间隔可无失真跟踪高达 500 kpps 的脉冲速率
//       (对应 dalpha ~ 785 rad/s), 覆盖本系统全部物理工况 (实测 < 10 rad/s)。
//
// -----------------------------------------------------------------------------
// 已知局限 / 简化之处 (如实记录, 不做粉饰):
//   S1. V_motor 采用 1ms 零阶保持 (ZOH), 且测量窗口与 DUT 内部 timer_1ms_cnt /
//       pwm_counter 未做硬同步 (二者均自 rst_n 释放起自由计数, 周期严格为
//       50000 / 2500 clk, 故窗口边界相对 PWM 载波相位固定, 占空比测量本身精确),
//       但相对 DUT 的 sample_done 采样时刻可能存在 <= 1ms 的对齐偏差,
//       等效于给闭环增加了至多 1 拍的控制延迟 (偏保守, 不会掩盖不稳定)。
//   S2. 未建模电枢电感 (电流瞬态)、齿轮背隙、传动柔性、电机转矩-转速非线性
//       饱和、温度漂移; 电磁力矩采用 dynamics.py 的一阶线性模型 km*V - b_emf*dalpha。
//   S3. 能耗制动与自由滑行在本模型中均等价于 V_motor = 0 (见上), 即滑行工况
//       下仍保留 b_emf 电气阻尼, 略偏乐观。
//   S4. 未生成编码器 Z 相脉冲 (enc_z 由 TB 恒接 0)。这是因为 j280_hw_top.v
//       以 USE_Z_INDEX(1) 例化 encoder_quad_reader, 任何 Z 上升沿都会把
//       pulse_count 清零, 从而摧毁转臂绝对位置与软限位保护 —— 详见复检报告
//       中对该项的分析, 属于待澄清的硬件风险, 本模型不主动触发它。
//   S5. 传感器噪声默认关闭 (sensor_noise_urad = 0), 以保证回归结果可复现;
//       置为非零值即启用 Box-Muller 高斯噪声 (config.py 的 sensor_noise_std
//       = 0.001 rad -> 1000 urad)。噪声仅叠加在摆杆 ADC 码值上, 编码器量化
//       噪声由脉冲取整天然产生。
//   S6. 不检测 NaN/Inf 传播, 仅以幅值窗口 (|theta|<=4 rad, |d*|<=1e4 rad/s)
//       输出 diverged 标志; 一旦 diverged 置位, 闭环结论一律无效。
// =============================================================================

`ifndef SYNTHESIS

`timescale 1ns / 1ps

module furuta_plant_model #(
    parameter integer CLK_FREQ_HZ     = 50_000_000, // 必须与 DUT 时钟一致
    parameter integer PHYS_RATE_HZ    = 5_000,      // 1/dt_phys = 5kHz (dt_phys = 0.2ms)
    parameter integer CTRL_RATE_HZ    = 1_000,      // 控制周期 1ms -> 每周期 5 次 RK4
    parameter integer OUT_REFRESH_DIV = 100,        // 传感器输出刷新分频 (clk 拍)
    parameter integer ADC_ZERO_CODE   = 2048,       // theta = 0 (倒立) 时的 ADC 码值
    parameter integer ADC_BITS        = 12,         // ADC 位数
    parameter integer ENC_CPR         = 4000,       // 编码器 4 倍频后每圈脉冲数
    parameter real    V_SUPPLY        = 12.0        // 电机驱动供电电压 (V)
)(
    input  wire               clk,                    // 与 DUT 同源 50MHz 时钟
    input  wire               rst_n,                  // 异步低电平复位 (与 DUT 同源)

    // ---- 执行机构: 直接取 DUT 的真实驱动引脚 ----
    input  wire               motor_in1,              // H 桥 IN1
    input  wire               motor_in2,              // H 桥 IN2
    input  wire               motor_stby,             // TB6612 STBY (0 -> 高阻, V=0)

    // ---- 外部扰动力矩注入接口 (单位: uN*m, 即 1e-6 N*m; 0.04 N*m -> 40000) ----
    input  wire signed [31:0] disturb_tau_pend_uNm,   // 作用于摆杆的推力力矩
    input  wire signed [31:0] disturb_tau_arm_uNm,    // 作用于转臂的扰动力矩

    // ---- 传感器噪声标准差 (单位: urad; 0 = 关闭, 保证回归可复现) ----
    input  wire signed [31:0] sensor_noise_urad,

    // ---- 初值装载 (电平有效: 高电平期间物理状态被钳位并冻结积分) ----
    input  wire               state_load,
    input  wire signed [31:0] init_alpha_q16,         // rad, Q12.16
    input  wire signed [31:0] init_dalpha_q16,        // rad/s, Q12.16
    input  wire signed [31:0] init_theta_q16,         // rad, Q12.16 (pi -> 205887)
    input  wire signed [31:0] init_dtheta_q16,        // rad/s, Q12.16

    // ---- 传感器反算输出 (直连 DUT 引脚) ----
    output reg  [ADC_BITS-1:0] raw_adc,               // 12-bit ADC 码值 (送 SPI 从机)
    output reg                 enc_a,                 // 正交 A 相 (送 DUT enc_a)
    output reg                 enc_b,                 // 正交 B 相 (送 DUT enc_b)

    // ---- 观测量输出 (仅供 TB 判读, 不参与闭环) ----
    output reg  signed [31:0]  theta_q16,             // 规范化后的摆杆角度 rad, Q16
    output reg  signed [31:0]  dtheta_q16,            // rad/s, Q16
    output reg  signed [31:0]  alpha_q16,             // rad (多圈不卷绕), Q16
    output reg  signed [31:0]  dalpha_q16,            // rad/s, Q16
    output reg  signed [31:0]  v_motor_mV,            // 本控制周期施加的电压 (mV)
    output reg  signed [31:0]  energy_rel_mJ,         // 相对倒立顶点机械能 (mJ)
    output reg                 phys_step,             // dt_phys 步进脉冲 (1 clk)
    output reg                 ctrl_tick,             // 控制周期脉冲 (1 clk, 1ms)
    output reg                 diverged,              // 数值发散标志
    output reg  signed [31:0]  rk4_step_count         // 累计 RK4 步数 (自检用)
);

    // -------------------------------------------------------------------------
    // 0. 派生时序常数
    // -------------------------------------------------------------------------
    localparam integer PHYS_CYCLES    = CLK_FREQ_HZ / PHYS_RATE_HZ;   // 10000 clk = 0.2ms
    localparam integer CTRL_CYCLES    = CLK_FREQ_HZ / CTRL_RATE_HZ;   // 50000 clk = 1.0ms
    localparam integer STEPS_PER_CTRL = PHYS_RATE_HZ / CTRL_RATE_HZ;  // 5
    localparam integer ADC_COUNTS     = 1 << ADC_BITS;                // 4096
    localparam real    DT_PHYS        = 1.0 / (PHYS_RATE_HZ * 1.0);   // 2.0e-4 s

    // -------------------------------------------------------------------------
    // 1. 物理常数 —— 逐条对应 config.py PendulumConfig (对标 J280 官方 FAQ 实测参数)
    // -------------------------------------------------------------------------
    // 机械结构: 水平旋转臂 (对标官方 FAQ: 长 15.2cm, 宽 3.6cm, 重 90g)
    localparam real L1           = 0.152;     // 转臂有效回转半径 (m)
    localparam real J1           = 0.000693;  // 转臂绕电机轴转动惯量 J1 = (1/3)*m1*L1^2 (kg*m^2)
    localparam real B1           = 0.0010;    // 转臂轴承黏性摩擦 (N*m*s/rad)
    localparam real COULOMB_TAU  = 0.0020;    // 转臂轴承库仑摩擦力矩 (N*m)
    // 机械结构: 垂直摆杆 (对标官方 FAQ: 长 15cm, 宽 3.4cm, 重 90g)
    localparam real L2_FULL      = 0.150;     // 摆杆全长 (m)  (仅存档, 方程未直接用)
    localparam real LC2          = 0.075;     // 摆杆转轴到质心距离 l2 = L2 / 2 (m)
    localparam real M2           = 0.090;     // 摆杆质量 (kg)
    localparam real JP           = 0.000675;  // 摆杆绕转轴转动惯量 Jp = (1/3)*m2*L2^2 (kg*m^2)
    localparam real B2           = 0.00010;   // 摆杆转轴阻尼 (N*m*s/rad)
    localparam real G_ACC        = 9.81;      // 重力加速度 (m/s^2)
    // 执行机构与直流电机
    localparam real V_MAX        = 12.0;      // 供电电压 (V)
    localparam real R_M          = 4.0;       // 电枢电阻 (Ohm)
    localparam real K_T          = 0.050;     // 转矩常数 (N*m/A)
    localparam real K_B          = 0.050;     // 反电动势常数 (V*s/rad)
    localparam real GEAR_RATIO   = 1.0;       // 减速比
    localparam real MOTOR_DEAD   = 0.25;      // 电机死区电压 (V)
    // __post_init__ 复合常数
    localparam real J0           = J1 + M2 * L1 * L1;             // 0.002772
    localparam real KM           = (GEAR_RATIO * K_T) / R_M;      // 0.0125
    localparam real B_EMF        = (GEAR_RATIO * K_T * K_B) / R_M;// 0.000625
    // 数学常数
    localparam real PI       = 3.14159265358979323846;
    localparam real TWO_PI   = 6.28318530717958647692;
    localparam real RAD2DEG  = 57.29577951308232087680;

    // -------------------------------------------------------------------------
    // 2. 连续状态 (real, 仅行为级)
    // -------------------------------------------------------------------------
    real alpha;      // 转臂角度 (rad), 多圈连续
    real dalpha;     // 转臂角速度 (rad/s)
    real theta;      // 摆杆角度 (rad), 每步规范化至 [-pi, pi]
    real dtheta;     // 摆杆角速度 (rad/s)
    real V_motor;    // 本控制周期施加的电枢电压 (V), 零阶保持
    real dist_pend;  // 摆杆扰动力矩 (N*m)
    real dist_arm;   // 转臂扰动力矩 (N*m)

    integer rk4_steps_i;  // 累计 RK4 步数 (由 rk4_step 递增, 输出镜像见第 9 节)

    // -------------------------------------------------------------------------
    // 3. 数学辅助函数
    // -------------------------------------------------------------------------
    // 绝对值 (Verilog-2001 无 $abs, 自行实现)
    function real fabs_r;
        input real x;
        begin
            fabs_r = (x < 0.0) ? -x : x;
        end
    endfunction

    // tanh: 按任务要求用 exp 实现, 并对输入做 +/-20 钳位防止 $exp 溢出
    function real ftanh;
        input real x;
        real xx, ep, en;
        begin
            xx = x;
            if (xx >  20.0) xx =  20.0;
            if (xx < -20.0) xx = -20.0;
            ep    = $exp(xx);
            en    = $exp(-xx);
            ftanh = (ep - en) / (ep + en);
        end
    endfunction

    // normalize_theta: 规范化至 [-pi, pi] (等价 dynamics.py 的取模实现,
    // 用有界循环替代 $floor 以提高工具可移植性; guard 防止异常值导致挂死)
    function real norm_theta;
        input real x;
        real    r;
        integer guard;
        begin
            r     = x;
            guard = 0;
            while ((r > PI) && (guard < 64)) begin
                r     = r - TWO_PI;
                guard = guard + 1;
            end
            while ((r < -PI) && (guard < 128)) begin
                r     = r + TWO_PI;
                guard = guard + 1;
            end
            norm_theta = r;
        end
    endfunction

    // 四舍五入 (Verilog $rtoi 为向零截断, 需按符号补偿 0.5), 并对超界值钳位
    function integer round_real;
        input real x;
        real xc;
        begin
            round_real = 0;
            xc = x;
            if (xc >  1.0e9) xc =  1.0e9;
            if (xc < -1.0e9) xc = -1.0e9;
            if (xc >= 0.0) round_real = $rtoi(xc + 0.5);
            else           round_real = $rtoi(xc - 0.5);
        end
    endfunction

    // 均匀分布 (0,1) 与标准正态分布 (Box-Muller), 仅在 sensor_noise_urad != 0 时调用
    // 注: IEEE 1364-2001 要求 function 至少有一个 input, 故保留哑元 dummy
    integer noise_seed;
    function real urand01;
        input real dummy;
        integer r;
        begin
            r       = $random(noise_seed);
            urand01 = (r + 2147483648.0) / 4294967296.0;
            if (urand01 < 1.0e-12)      urand01 = 1.0e-12;
            if (urand01 > 0.9999999999) urand01 = 0.9999999999;
        end
    endfunction

    // 返回 sigma * N(0,1)
    function real gauss_scaled;
        input real sigma;
        real u1, u2;
        begin
            u1           = urand01(0.0);
            u2           = urand01(0.0);
            gauss_scaled = sigma * $sqrt(-2.0 * $ln(u1)) * $cos(TWO_PI * u2);
        end
    endfunction

    // -------------------------------------------------------------------------
    // 4. dynamics.py derivatives() 的逐式移植
    //    返回 d/dt [alpha, dalpha, theta, dtheta]
    // -------------------------------------------------------------------------
    task plant_deriv;
        input  real s_a, s_da, s_th, s_dth, s_vm;
        output real d_a, d_da, d_th, d_dth;
        real th_n, sin_th, cos_th, sin_2th;
        real v_eff, tau_motor, tau_coulomb;
        real m11, m12, m22, f1, f2, det_m;
        begin
            // 角度规范化与三角函数
            th_n    = norm_theta(s_th);
            sin_th  = $sin(th_n);
            cos_th  = $cos(th_n);
            sin_2th = $sin(2.0 * th_n);

            // (1) 电机电磁力矩: 供电饱和 clip(V, +/-V_max) + 死区 + 反电动势 + 扰动
            v_eff = s_vm;
            if (v_eff >  V_MAX) v_eff =  V_MAX;
            if (v_eff < -V_MAX) v_eff = -V_MAX;
            if (fabs_r(v_eff) < MOTOR_DEAD) v_eff = 0.0;
            tau_motor = KM * v_eff - B_EMF * s_da + dist_arm;

            // 轴承库仑静摩擦 (平滑连续可微模型, 避免数值抖振)
            if (COULOMB_TAU > 0.0) tau_coulomb = COULOMB_TAU * ftanh(50.0 * s_da);
            else                   tau_coulomb = 0.0;

            // (2) 广义质量惯性矩阵 M(q)  (M21 = M12, 对称)
            m11 = J0 + JP * (sin_th * sin_th);
            m12 = M2 * L1 * LC2 * cos_th;
            m22 = JP;

            // (3) 广义力: 科氏/向心 + 重力 + 黏性 + 库仑 + 电磁 + 外扰
            f1 = tau_motor
               - B1 * s_da
               - tau_coulomb
               - JP * sin_2th * s_da * s_dth
               + M2 * L1 * LC2 * sin_th * (s_dth * s_dth);

            f2 = M2 * G_ACC * LC2 * sin_th
               + 0.5 * JP * sin_2th * (s_da * s_da)
               - B2 * s_dth
               + dist_pend;

            // (4) [ddalpha, ddtheta]^T = M^-1 * [F1, F2]^T
            det_m = m11 * m22 - m12 * m12;
            d_a   = s_da;
            d_da  = (m22 * f1 - m12 * f2) / det_m;
            d_th  = s_dth;
            d_dth = (-m12 * f1 + m11 * f2) / det_m;
        end
    endtask

    // -------------------------------------------------------------------------
    // 5. dynamics.py step_rk4() 的逐式移植 (enable_delay = False, 与 config 一致)
    // -------------------------------------------------------------------------
    task rk4_step;
        input real vm;
        input real dt;
        real k1a, k1b, k1c, k1d;
        real k2a, k2b, k2c, k2d;
        real k3a, k3b, k3c, k3d;
        real k4a, k4b, k4c, k4d;
        begin
            plant_deriv(alpha, dalpha, theta, dtheta, vm,
                        k1a, k1b, k1c, k1d);
            plant_deriv(alpha + 0.5*dt*k1a, dalpha + 0.5*dt*k1b,
                        theta + 0.5*dt*k1c, dtheta + 0.5*dt*k1d, vm,
                        k2a, k2b, k2c, k2d);
            plant_deriv(alpha + 0.5*dt*k2a, dalpha + 0.5*dt*k2b,
                        theta + 0.5*dt*k2c, dtheta + 0.5*dt*k2d, vm,
                        k3a, k3b, k3c, k3d);
            plant_deriv(alpha + dt*k3a, dalpha + dt*k3b,
                        theta + dt*k3c, dtheta + dt*k3d, vm,
                        k4a, k4b, k4c, k4d);

            alpha  = alpha  + (dt/6.0) * (k1a + 2.0*k2a + 2.0*k3a + k4a);
            dalpha = dalpha + (dt/6.0) * (k1b + 2.0*k2b + 2.0*k3b + k4b);
            theta  = norm_theta(theta + (dt/6.0) * (k1c + 2.0*k2c + 2.0*k3c + k4c));
            dtheta = dtheta + (dt/6.0) * (k1d + 2.0*k2d + 2.0*k3d + k4d);

            rk4_steps_i = rk4_steps_i + 1;
        end
    endtask

    // -------------------------------------------------------------------------
    // 6. 时序基准: 0.2ms 物理步与 1ms 控制周期
    //    (plant 的推进完全由时钟驱动的 always 块完成, 不使用任何 #延时 等待物理过程)
    // -------------------------------------------------------------------------
    reg [15:0] phys_cnt;
    reg [3:0]  sub_cnt;

    wire phys_now = (rst_n === 1'b1) && (phys_cnt == PHYS_CYCLES - 1);
    wire ctrl_now = phys_now && (sub_cnt == STEPS_PER_CTRL - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phys_cnt  <= 16'd0;
            sub_cnt   <= 4'd0;
            phys_step <= 1'b0;
            ctrl_tick <= 1'b0;
        end else begin
            phys_step <= phys_now;
            ctrl_tick <= ctrl_now;
            if (phys_now) begin
                phys_cnt <= 16'd0;
                if (ctrl_now) sub_cnt <= 4'd0;
                else          sub_cnt <= sub_cnt + 4'd1;
            end else begin
                phys_cnt <= phys_cnt + 16'd1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 7. 执行机构正算: 1ms 窗口内 IN1/IN2 高电平计数 -> 平均占空比 -> 电压
    // -------------------------------------------------------------------------
    reg signed [31:0] in1_hi_cnt;
    reg signed [31:0] in2_hi_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in1_hi_cnt <= 32'sd0;
            in2_hi_cnt <= 32'sd0;
        end else if (ctrl_now) begin
            in1_hi_cnt <= 32'sd0;
            in2_hi_cnt <= 32'sd0;
        end else begin
            if (motor_in1 && motor_stby) in1_hi_cnt <= in1_hi_cnt + 32'sd1;
            if (motor_in2 && motor_stby) in2_hi_cnt <= in2_hi_cnt + 32'sd1;
        end
    end

    // -------------------------------------------------------------------------
    // 8. 物理状态推进 + 传感器输出刷新 (单一 always 块，修复 H2 竞态)
    //
    // 已发出的正交脉冲计数 (修复 H1)。跨 refresh_outputs 调用保持，并在
    // 复位与 state_load 时同步到当前 alpha。声明必须位于本节之前，因为
    // 本节的 always 块与第 9 节的 refresh_outputs 都会访问它。
    // (out_cnt 的声明仍留在本节末尾、always 块之前。)
    // -------------------------------------------------------------------------
    integer pulse_emitted;

    // 原实现分成两个 always 块: 本块用阻塞赋值写 alpha/theta 等 real 状态，
    // 另一块 (posedge clk or negedge rst_n) 调用 refresh_outputs 读它们。
    // Verilog 不定义同一事件触发的两个 always 块的相对执行顺序，因此
    // refresh_outputs 可能读到 RK4 之前或之后的值 —— 传感器输出随机滞后
    // 一个物理步长 (0.2ms)。
    //
    // 且命中率是 100% 而非偶发: 两个计数器同源于 rst_n，
    //   phys_now 为真  <=> clk 序号 k = 0 (mod PHYS_CYCLES=10000)
    //   out_cnt 到期   <=> k = 0 (mod OUT_REFRESH_DIV=100)
    // k = 0 (mod 10000) 蕴含 k = 0 (mod 100)，故每一次 RK4 积分都恰好落在
    // 一次传感器刷新的同一个时钟沿上。
    //
    // 后果: 仿真结果依赖仿真器调度顺序与优化选项 (-voptargs=+acc 的开关就会
    // 改变它)。TEST B 的判据余量仅 0.303° (1.4%)，足以被这种非确定性翻转，
    // 对一个用来出验收结论的平台是不可接受的。
    //
    // 修法: 合并为单一 always 块，顺序固定为「先推进状态，再刷新传感器输出」。
    // 代价: 复位由异步变为同步 (对本 TB 无影响，rst_n 在时钟沿外保持有效)。
    // -------------------------------------------------------------------------
    reg [15:0] out_cnt;
    always @(posedge clk) begin
        if (rst_n !== 1'b1) begin
            alpha   = 0.0;
            dalpha  = 0.0;
            theta   = PI;      // 默认下垂稳定点, 与 dynamics.py 的初始 state 一致
            dtheta  = 0.0;
            V_motor = 0.0;
            rk4_steps_i   = 0;
            pulse_emitted = 0;
            out_cnt       <= 16'd0;
            raw_adc       <= ADC_ZERO_CODE;
            enc_a         <= 1'b0;
            enc_b         <= 1'b0;
            theta_q16     <= 32'sd0;
            dtheta_q16    <= 32'sd0;
            alpha_q16     <= 32'sd0;
            dalpha_q16    <= 32'sd0;
            v_motor_mV    <= 32'sd0;
            energy_rel_mJ <= 32'sd0;
            diverged      <= 1'b0;
            rk4_step_count<= 32'sd0;
        end else if (state_load) begin
            // 初值装载并冻结积分 (供 TB 摆位: 标定用倒立, 起摆用下垂)
            alpha   = init_alpha_q16  / 65536.0;
            dalpha  = init_dalpha_q16 / 65536.0;
            theta   = init_theta_q16  / 65536.0;
            dtheta  = init_dtheta_q16 / 65536.0;
            V_motor = 0.0;
            // 装载后必须同步 pulse_emitted，否则单步逼近会从旧位置一路追赶，
            // 产生大量虚假正交脉冲并污染 DUT 的 pulse_count。
            pulse_emitted = round_real(alpha * ENC_CPR / TWO_PI);
            refresh_outputs;
        end else begin
            dist_pend = disturb_tau_pend_uNm * 1.0e-6;
            dist_arm  = disturb_tau_arm_uNm  * 1.0e-6;

            if (ctrl_now) begin
                V_motor = V_SUPPLY * (in1_hi_cnt - in2_hi_cnt) / (CTRL_CYCLES * 1.0);
                if (!motor_stby) V_motor = 0.0;
            end

            if (phys_now) rk4_step(V_motor, DT_PHYS);

            // 传感器输出刷新: 必须在状态推进之后，顺序由此确定，不再有竞态
            if (out_cnt == OUT_REFRESH_DIV - 1) begin
                out_cnt <= 16'd0;
                refresh_outputs;
            end else begin
                out_cnt <= out_cnt + 16'd1;
            end
        end

        // 数值发散检测: 用 "非(在窗口内)" 形式, 使 NaN 也能被捕获
        if (!( (theta <= 4.0) && (theta >= -4.0) &&
               (fabs_r(dalpha) <= 1.0e4) && (fabs_r(dtheta) <= 1.0e4) &&
               (fabs_r(alpha)  <= 3.0e4) ))
            diverged <= 1'b1;
    end

    // -------------------------------------------------------------------------
    // 9. 传感器反算与观测量刷新 (每 OUT_REFRESH_DIV 拍一次)
    // -------------------------------------------------------------------------
    // (pulse_emitted 与 out_cnt 的声明已上移至第 8 节之前 —— Verilog 要求
    //  变量先声明后使用，而第 8 节的 always 块已引用 pulse_emitted。)

    task refresh_outputs;
        real    th_n, adc_r, pulse_r, th_out, al_out;
        integer adc_i, pulse_i, s_i;
        real    e_rel;
        begin
            th_n = norm_theta(theta);

            // --- 摆杆角度 -> 12-bit ADC 码值 ---
            th_out = th_n;
            if (th_out >  4.0) th_out =  4.0;
            if (th_out < -4.0) th_out = -4.0;
            adc_r  = ADC_ZERO_CODE + th_out * ADC_COUNTS / TWO_PI;
            if (sensor_noise_urad != 32'sd0)
                adc_r = adc_r + gauss_scaled((sensor_noise_urad * 1.0e-6) * ADC_COUNTS / TWO_PI);
            adc_i = round_real(adc_r) % ADC_COUNTS;
            if (adc_i < 0) adc_i = adc_i + ADC_COUNTS;
            raw_adc <= adc_i;

            // --- 转臂角度 -> 正交 A/B 电平 (单步逼近，修复 H1) ---
            // 原实现直接把 round(alpha*ENC_CPR/2pi) 映射为 A/B 状态。但 alpha 只在
            // phys_now (每 PHYS_CYCLES=10000 clk = 200us) 由 RK4 更新一次，
            // 因此一个 RK4 步内的脉冲增量为
            //     dpulse = |dalpha| * dt_phys * ENC_CPR/(2pi) = |dalpha| * 0.12732
            // 当 |dalpha| > 7.854 rad/s 时 dpulse > 1，pulse_i 一次跳 2 以上，
            // 使 encoder_quad_reader 的鉴相 case 命中非法组合 (如 4'b00_11) 而
            // 走 default: count_step = 0 —— 静默丢计数，无任何告警。
            // HIL 实测丢失 16 脉冲 = 1.42°，造成 DUT 的 alpha 读数系统性偏大，
            // 是定点伺服稳态误差的主因。
            //
            // 修法: 维护 pulse_emitted，每次刷新最多前进/后退一个状态，
            // 保证 encoder 永远看到合法的正交序列 00->10->11->01。
            // 余量核算: 刷新周期 OUT_REFRESH_DIV=100 clk = 2us，单步逼近上限
            //   = 1 pulse / 2us = 5e5 pulse/s = 785 rad/s；
            //   起摆期实测峰值仅 7.6 rad/s，余量约 100 倍。
            al_out  = alpha;
            if (al_out >  30000.0) al_out =  30000.0;
            if (al_out < -30000.0) al_out = -30000.0;
            pulse_r = al_out * ENC_CPR / TWO_PI;
            pulse_i = round_real(pulse_r);
            if      (pulse_emitted < pulse_i) pulse_emitted = pulse_emitted + 1;
            else if (pulse_emitted > pulse_i) pulse_emitted = pulse_emitted - 1;
            s_i     = pulse_emitted % 4;
            if (s_i < 0) s_i = s_i + 4;
            enc_a <= (s_i == 1) || (s_i == 2);
            enc_b <= (s_i == 2) || (s_i == 3);

            // --- 观测量 Q16 镜像 ---
            theta_q16  <= round_real(th_out * 65536.0);
            dtheta_q16 <= round_real(dtheta * 65536.0);
            alpha_q16  <= round_real(al_out * 65536.0);
            dalpha_q16 <= round_real(dalpha * 65536.0);
            v_motor_mV <= round_real(V_motor * 1000.0);

            // 相对倒立顶点的机械能 E = 0.5*Jp*dtheta^2 + m2*g*l2*(cos(th)-1)
            e_rel         = 0.5 * JP * (dtheta * dtheta)
                          + M2 * G_ACC * LC2 * ($cos(th_n) - 1.0);
            energy_rel_mJ <= round_real(e_rel * 1000.0);

            rk4_step_count <= rk4_steps_i;
        end
    endtask

    // (原第 9 节的第二个 always 块已合并至上方第 8 节，以消除对 real 状态的
    //  未定义顺序读写竞态 —— 修复 H2。out_cnt 声明也随之上移。)

    // -------------------------------------------------------------------------
    // 10. 自检: 上电即验证传感器反算映射的三个关键点与编码器鉴相序列
    // -------------------------------------------------------------------------
    // 可通过 +PLANT_SELFTEST=0 关闭 (默认开启, 输出 4 行自检信息)
    integer selftest_en;
    real    st_adc_r;
    integer st_adc_i, st_pulse_i, st_s, st_i;
    reg     st_a, st_b;

    task map_adc;   // theta -> ADC 码值
        input real th;
        integer ai;
        begin
            st_adc_r = ADC_ZERO_CODE + th * ADC_COUNTS / TWO_PI;
            ai       = round_real(st_adc_r) % ADC_COUNTS;
            if (ai < 0) ai = ai + ADC_COUNTS;
            st_adc_i = ai;
        end
    endtask

    task map_ab;    // alpha -> (A,B)
        input real al;
        begin
            st_pulse_i = round_real(al * ENC_CPR / TWO_PI);
            st_s       = st_pulse_i % 4;
            if (st_s < 0) st_s = st_s + 4;
            st_a = (st_s == 1) || (st_s == 2);
            st_b = (st_s == 2) || (st_s == 3);
        end
    endtask

    initial begin
        selftest_en = 1;
        if ($value$plusargs("PLANT_SELFTEST=%d", st_i)) selftest_en = st_i;
        noise_seed  = 32'h1234_5678;
        dist_pend   = 0.0;
        dist_arm    = 0.0;

        if (selftest_en) begin
            $display("  [PLANT-SELFTEST] dt_phys=%.6f s, steps/ctrl=%0d, phys_cycles=%0d, ctrl_cycles=%0d",
                     DT_PHYS, STEPS_PER_CTRL, PHYS_CYCLES, CTRL_CYCLES);

            map_adc(0.0);
            $display("  [PLANT-SELFTEST] theta=0 (inverted)  -> raw_adc=%0d (expect %0d)",
                     st_adc_i, ADC_ZERO_CODE);
            if (st_adc_i !== ADC_ZERO_CODE)
                $display("  [PLANT-SELFTEST] *** FAIL: inverted point mapping ***");

            map_adc(PI);
            $display("  [PLANT-SELFTEST] theta=+pi (hanging) -> raw_adc=%0d (expect 0)", st_adc_i);
            if (st_adc_i !== 0)
                $display("  [PLANT-SELFTEST] *** FAIL: +pi mapping ***");

            map_adc(-PI);
            $display("  [PLANT-SELFTEST] theta=-pi (hanging) -> raw_adc=%0d (expect 0)", st_adc_i);
            if (st_adc_i !== 0)
                $display("  [PLANT-SELFTEST] *** FAIL: -pi mapping ***");

            // 编码器正转序列: alpha = 0, 1/4, 2/4, 3/4, 4/4 个脉冲步
            $write("  [PLANT-SELFTEST] quadrature sequence for increasing alpha: ");
            for (st_i = 0; st_i < 5; st_i = st_i + 1) begin
                map_ab(st_i * TWO_PI / (ENC_CPR * 1.0));
                $write("%b%b ", st_a, st_b);
            end
            $display("");
            $display("  [PLANT-SELFTEST]   expect 00 10 11 01 00 (matches encoder_quad_reader +1 table)");

            // 反向
            $write("  [PLANT-SELFTEST] quadrature sequence for decreasing alpha: ");
            for (st_i = 0; st_i < 5; st_i = st_i + 1) begin
                map_ab(-st_i * TWO_PI / (ENC_CPR * 1.0));
                $write("%b%b ", st_a, st_b);
            end
            $display("");
            $display("  [PLANT-SELFTEST]   expect 00 01 11 10 00 (matches encoder_quad_reader -1 table)");
        end
    end

endmodule

`endif // SYNTHESIS
