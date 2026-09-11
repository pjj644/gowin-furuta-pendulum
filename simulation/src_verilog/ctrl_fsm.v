// =============================================================================
// 全国大学生嵌入式芯片与系统设计竞赛 - 选题一 (J280 姿态控制系统竞赛套件)
// 模块名称: ctrl_fsm
// 目标芯片: 高云 GW2A-LV55PG484C8/I7
// 功能描述: 旋转倒立摆系统主控制状态机 (解决起摆与跌落保护的结构性冲突)
//           对标 Python 状态机模型 (simulation/controller.py):
//           STATE_HANGING (0): 下垂静止态 (待机)
//           STATE_SWINGUP (1): 能量泵起摆态 (放行大角度动力, 屏蔽跌落自锁)
//           STATE_BALANCE (2): 倒立自平衡与轨迹跟踪态 (小角度精准控制)
//           STATE_PROTECT (3): 急停/软限位超限/严重故障自锁态
//           双重捕获判据: |theta| < 22度 && |dtheta| < 4.0 rad/s (D7 整改)
// =============================================================================

module ctrl_fsm (
    input  wire                   clk,                  // 50MHz 系统时钟
    input  wire                   rst_n,                // 异步复位 (低有效)
    input  wire                   calc_en,              // 1ms 控制节拍脉冲
    input  wire                   motor_en_sw,          // 拨码开关电机使能信号
    input  wire                   calib_done,           // 摆杆垂直零位标定完成标志
    input  wire                   soft_limit_err,       // 转臂多圈超限保护报警 (D3)
    input  wire signed [31:0]     theta_err_q16,        // 摆杆偏角 (rad, Q12.16)
    input  wire signed [31:0]     dtheta_q16,           // 摆杆角速度 (rad/s, Q12.16)

    input  wire signed [15:0]     pwm_swing_duty,       // 来自起摆核的 PWM 占空比
    input  wire signed [15:0]     pwm_lqr_duty,         // 来自 LQI 平衡核的 PWM 占空比

    output reg  [1:0]             current_state,        // 当前主状态
    output reg  signed [15:0]     final_pwm_duty,       // 仲裁后的最终电机输出占空比
    output reg                    reset_integral_pulse, // 切入平衡瞬间积分器清零脉冲
    output reg                    traj_sync_pulse,      // 切入平衡瞬间轨迹相位同步脉冲 (F8)
    output wire                   is_in_balance_zone,   // 是否处于平衡区指示
    output wire                   lqr_en,               // LQR 控制使能
    output wire                   swing_en,             // 起摆控制使能
    output wire                   motor_active          // 电机整体使能状态
);

    // 状态编码
    localparam [1:0] STATE_HANGING = 2'd0;
    localparam [1:0] STATE_SWINGUP = 2'd1;
    localparam [1:0] STATE_BALANCE = 2'd2;
    localparam [1:0] STATE_PROTECT = 2'd3;

    // 状态切换阈值 (Q12.16)
    localparam signed [31:0] BAL_ANG_THRESH = 32'sd25166;   // 22 度 (0.384 rad * 65536)
    localparam signed [31:0] BAL_OMG_THRESH = 32'sd262144;  // 4.0 rad/s * 65536
    localparam signed [31:0] FALL_THRESH    = 32'sd51472;   // 45 度 (0.785 rad * 65536)

    // 绝对值计算
    wire signed [31:0] abs_th  = (theta_err_q16 < 0) ? (-theta_err_q16) : theta_err_q16;
    wire signed [31:0] abs_dth = (dtheta_q16 < 0)    ? (-dtheta_q16)    : dtheta_q16;

    // 倒立自平衡区双重判定 (角度 < 22度 且 角速度 < 4.0 rad/s, 杜绝高速冲过平衡区 D7)
    wire can_enter_balance = (abs_th < BAL_ANG_THRESH) && (abs_dth < BAL_OMG_THRESH);
    assign is_in_balance_zone = (abs_th < BAL_ANG_THRESH);

    // 跌落判定: 在自平衡态下若倾角超过 45度 则判定为跌落
    wire is_fall_down = (abs_th > FALL_THRESH);

    // 控制使能与状态信号
    assign lqr_en       = (current_state == STATE_BALANCE);
    assign swing_en     = (current_state == STATE_SWINGUP);
    assign motor_active = motor_en_sw && (current_state != STATE_HANGING) && (current_state != STATE_PROTECT);

    // 状态机时序转移与控制量多路选择
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state        <= STATE_HANGING;
            final_pwm_duty       <= 16'sd0;
            reset_integral_pulse <= 1'b0;
            traj_sync_pulse      <= 1'b0;
        end else if (!motor_en_sw) begin
            // 拨码开关拉低时无论何种状态立即停机归位
            current_state        <= STATE_HANGING;
            final_pwm_duty       <= 16'sd0;
            reset_integral_pulse <= 1'b0;
            traj_sync_pulse      <= 1'b0;
        end else if (soft_limit_err) begin
            // 转臂多圈超限保护自锁 (D3)
            current_state        <= STATE_PROTECT;
            final_pwm_duty       <= 16'sd0;
            reset_integral_pulse <= 1'b0;
            traj_sync_pulse      <= 1'b0;
        end else if (calc_en) begin
            case (current_state)
                STATE_HANGING: begin
                    reset_integral_pulse <= 1'b0;
                    traj_sync_pulse      <= 1'b0;
                    if (motor_en_sw && calib_done) begin
                        // 零位已就绪且开关已打开: 启动起摆并注入 3.0V 初始扰动脉冲 (F6: 3.0*1000/12 = 250)
                        current_state  <= STATE_SWINGUP;
                        final_pwm_duty <= 16'sd250;
                    end else begin
                        final_pwm_duty <= 16'sd0;
                    end
                end

                STATE_SWINGUP: begin
                    if (can_enter_balance) begin
                        // 满足双重捕获窗口: 立即无缝切入 LQI 自平衡态
                        current_state        <= STATE_BALANCE;
                        final_pwm_duty       <= pwm_lqr_duty;
                        reset_integral_pulse <= 1'b1; // 清零历史积分防止积分冲量超调
                        traj_sync_pulse      <= 1'b1; // 同步正弦轨迹发生器初始相位 (F8)
                    end else begin
                        // 起摆中: 放行大角度动力输出 (彻底解开原顶层的跌落死锁!)
                        current_state        <= STATE_SWINGUP;
                        final_pwm_duty       <= pwm_swing_duty;
                        reset_integral_pulse <= 1'b0;
                        traj_sync_pulse      <= 1'b0;
                    end
                end

                STATE_BALANCE: begin
                    reset_integral_pulse <= 1'b0;
                    traj_sync_pulse      <= 1'b0;
                    if (is_fall_down) begin
                        // 极端推力或失稳倾倒 (>45度): 自动回退至起摆态重新拉起
                        current_state  <= STATE_SWINGUP;
                        final_pwm_duty <= 16'sd0;
                    end else begin
                        // 正常自平衡与转臂定点/正弦轨迹跟踪
                        current_state  <= STATE_BALANCE;
                        final_pwm_duty <= pwm_lqr_duty;
                    end
                end

                STATE_PROTECT: begin
                    // F5 严重缺陷修复: 软限位故障消除后 (操作者长按 KEY2 清零编码器) 自动恢复至待机起摆态!
                    if (!soft_limit_err) begin
                        current_state <= STATE_HANGING;
                    end
                    final_pwm_duty       <= 16'sd0;
                    reset_integral_pulse <= 1'b0;
                    traj_sync_pulse      <= 1'b0;
                end

                default: begin
                    current_state        <= STATE_HANGING;
                    final_pwm_duty       <= 16'sd0;
                    reset_integral_pulse <= 1'b0;
                    traj_sync_pulse      <= 1'b0;
                end
            endcase
        end else begin
            reset_integral_pulse <= 1'b0;
            traj_sync_pulse      <= 1'b0;
        end
    end

endmodule
