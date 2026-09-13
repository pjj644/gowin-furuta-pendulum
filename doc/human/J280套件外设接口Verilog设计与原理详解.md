# J280 姿态控制系统竞赛套件 —— 外设接口 Verilog 设计与硬件原理解析全手册

> **适用竞赛**：全国大学生嵌入式芯片与系统设计竞赛（FPGA 创新设计赛道 —— 选题一：基于 FPGA 的实时姿态控制系统）  
> **核心板卡**：高云半导体晨熙家族第一代 FPGA（`GW2A-LV55PG484C8/I7`，PBGA484，Speed Grade 8）  
> **配套机械**：J280 姿态控制系统旋转倒立摆套件  
> **读者定位**：初次接触电机驱动、光电正交编码器与高精度角度传感器的参赛队员与 FPGA 开发者  
> **对应源码**：
> - [`motor_pwm_driver.v`](../../furuta_lqr_ctrl/src/motor_pwm_driver.v) —— 直流电机 20kHz 高频 PWM 与 H 桥驱动发生器
> - [`encoder_quad_reader.v`](../../furuta_lqr_ctrl/src/encoder_quad_reader.v) —— 电机编码器 4 倍频消抖滤波测速测角模块
> - [`angle_sensor_reader.v`](../../furuta_lqr_ctrl/src/angle_sensor_reader.v) —— 摆杆 12-bit SPI ADC / 磁编码器零点校准与 3 拍流水解卷绕模块
> - [`j280_hw_top.v`](../../furuta_lqr_ctrl/src/j280_hw_top.v) —— 硬件系统顶层集成互联、采样同步与安全保护模块

---

## 目录
1. [倒立摆硬件外设体系全景拓扑](#一倒立摆硬件外设体系全景拓扑)
2. [外设一：直流有刷电机与 H 桥驱动原理与 Verilog 实现 (`motor_pwm_driver.v`)](#二外设一直流有刷电机与-h-桥驱动原理与-verilog-实现)
3. [外设二：正交增量式光电编码器原理与 Verilog 实现 (`encoder_quad_reader.v`)](#三外设二正交增量式光电编码器原理与-verilog-实现)
4. [外设三：摆杆角度传感器与 12-bit ADC 原理与 Verilog 实现 (`angle_sensor_reader.v`)](#四外设三摆杆角度传感器与-12-bit-adc-原理与-verilog-实现)
5. [系统级整合：`j280_hw_top.v` 顶层互联、采样同步与安全闭环](#五系统级整合j280_hw_topv-顶层互联采样同步与安全闭环)
6. [Q12.16 定点数与实际物理量量纲换算完整字典](#六q1216-定点数与实际物理量量纲换算完整字典)

---

## 一、倒立摆硬件外设体系全景拓扑

在旋转倒立摆控制系统中，主控芯片 **高云 GW2A-LV55** 扮演着“超级大脑”的角色。它需要与套件上的三大核心电控元器件进行微秒级的实时双向数据吞吐：

```mermaid
flowchart TB
    subgraph J280_PHYS["J280 物理机械运动机构"]
        ARM["水平转臂 (Arm)"]
        PEND["垂直摆杆 (Pendulum)"]
    end

    subgraph SENSORS["传感器信号链 (输入)"]
        ENC_HW["电机尾部光电编码器<br/>(A 相 / B 相 / Z 相)"]
        ADC_HW["摆杆转轴角度传感器<br/>(精密电位器/磁阻 + 12-bit ADC)"]
    end

    subgraph GW2A["高云 GW2A-LV55 FPGA 内部逻辑 (50MHz 系统时钟)"]
        subgraph MOD_IN["外设采集与信号处理模块"]
            MOD_ENC["encoder_quad_reader.v<br/>8级数字消抖 (160ns) + 4倍频<br/>1ms差分测速 + IIR平滑滤波<br/>32位脉冲计数器 + Z相支持"]
            MOD_ANG["angle_sensor_reader.v<br/>硬件 SPI 主机 (2.5MHz)<br/>360°去卷绕 + 零点扣除<br/>3拍流水差分测速 + sample_done"]
        end

        subgraph CORE["核心控制算法链"]
            HEARTBEAT["1ms (1000Hz) 控制心跳定时器<br/>TIMER_1MS_LIMIT = 50000"]
            FSM["ctrl_fsm.v<br/>四态状态机仲裁<br/>HANGING / SWINGUP / BALANCE / PROTECT"]
            SWING["swing_up_ctrl.v<br/>Lyapunov 能量泵起摆核<br/>64点 cos LUT + 4段折线 tanh"]
            LQR["furuta_lqr_ctrl.v<br/>5状态 LQI 最优姿态自平衡核<br/>Q10增益定标 + 18位DSP乘法<br/>4级流水线, 80ns 确定性极速计算"]
            TRAJ["traj_gen.v<br/>4模式轨迹发生器<br/>128点正弦 LUT + 0.05°/ms 平滑斜坡<br/>★ R1-A 3拍速度退饱和算法"]
            INT_LOGIC["j280_hw_top.v 内部逻辑<br/>LQI 积分器抗饱和 (±10°外冻结)<br/>无乘除移位累加 + ±720°软限位"]
        end

        subgraph MOD_OUT["执行驱动模块"]
            MOD_PWM["motor_pwm_driver.v<br/>20kHz 载波 + 250拍(5us)硬件死区<br/>双载波周期影子寄存器<br/>STBY = motor_en | brake_mode"]
        end
    end

    subgraph ACTUATOR["执行机构 (输出)"]
        DRIVER["H 桥驱动芯片 (TB6612FNG / A4950)"]
        MOTOR["直流高速/永磁减速电机"]
    end

    %% 硬件物理连接
    ARM -.->|带动旋转| ENC_HW
    PEND -.->|角度偏转| ADC_HW
    ENC_HW -->|A/B/Z 正交 TTL 电平| MOD_ENC
    ADC_HW -->|SPI 总线 (MISO)| MOD_ANG

    HEARTBEAT -->|1ms calc_en 脉冲| MOD_ANG
    MOD_ANG -->|sample_done 同步采样完成| MOD_ENC
    MOD_ANG -->|sample_done 同步采样完成| FSM
    MOD_ANG -->|sample_done 同步采样完成| INT_LOGIC
    MOD_ANG -->|sample_done 同步采样完成| LQR

    MOD_ENC -->|alpha_rad_q16, dalpha_q16| INT_LOGIC
    MOD_ANG -->|theta_err_q16, dtheta_q16| INT_LOGIC
    MOD_ANG -->|theta_err_q16, dtheta_q16| SWING
    TRAJ -->|alpha_ref_q16, dalpha_ref_q16| INT_LOGIC

    INT_LOGIC -->|alpha_err, dalpha_err, alpha_int| LQR
    INT_LOGIC -->|theta_err, dtheta| LQR

    FSM -->|仲裁输出 pwm_duty [-1000, 1000]| MOD_PWM
    MOD_PWM -->|PWM / DIR / IN1 / IN2 / STBY| DRIVER
    DRIVER -->|±12V 动力大电流| MOTOR
    MOTOR -->|扭矩驱动| ARM
```

---

## 二、外设一：直流有刷电机与 H 桥驱动原理与 Verilog 实现

### 2.1 电机是怎么转动的？为什么不能直接接 FPGA 引脚？
* **工作本质**：直流电机内部包含电枢线圈与永磁体。当电枢通入直流电流时，在安培力作用下产生电磁转矩驱动转子旋转。输出扭矩与电流成正比：$\tau = K_t \cdot I$；
* **为什么 FPGA 绝对不能直连电机？**
  1. **驱动能力极其有限**：FPGA 的普通 I/O 引脚（如 LVCMOS33）最大驱动电流仅 **4mA ~ 16mA**；
  2. **电机电流需求极大**：倒立摆直流电机的额定工作电流在 **0.5A ~ 2.0A**，启动与急停瞬间堵转冲击电流甚至可达 **3A ~ 5A**，直接相连会当场击穿烧毁 FPGA 芯片引脚；
  3. **反电动势（Back-EMF）高压击穿**：电机本质是一个强感性负载。断开瞬间，电感线圈两端会感应出高达几十伏的反向高压尖峰脉冲，必须通过专用的**驱动隔离芯片与续流二极管**进行保护。

---

### 2.2 什么是 H 桥？正转、反转、刹车与滑行机理
H 桥（H-Bridge）是由 4 个大功率开关管（MOSFET）组成的“H”形拓扑电路：

```text
       +12V 电源 (VM)
         |         |
      [ Q1 ]     [ Q3 ]    (高边 MOS)
         |----+----|
              |
           ( M )  直流电机
              |
         |----+----|
      [ Q2 ]     [ Q4 ]    (低边 MOS)
         |         |
        GND       GND
```

| 控制动作 | 导通的开关管 | 电机端子电压极性 | TB6612 输入 (IN1, IN2) | 物理现象与原理 |
| :--- | :--- | :---: | :---: | :--- |
| **正转 (Forward)** | **Q1 与 Q4 导通** | 左 (+) 右 (-) | `IN1 = PWM, IN2 = 0` | 电流从左向右，转臂顺时针旋转； |
| **反转 (Reverse)** | **Q3 与 Q2 导通** | 左 (-) 右 (+) | `IN1 = 0, IN2 = PWM` | 电流从右向左，转臂逆时针旋转； |
| **动态能耗制动 (Brake)** | **Q2 与 Q4 同时导通** | 两端短接到地 | `IN1 = 1, IN2 = 1` | **电机两端被短路！** 旋转的电机变成“发电机”，自感反向阻尼力矩实现**瞬间锁死刹车**； |
| **自由滑行 (Coast / Off)**| **4 个管子全部关断** | 高阻悬空态 (Hi-Z) | `IN1 = 0, IN2 = 0` | 电机两端断路，依靠机械摩擦缓慢自然减速。 |

> [!CAUTION]
> **严禁直通短路（Shoot-Through）**：绝对不能让同侧的 Q1 和 Q2（或 Q3 和 Q4）同时导通，否则 12V 电源直接短路到 GND，瞬间烧毁驱动芯片！商业驱动芯片与驱动代码必须包含死区时间（Dead-time）保护。

---

### 2.3 什么是 PWM？为什么必须选 20kHz 载波？
* **脉宽调制（PWM）**：直流电源供电电压恒定为 12V。PWM 通过以极高频率开关 12V 电源，改变**高电平所占周期的百分比（占空比 Duty Cycle）**，使得电机两端的等效平均电压平滑可调：
  $$V_{avg} = V_{bus} \times \frac{t_{on}}{T_{pwm}} = V_{bus} \times \text{Duty}$$
* **为什么倒立摆系统 PWM 载波必须严格设在 20kHz？**
  1. **彻底消除人耳啸叫**：人耳听觉极限在 $20\,\text{Hz} \sim 20\,\text{kHz}$。低于 20kHz 的载波（如 1kHz 或 8kHz）会使电机线圈发出刺耳的蜂鸣尖叫声；
  2. **保证电感电流连续平稳**：电机线圈内阻小、电感大。20kHz 周期仅 $50\,\mu\text{s}$，在如此短的时间内，电感电流几乎为连续平滑的直线（连续导通模式 CCM），消除电磁力矩脉动，保障倒立摆平衡控制极其丝滑。

---

### 2.4 `motor_pwm_driver.v` 逐行代码与逻辑深度精解

#### 1. 周期与比较阈值计算（无除法器设计）
系统时钟 50MHz（周期 20ns），PWM 目标频率 20kHz（周期 50μs）：
$$\text{TIMER\_PERIOD} = \frac{50\,\text{MHz}}{20\,\text{kHz}} = 2500\,\text{时钟周期}$$
输入 `pwm_duty` 范围为 $[-1000, +1000]$。换算比例为 $2500 / 1000 = 2.5$。
为了消除 FPGA 内部运行时的除法器（缺陷 E4），代码采用 Q16 定点乘法预换算：
$$\text{SCALE\_FACTOR\_Q16} = \frac{2500 \times 65536}{1000} = 163840$$
$$\text{raw\_compare} = (\text{duty\_clamped} \times 163840) \gg 16$$

#### 2. 无毛刺影子寄存器（Shadow Register）
```verilog
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pwm_counter <= 16'd0;
        compare_val <= 16'd0;
        dir_reg     <= 1'b0;
    end else begin
        if (pwm_counter >= TIMER_PERIOD - 1) begin
            pwm_counter <= 16'd0;
            // 只有当 PWM 计数器走到周期谷底 (归零) 的瞬间，才更新比较值与方向！
            compare_val <= raw_compare[15:0];
            dir_reg     <= dir_detected;
        end else begin
            pwm_counter <= pwm_counter + 1'b1;
        end
    end
end
```
> [!TIP]
> **设计玄机**：若在计数器走到中间（如 1500）时外部突变占空比将比较值缩小为 1000，计数器将错过匹配点一路数到 2500 溢出，导致该周期输出 100% 满偏波形，产生电磁爆震。**影子寄存器保证占空比与方向永远只在载波周期边界无缝切换**。

#### 3. TB6612FNG 专属 STBY 休眠引脚自适应控制
```verilog
assign stby_out = motor_en | brake_mode;
```
* 当系统使能运行（`motor_en=1`）时，`stby_out = 1` 芯片正常输出；
* 当电机停机（`motor_en=0`）但要求动态能耗制动（`brake_mode=1`）时，**必须保持 `stby_out = 1`**，使 $IN1=1, IN2=1$ 的接地短路刹车真实生效；
* 仅在停机且选择自由滑行（`brake_mode=0`）时，才拉低 `stby_out = 0` 进入高阻休眠。

---

## 三、外设二：正交增量式光电编码器原理与 Verilog 实现

### 3.1 编码器内部长什么样？AB 相正交脉冲是怎么产生的？
套件水平转臂电机尾部装有一只 **1000 线增量式光电编码器**：
1. **机械结构**：电动机转轴同轴带动一块精密的透光码盘，码盘圆周上刻有 1000 条极其细微的等间距辐射狭缝；
2. **光电对管与 90° 相位差**：传感器内部设有 A 相与 B 相两组光电管，在空间安装位置上**故意错开了 1/4 个狭缝周期（电气相位差严格为 90°，即正交）**。

```text
顺时针正转 (CW, A 相超前 B 相 90°):
A 相: ___|ˉˉˉ|___|ˉˉˉ|___|ˉˉˉ|___
B 相: _____|ˉˉˉ|___|ˉˉˉ|___|ˉˉˉ|_
状态:  00   10   11   01   00

逆时针反转 (CCW, B 相超前 A 相 90°):
A 相: _____|ˉˉˉ|___|ˉˉˉ|___|ˉˉˉ|_
B 相: ___|ˉˉˉ|___|ˉˉˉ|___|ˉˉˉ|___
状态:  00   01   11   10   00
```

---

### 3.2 为什么必须进行 4 倍频鉴相？方向是怎么判断的？
* **四倍频（4x Quadrature Decoding）的原理**：
  A 相包含上升沿与下降沿（2 次跳变），B 相也包含上升沿与下降沿（2 次跳变）。每个狭缝周期内共有 **4 次电平组合变化**。1000 线编码器经 4 倍频后，电机转动一整圈产生 **4000 个脉冲（Counts Per Revolution, CPR）**！
* **测量角分辨率提升至**：
  $$\Delta \alpha = \frac{360^\circ}{4000} = 0.09^\circ = 0.00157\,\text{rad}$$
* **方向判别真值表（状态机转移）**：
  设上一时刻电平为 $(A_{prev}, B_{prev})$，当前电平为 $(A_{curr}, B_{curr})$：
  - 若跳变序列为 `00->10`, `10->11`, `11->01`, `01->00`，判定为**正转，计数累加 +1**；
  - 若跳变序列为 `00->01`, `01->11`, `11->10`, `10->00`，判定为**反转，计数递减 -1**；
  - 其余未跳变或同时跳变（`00->11` 非法态）计数累加 0。

---

### 3.3 为什么实物必须加硬件消抖滤波？
电机换向电刷打火与 20kHz PWM 快速开关会在长排线上感应出高频毛刺。若直接使用沿触发器，毛刺将引发误计数导致转臂漂移。
* **第一道防线：双级 D 触发器防亚稳态**（`a_sync_reg`）；
* **第二道防线：连续积分消抖低通滤波器**：
  ```verilog
  if (a_sync_reg[1] == a_filt) begin
      a_filter_cnt <= 8'd0;
  end else if (a_filter_cnt >= FILTER_CYCLES - 1) begin
      a_filt       <= a_sync_reg[1];
      a_filter_cnt <= 8'd0;
  end else begin
      a_filter_cnt <= a_filter_cnt + 1'b1;
  end
  ```
  设置 `FILTER_CYCLES = 8`（$8 \times 20\,\text{ns} = 160\,\text{ns}$）。电平必须稳定保持 160ns 以上才允许内部翻转，小于 160ns 的高频电磁尖峰被 100% 滤除。

---

### 3.4 32 位脉冲累加与 1ms M 法测速定点化 (`encoder_quad_reader.v`)
* **32 位宽有符号计数器（修复 D5）**：
  旧代码采用 16 位计数器，在转动数十圈后会发生溢出翻转。扩展至 32 位 signed 彻底消除溢出截断；
* **角度变换（无除法器）**：
  4000 CPR 对应 $2\pi\,\text{rad}$。$2\pi \times 65536 / 4000 = 102.9437$。
  采用 18 位有符号常数乘以 1024 定标：$102.9437 \times 1024 = 105414$。
  ```verilog
  alpha_rad_q16 <= (pulse_cnt_18 * 18'sd105414) >>> 10;
  ```
  乘法位宽收敛在 MULT18X18 内，相对误差仅 $0.00033\%$；
* **1ms M 法测速与一阶 IIR 滤波**：
  在 1ms（0.001s）节拍下，脉冲增量 $\Delta P$ 换算为速度：
  $$\dot{\alpha} = \frac{\Delta P}{0.001} \times \frac{2\pi}{4000} = \Delta P \times \frac{\pi}{2}$$
  定点 Q16 系数：$(\pi / 2) \times 65536 = 102944$。
  经过一阶 IIR 滤波（$\beta = 0.35$）：
  $$\hat{\dot{\alpha}}_k = \hat{\dot{\alpha}}_{k-1} + 0.35 \times (\text{raw} - \hat{\dot{\alpha}}_{k-1})$$
  其中 $0.35 \times 65536 = 22938$，完全消除硬件除法。

---

## 四、外设三：摆杆角度传感器与 12-bit ADC 原理与 Verilog 实现

### 4.1 倒立摆为什么需要绝对角度传感器？与电机编码器的根本区别
* **水平转臂**：关心的是相对电机的回转角和转速，掉电后转臂停在任意位置都可重新标定为零点，使用**增量式光电编码器**即可；
* **垂直摆杆**：必须知道摆杆相对于**绝对垂直重力方向**的夹角！系统上电时摆杆自然下垂（$\theta = 180^\circ$），必须有一款能在上电瞬间直接给出绝对位置的**绝对式传感器**（精密电位器或磁编码器 + 12-bit ADC），否则无法判断摆杆在哪个象限，无法启动能量泵。

---

### 4.2 2.5MHz 硬件 SPI 主机通信时序
摆杆传感器通过 SPI 总线与 FPGA 相连：
* `adc_cs_n`：片选引脚，低电平有效；
* `adc_sclk`：同步串行时钟，空闲为低（Mode 0），频率 $2.5\,\text{MHz}$（时钟周期 400ns）；
* `adc_miso`：从机串行数据输入，在 SCLK 上升沿采样数据。
一帧传输 16 个 bit，耗时 $16 \times 400\,\text{ns} = 6.4\,\mu\text{s}$，提取低 12 位转换数据（码值范围 $0 \sim 4095$）。

---

### 4.3 360° 越界最短角位移解卷绕与 3 拍流水线 (`angle_sensor_reader.v`)

```
 [SPI 采样完成 12-bit ADC] ──► active_raw_adc
                                   │
                                   ▼
                   diff_raw = active_raw_adc - zero_offset
                   若 diff_raw >  2047 -> diff - 4096 (映射到 -2048~2047)
                   若 diff_raw < -2048 -> diff + 4096
                                   │
                                   ▼
                   diff_rad = (diff_unwrapped × 411775) >>> 12
                                   │
 ┌─────────────────────────────────┴─────────────────────────────────┐
 │ 流水拍 1 (20ns): 锁存当前角度，计算未解卷绕角度差                     │
 │   theta_err_q16 <= current_theta_q16;                             │
 │   diff_raw_r    <= current_theta_q16 - theta_prev_q16;            │
 ├───────────────────────────────────────────────────────────────────┤
 │ 流水拍 2 (40ns): ★ 最短角位移解卷绕 (杜绝 ±180° 边界 6283 rad/s 尖峰)  │
 │   若 diff_raw_r >  pi ( 205887) -> diff - 2pi (411775)           │
 │   若 diff_raw_r < -pi (-205888) -> diff + 2pi (411775)           │
 │   raw_dtheta_q16 <= diff_theta_step × 1000;                       │
 ├───────────────────────────────────────────────────────────────────┤
 │ 流水拍 3 (60ns): 一阶 IIR 滤波平滑 + 发出 sample_done               │
 │   dtheta_q16  <= dtheta_q16 + ((raw - dtheta) × 22938) >>> 16;   │
 │   sample_done <= 1'b1;                                            │
 └───────────────────────────────────────────────────────────────────┘
```

#### ★ 为什么必须进行最短角位移解卷绕？（缺陷 M2 深度复盘）
当摆杆起摆穿越悬垂点（$\pm 180^\circ$）时，角度采样值会从 $+179^\circ$ 突变到 $-179^\circ$（真实角位移仅 $2^\circ$）。
如果直接做差分：
$$\Delta \theta = -179^\circ - (+179^\circ) = -358^\circ \approx -6.248\,\text{rad}$$
在 1ms 内直接乘以 1000，将产生高达 **$-6248\,\text{rad/s}$ 的伪大尖峰**！
而摆杆物理真实速度仅约 $35\,\text{rad/s}$，尖峰为其 178 倍！IIR 滤波器会被严重污染达 20ms，直接摧毁起摆能量计算。
**最短角位移解卷绕算法**在差值超出 $\pm \pi$ 时自动叠加或扣除 $2\pi$，将差分限制在最短物理路径内，伪尖峰实测彻底归零（判据 A2 验证通过）。

#### 时序流水线拆分依据
解卷绕三选逻辑若与 64 位乘法串联，组合逻辑延时高达 19.9ns，使系统 Fmax 崩至 50.194MHz（裕量仅 0.39%）。
将其拆分为严格的 **3 拍流水线**后，最差路径数据延时大幅降至 16.0ns，保障整体 Fmax 稳稳达到 **$65.849\,\text{MHz}$**。

---

## 五、系统级整合：`j280_hw_top.v` 顶层互联、采样同步与安全闭环

硬件顶层模块 [`j280_hw_top.v`](../../furuta_lqr_ctrl/src/j280_hw_top.v) 负责整体协调调度：

### 5.1 1ms 控制节拍与采样无偏差同步（修复 D6）
* 内部由 `TIMER_1MS_LIMIT = 50000` 产生 1ms 控制脉冲启动 SPI 采样；
* 当 SPI 采样、解卷绕与 IIR 滤波在第 3 拍完成时，`angle_sensor_reader` 发出单时钟周期高电平的 **`sample_done`** 信号；
* 顶层所有下游模块（状态机、编码器测速锁存、LQI 积分器、流水线计算核）**全部由 `sample_done` 统一使能触发**！两轴采样时间差被硬件级彻底对齐在同一个纳秒时刻。

---

### 5.2 一键垂直零位自适应硬件锁存（SOP 前置）
摆杆每次装配后均存在机械微小公差：
1. 操作人员将摆杆手扶至铅垂线上；
2. 长按核心板上的 `key_zero_calib`（KEY1）；
3. 经 100 万时钟周期（20ms）防抖后，当前最新的 `raw_adc_data` 自动锁存进 `zero_offset_reg`；
4. 板载指示灯 `led_calib_ok` 保持常亮，`calib_done` 置 1，放行状态机允许起摆。

---

### 5.3 独立多模式轮换与位置清零（KEY2 复合功能）
独立按键 KEY2 兼具短按与长按双重功能：
* **短按（按键时长 $20\,\mu\text{s} \sim 70\,\text{ms}$）**：
  状态机轮转切换轨迹模式（$0 \to 1 \to 2 \to 3 \to 0$），对应“0°平衡 $\to$ +45°斜坡 $\to$ -45°斜坡 $\to$ 0.2Hz正弦”；
* **长按（按键时长 $\ge 70\,\text{ms}$）**：
  发出 `clear_pos_pulse`，将光电编码器的 `pulse_count` 绝对值强制清零，用于对齐转臂起始基准。

---

### 5.4 积分分离与抗饱和保护网
* **积分累加**：仅在系统进入 BALANCE 平衡态且转臂误差处于 $\pm 10^\circ$（`INT_ERR_THRESH_Q16 = 11439`）之内时进行无乘除移位累加；
* **窗口外冻结（M1）**：超出 $\pm 10^\circ$ 误差带时**冻结已有积分值不更新**，既杜绝大步长前冲过程中的积分恶性累加，又保护了斜坡跟踪所需的前馈补偿；
* **饱和限幅**：积分项绝对值限制在 $\pm 0.25\,\text{rad}$（Q16: `±16384`）；
* **转臂双向软限位（D3）**：当绝对位置超出 $\pm 2$ 整圈（$\pm 720^\circ$）时，`soft_limit_err` 置位切入停机，防止损坏机械结构。

---

## 六、Q12.16 定点数与实际物理量量纲换算完整字典

在调试观察高云 GAO 逻辑分析仪或 Modelsim 仿真波形时，物理量与 32 位有符号定点数换算如下：

| 物理状态变量 | 物理量纲 | 典型工作区间 | Q12.16 换算公式 | 示例物理值 | 对应 16 进制 / 10 进制 Q16 码值 |
| :--- | :---: | :---: | :--- | :--- | :--- |
| **摆杆倾角偏差 $\theta$** | $\text{rad}$ | $[-0.78, +0.78]$ | $\text{Val}_{Q16} = \theta \times 65536$ | $+0.100\,\text{rad} \approx 5.73^\circ$ | `32'd6554` (`0x0000199A`) |
| **倒立平衡捕获门限** | $\text{deg}$ | $\pm 22.0^\circ$ | $\text{Val}_{Q16} = 0.38397 \times 65536$ | $\pm 22.0^\circ$ | `32'sd25164` (`0x0000624C`) |
| **摆杆跌落急停门限** | $\text{deg}$ | $\pm 45.0^\circ$ | $\text{Val}_{Q16} = 0.78540 \times 65536$ | $\pm 45.0^\circ$ | `32'sd51472` (`0x0000C910`) |
| **摆杆角速度 $\dot{\theta}$** | $\text{rad/s}$ | $[-32.0, +32.0]$ | $\text{Val}_{Q16} = \dot{\theta} \times 65536$ | $+17.151\,\text{rad/s}$ (起摆临界速度) | `32'sd1124008` (`0x001126A8`) |
| **转臂角位移 $\alpha$** | $\text{rad}$ | $[-12.56, +12.56]$ | $\text{Val}_{Q16} = \alpha \times 65536$ | $+0.7854\,\text{rad} = +45^\circ$ | `32'sd51472` (`0x0000C910`) |
| **转臂软限位保护** | $\text{deg}$ | $\pm 720.0^\circ$ | $\text{Val}_{Q16} = 4\pi \times 65536$ | $\pm 2$ 整圈 ($\pm 720^\circ$) | `32'sd823548` (`0x000C90FC`) |
| **转臂角速度 $\dot{\alpha}$** | $\text{rad/s}$ | $[-15.0, +15.0]$ | $\text{Val}_{Q16} = \dot{\alpha} \times 65536$ | $+0.8698\,\text{rad/s}$ (斜坡标准巡航速度) | `32'sd57000` (`0x0000DE98`) |
| **LQI 转臂位置积分** | $\text{rad}\cdot\text{s}$ | $[-0.25, +0.25]$ | $\text{Val}_{Q16} = \text{Int} \times 65536$ | $\pm 0.25\,\text{rad}\cdot\text{s}$ (饱和上限) | `32'sd16384` (`0x00004000`) |
| **积分分离误差窗口** | $\text{deg}$ | $\pm 10.0^\circ$ | $\text{Val}_{Q16} = 0.17453 \times 65536$ | $\pm 10.0^\circ$ (窗口外冻结) | `32'sd11439` (`0x00002CAF`) |
| **电机驱动 `pwm_duty`**| 千分比 | $[-1000, +1000]$ | 直读整数 (符号位代表方向) | $+250$ ($3.0\,\text{V}$ 起动死区突破脉冲) | `16'sd250` (`0x00FA`) |
