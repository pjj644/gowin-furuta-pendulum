# J280 姿态控制系统竞赛套件 —— 外设接口 Verilog 设计与硬件原理解析全手册

> **适用竞赛**：全国大学生嵌入式芯片与系统设计竞赛（FPGA 创新设计赛道 —— 选题一：基于 FPGA 的实时姿态控制系统）  
> **核心板卡**：高云半导体晨熙家族第一代 FPGA（`GW2A-LV55PG484C8/I7`）  
> **配套机械**：J280 姿态控制系统旋转倒立摆套件  
> **读者定位**：初次接触电机驱动、光电正交编码器与高精度角度传感器的参赛队员与 FPGA 开发者  
> **对应源码**：
> - [`motor_pwm_driver.v`](../../furuta_lqr_ctrl/src/motor_pwm_driver.v) —— 直流电机 20kHz 高频 PWM 与 H 桥驱动发生器
> - [`encoder_quad_reader.v`](../../furuta_lqr_ctrl/src/encoder_quad_reader.v) —— 电机编码器 4 倍频消抖滤波测速测角模块
> - [`angle_sensor_reader.v`](../../furuta_lqr_ctrl/src/angle_sensor_reader.v) —— 摆杆 12-bit SPI ADC / 磁编码器零点校准与解卷绕模块
> - [`j280_hw_top.v`](../../furuta_lqr_ctrl/src/j280_hw_top.v) —— 硬件系统顶层集成互联与安全保护模块

---

## 目录
1. [倒立摆硬件外设体系全景拓扑](#一倒立摆硬件外设体系全景拓扑)
2. [外设一：直流有刷电机与 H 桥驱动原理与 Verilog 实现](#二外设一直流有刷电机与-h-桥驱动原理与-verilog-实现)
   - [2.1 电机是怎么转动的？为什么不能直接接 FPGA 引脚？](#21-电机是怎么转动的为什么不能直接接-fpga-引脚)
   - [2.2 什么是 H 桥？正转、反转、刹车与滑行机理](#22-什么是-h-桥正转反转刹车与滑行机理)
   - [2.3 什么是 PWM？为什么必须选 20kHz 载波？](#23-什么是-pwm为什么必须选-20khz-载波)
   - [2.4 `motor_pwm_driver.v` 逐行代码与逻辑深度精解](#24-motor_pwm_driverv-逐行代码与逻辑深度精解)
3. [外设二：正交增量式光电编码器原理与 Verilog 实现](#三外设二正交增量式光电编码器原理与-verilog-实现)
   - [2.1 编码器内部长什么样？AB 相正交脉冲是怎么产生的？](#31-编码器内部长什么样ab-相正交脉冲是怎么产生的)
   - [2.2 为什么必须进行 4 倍频鉴相？方向是怎么判断的？](#32-为什么必须进行-4-倍频鉴相方向是怎么判断的)
   - [2.3 为什么实物必须加硬件消抖滤波？电机电磁干扰分析](#33-为什么实物必须加硬件消抖滤波电机电磁干扰分析)
   - [2.4 1ms 定时测速 M 法与一阶 IIR 平滑滤波原理](#34-1ms-定时测速-m-法与一阶-iir-平滑滤波原理)
   - [2.5 `encoder_quad_reader.v` 逐行代码与逻辑深度精解](#35-encoder_quad_readerv-逐行代码与逻辑深度精解)
4. [外设三：摆杆角度传感器与 12-bit ADC 原理与 Verilog 实现](#四外设三摆杆角度传感器与-12-bit-adc-原理与-verilog-实现)
   - [4.1 倒立摆为什么需要绝对角度传感器？与电机编码器的根本区别](#41-倒立摆为什么需要绝对角度传感器与电机编码器的根本区别)
   - [4.2 传感器类型：精密导电塑料电位器 vs 磁编码器](#42-传感器类型精密导电塑料电位器-vs-磁编码器)
   - [4.3 SPI 串行总线通信时序（CS_N, SCLK, MISO）](#43-spi-串行总线通信时序cs_n-sclk-miso)
   - [4.4 360° 越界解卷绕（Unwrapping）与垂直零点校准算法](#44-360-越界解卷绕unwrapping与垂直零点校准算法)
   - [4.5 `angle_sensor_reader.v` 逐行代码与逻辑深度精解](#45-angle_sensor_readerv-逐行代码与逻辑深度精解)
5. [系统级整合：`j280_hw_top.v` 顶层互联与安全闭环](#五系统级整合j280_hw_topv-顶层互联与安全闭环)
   - [5.1 1ms (1000Hz) 控制节拍产生器](#51-1ms-1000hz-控制节拍产生器)
   - [5.2 一键垂直零位自适应硬件锁存](#52-一键垂直零位自适应硬件锁存)
   - [5.3 倾角超限急停自锁与 LQI 积分抗饱和](#53-倾角超限急停自锁与-lqi-积分抗饱和)
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

    subgraph GW2A["高云 GW2A-LV55 FPGA 内部逻辑"]
        subgraph MOD_IN["外设采集与信号处理模块"]
            MOD_ENC["encoder_quad_reader.v<br/>8级数字消抖 + 4倍频<br/>1ms差分测速 + IIR平滑滤波"]
            MOD_ANG["angle_sensor_reader.v<br/>硬件 SPI 主机 (2.5MHz)<br/>360°去卷绕 + 零点扣除 + 差分测速"]
        end

        subgraph CORE["核心控制算法"]
            HEARTBEAT["1ms (1000Hz) 控制心跳定时器"]
            LQR["furuta_lqr_ctrl.v<br/>LQI 最优姿态自平衡乘加流水线<br/>(3级流水线, 60ns极速计算)"]
            INT_LOGIC["转臂积分累加与跌落急停保护"]
        end

        subgraph MOD_OUT["执行驱动模块"]
            MOD_PWM["motor_pwm_driver.v<br/>20kHz 影子寄存器 PWM<br/>正反转/刹车/滑行逻辑"]
        end
    end

    subgraph ACTUATOR["执行机构 (输出)"]
        DRIVER["H 桥驱动芯片<br/>(TB6612FNG / A4950)"]
        MOTOR["直流高速/永磁减速电机"]
    end

    %% 硬件物理连接
    ARM -.->|带动旋转| ENC_HW
    PEND -.->|角度偏转| ADC_HW
    ENC_HW -->|A/B 正交 TTL 电平| MOD_ENC
    ADC_HW -->|SPI 总线 (MISO)| MOD_ANG

    HEARTBEAT -->|1ms calc_en 脉冲| MOD_ENC
    HEARTBEAT -->|1ms calc_en 脉冲| MOD_ANG
    HEARTBEAT -->|1ms calc_en 脉冲| LQR

    MOD_ENC -->|alpha (Q16 rad)<br/>dalpha (Q16 rad/s)| INT_LOGIC
    MOD_ANG -->|theta (Q16 rad)<br/>dtheta (Q16 rad/s)| INT_LOGIC

    INT_LOGIC -->|alpha_err, dalpha, alpha_int| LQR
    INT_LOGIC -->|theta_err, dtheta| LQR

    LQR -->|pwm_duty [-1000, 1000]| MOD_PWM
    MOD_PWM -->|PWM / DIR / IN1 / IN2 / STBY| DRIVER
    DRIVER -->|±12V 动力大电流| MOTOR
    MOTOR -->|扭矩驱动| ARM
```

---

## 二、外设一：直流有刷电机与 H 桥驱动原理与 Verilog 实现

### 2.1 电机是怎么转动的？为什么不能直接接 FPGA 引脚？
* **工作本质**：直流电机内部包含电枢线圈与永磁体。当电枢通入直流电流时，在安培力作用下产生电磁转矩，驱动转子旋转。输出扭矩与电流成正比：$\tau = K_t \cdot I$。
* **为什么 FPGA 绝对不能直连电机？**
  1. **驱动能力极其有限**：FPGA 的普通 I/O 引脚（如 LVCMOS33）最大输出电流通常只有 **4mA ~ 16mA**；
  2. **电机电流需求极大**：倒立摆直流电机的额定工作电流在 **0.5A ~ 2.0A**，启动瞬间堵转冲击电流甚至可达 **3A ~ 5A**，直接相连会当场击穿烧毁 FPGA 芯片引脚！
  3. **反电动势（Flyback / Back-EMF）高压击穿**：电机本质是一个强感性负载。断开瞬间，电感线圈两端会感应出高达几十伏的反向高压尖峰脉冲，必须通过专用的**驱动隔离芯片与续流二极管**进行保护。

---

### 2.2 什么是 H 桥？正转、反转、刹车与滑行机理
H 桥（H-Bridge）是由 4 个大功率开关管（通常为 MOSFET）组成的“H”形拓扑电路。通过切换对角线开关管的通断，可以自由改变电机线圈两端的电压极性：

```text
       +12V 电源 (VM)
         |         |
      [ Q1 ]     [ Q3 ]    (高边 P-MOS / N-MOS)
         |----+----|
              |
           ( M )  直流电机
              |
         |----+----|
      [ Q2 ]     [ Q4 ]    (低边 N-MOS)
         |         |
        GND       GND
```

| 控制动作 | 导通的开关管 | 电机端子电压极性 | 物理现象与原理 |
| :--- | :--- | :--- | :--- |
| **正转 (Forward)** | **Q1 与 Q4 导通**，其余关断 | 左 (+) 右 (-) | 电流从左往右流过线圈，电机顺时针旋转； |
| **反转 (Reverse)** | **Q3 与 Q2 导通**，其余关断 | 左 (-) 右 (+) | 电流从右往左流过线圈，电机逆时针旋转； |
| **动态能耗制动 (Brake)** | **Q2 与 Q4 同时导通** (或 Q1/Q3 同时导通) | 两端短接到地 | **电机两个端子被短路！** 旋转的电机变成“发电机”，产生巨大的自感反向阻尼力矩，实现**瞬间刹车锁定**； |
| **自由滑行 (Coast / Off)**| **4 个管子全部关断** | 高阻悬空态 (Hi-Z) | 电机端子断路，依靠机械轴承摩擦慢慢自然减速停下。 |

> [!CAUTION]
> **严禁直通短路（Shoot-Through）**：绝对不能让同侧的 Q1 和 Q2（或 Q3 和 Q4）同时导通，否则 12V 供电直接短路到地，瞬间烧穿驱动板！商业芯片（如 TB6612 / A4950）内部集成了硬件死区时间（Dead-time）防直通保护。

---

### 2.3 什么是 PWM？为什么必须选 20kHz 载波？
* **脉宽调制（PWM）**：直流供电电压恒定为 12V，无法输出连续的 3.7V、8.2V 等中间模拟电压。PWM 通过极快地开关 12V 电源，通过改变**高电平所占周期的百分比（占空比 Duty Cycle）**，使得电机两端的等效平均电压平滑可调：
  $$V_{avg} = V_{bus} \times \frac{t_{on}}{T_{pwm}} = V_{bus} \times \text{Duty}$$
* **为什么倒立摆系统 PWM 载波必须设在 20kHz？**
  1. **消除刺耳的人耳啸叫**：人耳听觉极限频率在 $20\,\text{Hz} \sim 20\,\text{kHz}$。若使用常见的单片机 1kHz 或 5kHz PWM，电机线圈在交流磁场震荡下会发出极其尖锐刺耳的高频“蜂鸣尖叫声”，严重干扰现场答辩与调参；
  2. **保证电感滤波电流平稳连续**：电机线圈内阻小、电感大。20kHz 周期仅 $50\,\mu\text{s}$，在这样短的时间内，电感电流几乎为平滑的一条直线（连续电流模式 CCM），力矩输出极其平滑无脉动，倒立摆平衡控制极其丝滑。

---

### 2.4 `motor_pwm_driver.v` 逐行代码与逻辑深度精解

该模块位于 [`C:\Users\28399\Desktop\GoWin\furuta_lqr_ctrl\src\motor_pwm_driver.v`](../../furuta_lqr_ctrl/src/motor_pwm_driver.v)：

#### 1. 周期与比较阈值计算
系统时钟为板载 50MHz（周期 20ns），PWM 目标频率为 20kHz（周期 50μs）：
$$\text{TIMER\_PERIOD} = \frac{50\,\text{MHz}}{20\,\text{kHz}} = 2500\,\text{时钟周期}$$
我们的控制算法 `furuta_lqr_ctrl` 输出的 `pwm_duty` 范围为 $[-1000, +1000]$。换算关系：
$$\text{compare\_val} = \frac{|\text{pwm\_duty}| \times 2500}{1000} = \frac{|\text{pwm\_duty}| \times 5}{2}$$

#### 2. 无毛刺影子寄存器（Shadow Register）机制
```verilog
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pwm_counter <= 16'd0;
        compare_val <= 16'd0;
        dir_reg     <= 1'b0;
    end else begin
        if (pwm_counter >= TIMER_PERIOD - 1) begin
            pwm_counter <= 16'd0;
            // 只有当 PWM 计数器走到周期终点 (归零) 的瞬间，才更新 compare_val 和 dir_reg！
            compare_val <= raw_compare[15:0];
            dir_reg     <= dir_detected;
        end else begin
            pwm_counter <= pwm_counter + 1'b1;
        end
    end
end
```
> [!TIP]
> **设计玄机**：如果在 PWM 计数器正走到一半（例如计数值 1200）时外部突变占空比把比较值改小为 800，计数器将错过匹配点一直数到溢出，导致这一周期电机输出异常的 100% 满偏波形，产生电磁爆震。**影子寄存器机制保证了占空比与方向永远只在 PWM 周期边界无缝平滑切换**。

#### 3. 双模式引脚输出兼容
模块同时驱动 `pwm_out + dir_out`（用于单极性驱动板）以及 `in1_out + in2_out`（直接兼容 J280 套件上的 TB6612FNG 芯片）：
* **正转时**：`in1_out <= pwm_active; in2_out <= 1'b0;`
* **反转时**：`in1_out <= 1'b0; in2_out <= pwm_active;`
* **急停刹车时**：`in1_out <= 1'b1; in2_out <= 1'b1;`

---

## 三、外设二：正交增量式光电编码器原理与 Verilog 实现

### 3.1 编码器内部长什么样？AB 相正交脉冲是怎么产生的？
套件水平转臂电机尾部装有一只 **1000 线增量式光电编码器**：
1. **机械结构**：电动机转轴同轴带动一块精密的透光玻璃/金属码盘，码盘圆周上刻有 1000 条极其细微的等间距透光辐射狭缝；
2. **光电对管与 90° 相位差**：在码盘两侧固定有发光二极管与光敏接收管。巧妙的是，传感器内有 **A 相与 B 相** 两个接收窗口，在空间机械安装位置上**故意错开了 1/4 个狭缝周期（电气相位差严格为 90°，即正交）**。

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
* **传统单边沿计数的局限**：如果只在 A 相上升沿计数，电机转一圈只能计 1000 个数，分辨率仅为 $360^\circ / 1000 = 0.36^\circ$；
* **四倍频（4x Quadrature Decoding）的巨大优势**：
  - A 相有上升沿和下降沿（2 次跳变）；
  - B 相也有上升沿和下降沿（2 次跳变）；
  - 每一个机械狭缝周期内，A 和 B 组合共有 **4 次电平跳变状态**！
  - 1000 线编码器经过 4 倍频后，电机转动一整圈产生 **4000 个脉冲（Counts Per Revolution, CPR）**！
  - 测量角分辨率被暴增提升到：
    $$\Delta \alpha = \frac{360^\circ}{4000} = 0.09^\circ = 0.00157\,\text{rad}$$
* **方向判别真值表（状态机转移）**：
  设上一时刻电平为 $(A_{prev}, B_{prev})$，当前电平为 $(A_{curr}, B_{curr})$：
  - 若跳变序列为 `00->01`, `01->11`, `11->10`, `10->00`，说明为**顺时针正转，计数累加 +1**；
  - 若跳变序列为 `00->10`, `10->11`, `11->01`, `01->00`，说明为**逆时针反转，计数递减 -1**；
  - 若状态未改变，计数加 0；若发生同向突变（如 `00->11`），说明采样速度不足或发生故障。

---

### 3.3 为什么实物必须加硬件消抖滤波？电机电磁干扰分析
> [!WARNING]
> **工程血泪教训**：初学者最常犯的错误，就是将外部编码器 A/B 引脚直接连到上升沿触发器 `always @(posedge enc_a)`！
> 在实物运行中，直流电机换向电刷不断打火，PWM 20kHz 快速开关会在长排线上感应出强烈的纳秒级高频尖峰电压毛刺。如果不做滤波，一个毛刺就会让寄存器误触发几十次甚至上千次，导致转臂位置计数器狂漂、电机失控打飞！

**硬件级多重滤波防护机制**：
1. **第一道防线：双级 D 触发器防亚稳态同步**：
   外部编码器信号与 FPGA 50MHz 时钟异步。两级触发器串联彻底打掉亚稳态；
2. **第二道防线：连续采样积分消抖低通滤波器**：
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
   设定 `FILTER_CYCLES = 8`。只有当引脚电平稳定维持 8 个 50MHz 时钟周期（$8 \times 20\,\text{ns} = 160\,\text{ns}$）以上，内部信号才允许翻转。宽度小于 160ns 的高频电磁毛刺被 100% 滤除！

---

### 3.4 1ms 定时测速 M 法与一阶 IIR 平滑滤波原理
* **M 法测速原理**：FPGA 内部心跳节拍为 $T_s = 1.0\,\text{ms}$。在每个 1ms 周期内：
  $$\Delta \text{pulse} = \text{pulse\_count}[k] - \text{pulse\_count}[k-1]$$
  水平转臂角速度计算公式：
  $$\dot{\alpha}_{raw} = \frac{\Delta \text{pulse} \times (2\pi / 4000)}{0.001\,\text{s}} = \Delta \text{pulse} \times \frac{\pi}{2} \approx \Delta \text{pulse} \times 1.5707963\,\text{rad/s}$$
* **为什么需要一阶 IIR 滤波？**
  因为 M 法测速存在 $\pm 1$ 个脉冲的固有离散量化阶跃（若 1ms 差 1 个脉冲，速度就会产生 $1.57\,\text{rad/s}$ 的剧烈跳变）。
  我们在硬件中实现一阶 IIR 数字低通滤波（对应 `config.py` 中的 35% 权重）：
  $$\dot{\alpha}[k] = \dot{\alpha}[k-1] + 0.35 \times (\dot{\alpha}_{raw}[k] - \dot{\alpha}[k-1])$$
  在 FPGA 中全定点整数运算，输出极其丝滑纯净的角速度！

---

### 3.5 `encoder_quad_reader.v` 逐行代码与逻辑深度精解
该模块位于 [`C:\Users\28399\Desktop\GoWin\furuta_lqr_ctrl\src\encoder_quad_reader.v`](../../furuta_lqr_ctrl/src/encoder_quad_reader.v)：
* **输入输出**：50MHz 时钟、复位、外部 A/B/Z 引脚、1ms 控制使能 `calc_en`；
* **核心输出**：
  - `pulse_count`：32 位有符号绝对位置（顺时针累加，逆时针累减）；
  - `alpha_rad_q16`：转换为 Q12.16 定点格式的弧度值；
  - `dalpha_rad_s_q16`：滤波后的 Q12.16 格式角速度。

---

## 四、外设三：摆杆角度传感器与 12-bit ADC 原理与 Verilog 实现

### 4.1 倒立摆为什么需要绝对角度传感器？与电机编码器的根本区别
* **电机编码器是“增量式（Relative）”的**：掉电后不知道当前转到了几度，必须从 0 重新开始数；
* **摆杆必须使用“绝对式（Absolute）”角度传感器**：
  倒立摆在自由下垂（自然静止）时倾角为 $180^\circ$（即 $\pm\pi\,\text{rad}$），在被甩到倒立顶点自平衡时倾角为 $0^\circ$。控制算法每一时刻都必须确切知道摆杆在 360° 空间内的**绝对真实姿态**！哪怕中途重启，也能立即读出角度，否则无法实施能量起摆。

---

### 4.2 传感器类型：精密导电塑料电位器 vs 磁编码器
在 J280 套件及电赛倒立摆中，主要采用以下两种主流传感器：
1. **方案 A：高精度 360° 连续回转导电塑料电位器 + 12-bit SPI ADC 芯片（最通用）**：
   - 电位器中心抽头输出 $0\,\text{V} \sim 3.3\,\text{V}$ 的模拟连续线性电压；
   - 由板载的高速 12-bit ADC 芯片（如 TI 的 ADS7886、Microchip 的 MCP3201/3202、或 TLC549）转换为 0~4095 的离散数字码，并通过 SPI 总线串行发给 FPGA；
2. **方案 B：SPI 绝对式磁编码芯片（如 TLE5012B / AS5048A / AS5600）**：
   - 摆杆旋转轴末端镶嵌一块径向对充磁铁，芯片内部霍尔阵列感应磁场向量，直接通过 SPI 串行接口输出 12 位或 14 位的当前角度数字值。

---

### 4.3 SPI 串行总线通信时序（CS_N, SCLK, MISO）
SPI（Serial Peripheral Interface）是全双工高速同步串行接口。我们的 FPGA 作为 **SPI 主机（Master）**，ADC/磁传感器作为 **从机（Slave）**：
* `spi_cs_n`（片选，低有效）：平常为高电平；拉低代表开始一次数据转换与传输；
* `spi_sclk`（同步时钟）：FPGA 将 50MHz 主频分频为 **2.5MHz** 发送给从机；
* `spi_miso`（从机输出数据）：在每个 `spi_sclk` 上升沿，FPGA 锁存采样该线上的 1 位数据，传输 16 个周期即可拼成完整的一帧 12 位数据字。

```text
CS_N : ˉˉ\________________________________________________/ˉˉˉ
SCLK : ____/ˉ\_/ˉ\_/ˉ\_/ˉ\_/ˉ\_/ˉ\_/ˉ\_/ˉ\_/ˉ\_/ˉ\_/ˉ\_/ˉ\_____
MISO : ----< D11 >< D10 >< D9  >< ...  >< D1  >< D0  >---------
```

---

### 4.4 360° 越界解卷绕（Unwrapping）与垂直零点校准算法
12-bit ADC 的采样数据是正整数 $0 \sim 4095$（对应 $0^\circ \sim 360^\circ$）。在倒立摆控制中，必须解决两大关键数学转换：

#### 1. 垂直零点扣除
设摆杆垂直倒立（平衡点）时实测的 ADC 码值为 `zero_offset_raw`（例如 2048）：
$$\text{diff} = \text{raw\_adc} - \text{zero\_offset\_raw}$$

#### 2. 角度全圆周越界解卷绕（Unwrapping）
当摆杆在垂直零位附近轻微左右晃动时，如果是零点附近的边界（例如零点在 0 附近，往左微偏会变成 4095），差值会出现 $-4090$ 到 $+5$ 的巨大突跳！
我们在 Verilog 中使用全圆周模运算进行硬件平移：
```verilog
always @(*) begin
    diff_raw = $signed({1'b0, active_raw_adc}) - $signed({1'b0, zero_offset_raw});
    if (diff_raw > 13'sd2047) begin
        diff_unwrapped = diff_raw - 13'sd4096; // 逆时针越过半周平移
    end else if (diff_raw < -13'sd2048) begin
        diff_unwrapped = diff_raw + 13'sd4096; // 顺时针越过半周平移
    end else begin
        diff_unwrapped = diff_raw;
    end
end
```
> [!IMPORTANT]
> 经过解卷绕后，无论机械传感器安装时的绝对角度朝向如何，**`diff_unwrapped` 都会严格落在 $[-2048, +2047]$ 的区间内，且以垂直倒立位置严格为 0 点**！

#### 3. 刻度转换为 Q12.16 定点弧度
4096 个刻度对应 $2\pi\,\text{rad}$：
$$\theta_{err} = \text{diff\_unwrapped} \times \frac{2\pi}{4096} = \text{diff\_unwrapped} \times 0.00153398\,\text{rad}$$
在 Q16 格式下（乘 65536）：
$$\text{scale} = \frac{2\pi \times 65536}{4096} = \frac{411774.8}{4096} \approx 100.531$$
在 Verilog 中使用一条极简位移实现超高精度：
```verilog
theta_err_q16 <= (diff_unwrapped * 32'sd411775) >>> 12;
```

---

### 4.5 `angle_sensor_reader.v` 逐行代码与逻辑深度精解
该模块位于 [`C:\Users\28399\Desktop\GoWin\furuta_lqr_ctrl\src\angle_sensor_reader.v`](../../furuta_lqr_ctrl/src/angle_sensor_reader.v)：
* 内置 4 状态 SPI 主机状态机（`IDLE`, `START`, `XFER`, `FINISH`）；
* 支持外部直接数据旁路模式 `ext_raw_valid`（便于仿真和板载并行 ADC 切换）；
* 每 1ms 完成一次采样并自动计算差分角速度 `dtheta_q16` 并经一阶 IIR 滤波滤除 ADC 量化白噪声。

---

## 五、系统级整合：`j280_hw_top.v` 顶层互联与安全闭环

硬件顶层模块 [`j280_hw_top.v`](../../furuta_lqr_ctrl/src/j280_hw_top.v) 是将各大外设驱动与控制算法融为一体的枢纽：

### 5.1 1ms (1000Hz) 控制节拍产生器
倒立摆控制不能随意乱跑，必须严格运行在确定的时间离散网格上：
```verilog
localparam integer TIMER_1MS_LIMIT = 50_000_000 / 1000; // 50,000 个时钟
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) ...
    else if (timer_1ms_cnt >= TIMER_1MS_LIMIT - 1) begin
        timer_1ms_cnt <= 20'd0;
        calc_en_pulse <= 1'b1; // 发出 1 个周期的 1ms 同步脉冲
    end else ...
end
```
`calc_en_pulse` 同时触发 ADC 启动采样、编码器计算瞬时速度、以及 LQR 核心执行 3 级硬件流水线计算。

---

### 5.2 一键垂直零位自适应硬件锁存
实物摆杆每次安装或移动后，垂直倒立点对应的 ADC 值会有微小偏差。如果每次都要看示波器查数据、修改 Verilog 重新全编译综合半小时，调参将极其痛苦！
**本工程设计的硬件自校准机制**：
1. 开发者用手将摆杆扶直在铅垂线上；
2. 按一下开发板上的 `key_zero_calib` 按键；
3. 顶层消抖后立即将当前最新的 `raw_adc_data` 锁存进 `zero_offset_reg` 寄存器；
4. 板载指示灯 `led_calib_ok` 点亮，零位校准完毕！

---

### 5.3 倾角超限急停自锁与 LQI 积分抗饱和
1. **倾角超限跌落保护**：
   ```verilog
   localparam signed [31:0] FALL_ZONE_Q16 = 32'sd51472; // 约 45 度
   wire is_fall_down = (theta_err_q16 > FALL_ZONE_Q16) || (theta_err_q16 < -FALL_ZONE_Q16);
   wire signed [15:0] safe_pwm_duty = (sw_motor_en && !is_fall_down) ? lqr_pwm_duty : 16'sd0;
   ```
   如果摆杆偏离超过 $45^\circ$，说明已经倒下失控，系统立即强行切断电机 PWM 输出（置 0），防止转臂在桌面上狂甩打手或撞坏线缆；
2. **LQI 积分抗饱和与积分分离**：
   只有当摆角处于自平衡区（$|\theta| < 22^\circ$）时，才开启转臂位置误差累加积分，并将积分项严格钳位在 $[-0.25\,\text{rad}, +0.25\,\text{rad}]$，既消除了电机摩擦死区造成的静差，又杜绝了积分饱和冲出限位的问题。

---

## 六、Q12.16 定点数与实际物理量量纲换算完整字典

在 FPGA 算法设计与信号监控（如使用高云 GAO 逻辑分析仪）时，物理真实值与 32-bit Q12.16 寄存器数值的换算标准如下：

| 物理状态变量 | 实际物理单位 | 连续浮点典型范围 | Q12.16 定点整数计算公式 | 示例物理值 | 对应 16 进制 / 10 进制 Q16 码值 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **摆杆倾角偏差 $\theta_{err}$** | $\text{rad}$ | $[-0.78, +0.78]$ | $\text{Val}_{Q16} = \theta \times 65536$ | $+0.100\,\text{rad} \approx 5.73^\circ$ | `32'd6554` (`0x0000199A`) |
| **摆杆角速度 $\dot{\theta}$** | $\text{rad/s}$ | $[-8.0, +8.0]$ | $\text{Val}_{Q16} = \dot{\theta} \times 65536$ | $-2.50\,\text{rad/s}$ | `-32'sd163840` (`0xFFFD8000`) |
| **转臂角位移 $\alpha_{err}$** | $\text{rad}$ | $[-\infty, +\infty]$ | $\text{Val}_{Q16} = \alpha \times 65536$ | $+1.5708\,\text{rad} \approx 90^\circ$ | `32'd102944` (`0x00019220`) |
| **转臂角速度 $\dot{\alpha}$** | $\text{rad/s}$ | $[-15.0, +15.0]$ | $\text{Val}_{Q16} = \dot{\alpha} \times 65536$ | $+3.1416\,\text{rad/s}$ | `32'd205887` (`0x0003243F`) |
| **LQI 转臂积分 $\int\alpha$** | $\text{rad}\cdot\text{s}$ | $[-0.25, +0.25]$ | $\text{Val}_{Q16} = \text{Int} \times 65536$ | $+0.25\,\text{rad}\cdot\text{s}$ (饱和上限) | `32'd16384` (`0x00004000`) |
| **电机驱动输出 `pwm_duty`**| 无量纲千分比 | $[-1000, +1000]$ | 整数直读（符号位代表转向） | $+500$ (50% 占空比正转) | `16'sd500` (`0x01F4`) |
