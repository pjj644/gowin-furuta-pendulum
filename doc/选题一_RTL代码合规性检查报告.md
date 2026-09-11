# 选题一 RTL 代码合规性检查报告

> **检查对象**：[`d:\Gowin_fpga\edu\project\furuta_lqr_ctrl`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl)
> **对标文件**：[`选题一_基于FPGA的实时姿态控制系统.md`](file:///c:/Users/28399/Desktop/赛道/选题一_基于FPGA的实时姿态控制系统.md)
> **目标芯片**：高云 GW2A-LV55PG484C8/I7（J280 姿态控制系统竞赛套件）
> **代码基线**：git commit `ddb4f60`，工作区干净
> **检查日期**：2026-09-11

---

## 一、总体判定：**不满足**

当前 RTL 只实现了题目算法链的**中段**（自平衡 LQI 计算核 + 三个外设驱动模块），存在三类根本性缺口：

1. **基础要求 1（摆杆起摆控制）完全缺失**，且现有安全保护逻辑与起摆需求**结构性冲突**；
2. **拓展要求 1 / 2 / 3 全部缺失**，转臂目标位置被硬编码为常量 0；
3. **工程层面当前无法下板**：顶层模块设错、引脚约束文件为空、无时序约束、bitstream 产物已过期。

Python 仿真侧（[`simulation/controller.py`](file:///c:/Users/28399/Desktop/赛道/simulation/controller.py)）的算法是**完整的**——能量泵起摆、HANGING/SWINGUP/BALANCE 状态机、LQI、正弦轨迹跟踪全部实现并通过 6 项测试——但**没有任何一行被移植到 Verilog**。当前 FPGA 工程相当于只做了整套系统的"平衡执行内核"。

### 完成度概览

| 赛题条目 | 完成度 | 阻断原因 |
| :--- | :---: | :--- |
| 设计要求 · 信号采集 | 70% | SPI 帧格式未按实际 ADC 型号适配 |
| 设计要求 · 主控计算平台 | 90% | LQI 核 bit-exact 达标 |
| 设计要求 · PWM 执行驱动 | 80% | TB6612 刹车逻辑失效 |
| 设计要求 · 摆臂定点调节 | 0% | 目标位置硬编码为 0 |
| **基础要求 1 · 起摆控制** | **0%** | **无算法 + 保护逻辑冲突** |
| 基础要求 2 · 自平衡控制 | 60% | 缺模式切换与捕获逻辑 |
| 基础要求 3 · 稳定性抗扰 | 未验证 | RTL 侧无闭环被控对象仿真 |
| **拓展要求 1 · 定点位置控制** | **0%** | **无设定输入通道** |
| **拓展要求 2 · 轨迹速度跟踪** | **0%** | **无轨迹生成器与前馈** |
| **拓展要求 3 · 动态姿态保持** | **0%** | **依赖拓展要求 1** |
| 工程可下板性 | 0% | 顶层/CST/SDC/bitstream 四项全缺 |

---

## 二、赛题要求逐条对照

### 2.1 设计要求

| 要求项 | 状态 | 证据与说明 |
| :--- | :---: | :--- |
| 角度传感器实时采集摆杆倾角 | ⚠️ 部分满足 | [`angle_sensor_reader.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/angle_sensor_reader.v) 已实现硬件 SPI 主机状态机（2.5MHz）、360° 解卷绕（L156-L168）、垂直零点扣减、1ms 差分测速 + 一阶 IIR 滤波。**但**数据提取硬编码为 `adc_latch <= shift_reg[11:0]`（L130），且无 MOSI / 配置命令字输出，未按具体芯片型号适配——ADS7886、MCP3201、AS5048A、TLE5012B 的帧长度、首位 null bit、数据位对齐方式互不相同，实测极可能读到错位数据 |
| 编码器采集电机转速与旋转方向 | ✅ 满足 | [`encoder_quad_reader.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/encoder_quad_reader.v) 双级 D 触发器防亚稳态（L38-L52）+ 8 拍积分消抖滤波（L62-L89）+ 严格 4 倍频鉴相（L116-L125）+ 1ms M 法测速 + IIR 平滑。1000 线 × 4 倍频 = 4000 CPR，分辨率 0.09°，符合题目"微秒级精确位置与转速"要求 |
| FPGA 内部实现在线控制算法 | ✅ 满足 | [`furuta_lqr_ctrl.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/furuta_lqr_ctrl.v) 3 级硬件乘加流水线，Q12.16 定点，确定性延迟 60ns。**已实测核对增益与 Python CARE 求解结果 bit-exact 一致**：`K_q16 = [-5388751, -625050, -422594, -296554, -131072]`，对应浮点 `[-82.2258, -9.5375, -6.4483, -4.5250, -2.0000]`；`VOLT_TO_PWM = 5461163` 与 `controller.py#L349` 一致 |
| 输出 PWM 驱动直流电机 | ⚠️ 部分满足 | [`motor_pwm_driver.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/motor_pwm_driver.v) 20kHz 超音频载波 + 无毛刺影子寄存器（L70-L85），双模式引脚兼容 PWM/DIR 与 IN1/IN2。**但**刹车逻辑存在缺陷（见 §4 缺陷 D1） |
| 稳态下摆臂定点位置调节 | ❌ 不满足 | [`j280_hw_top.v#L175`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top.v) `wire signed [31:0] alpha_target_q16 = 32'sd0;` 为硬编码常量，无任何外部设定通道 |

### 2.2 基础要求

#### 基础要求 1：摆杆起摆控制 —— ❌ **完全缺失（最高优先级缺口）**

- **无起摆算法**：工程中不存在能量泵（Energy-based Swing-up）、Bang-Bang 变结构或任何等效逻辑；`src/` 下无 `swing_up_ctrl.v`。
- **无状态机**：题目技术要点明确要求"状态切换控制器（起摆 / 平衡 / 定位 / 保护）"，当前顶层是纯组合式信号直通，无 FSM。Python 侧的 `STATE_HANGING / STATE_SWINGUP / STATE_BALANCE`（`controller.py#L55-L57`）未移植。
- **结构性冲突（关键）**：[`j280_hw_top.v#L188-L189`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top.v) 定义跌落保护

  ```verilog
  localparam signed [31:0] FALL_ZONE_Q16 = 32'sd51472;   // 约 45 度
  wire is_fall_down = (theta_err_q16 > FALL_ZONE_Q16) || (theta_err_q16 < -FALL_ZONE_Q16);
  ```

  并在 L231 强制门控 `safe_pwm_duty = (sw_motor_en && !is_fall_down) ? lqr_pwm_duty : 16'sd0;`。
  摆杆自然下垂时 θ ≈ 180°，远超 45° 阈值 → **电机被永久封锁，系统无法输出任何起摆动力**。即使后续补上起摆算法，若不把该门控改为"仅 BALANCE 态生效"，起摆依然不可能成功。
- **绝对角度不可用**：摆杆下垂时 ADC 差值落在 ±2048 counts 的解卷绕边界上，`theta_err_q16` 会在 +π / −π 之间因噪声跳变，而起摆能量泵依赖 `cos(θ)` 与 `sign(θ̇·cosθ)` 的连续性。移植时必须先解决大角度区间的符号连续性。

#### 基础要求 2：摆杆自平衡控制 —— ⚠️ **部分满足**

- **已具备**：5 状态 LQI 全状态反馈（θ、θ̇、α、α̇、∫α），纯硬件流水线，控制周期 1ms（1000Hz），Q12.16 定点，饱和限幅 ±1000（±12V）。
- **缺失**：
  - 无"起摆 → 平衡"自动捕获切换，必须**人工手扶摆杆至直立**后系统才能工作，不满足题目"实时姿态控制"的自主性要求；
  - [`j280_hw_top.v#L185`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top.v) 的平衡区判定 `is_in_balance` **仅用角度阈值 22°，缺少角速度判据**。Python 模型要求 `abs(th) < 22° AND abs(dth) < 4.0 rad/s` 双重条件（`controller.py#L272`、`config.py#L57-L58`），硬件窗口比模型宽松，可能捕获到高速扫过平衡区的摆杆而导致冲力过大无法刹停。

#### 基础要求 3：稳定性与抗扰能力 —— ⚠️ **未在硬件层验证**

| 子项 | 题目要求 | RTL 验证状态 |
| :--- | :--- | :--- |
| 起摆后缓慢停住、无明显震荡 | 要求 | ❌ 无起摆，无从验证 |
| 长时间保持稳定不倒 | 要求 | ❌ RTL 侧无闭环被控对象仿真，无长时间稳定性证据 |
| 抵抗轻微外部推力并短时恢复 | 要求 | ❌ 仅 Python `test_suite.py::test_disturbance_rejection` 通过，非 RTL 结果 |

现有 ModelSim 测试（`sim_run/transcript`，`Errors: 0, Warnings: 0`）全部为**开环阶跃激励**，TB 中没有倒立摆动力学模型，因此"摆杆能否立住"这一核心指标在硬件描述层面**完全没有被验证过**。

### 2.3 拓展要求

| # | 要求 | 状态 | 缺口说明 |
| :---: | :--- | :---: | :--- |
| 1 | 定点位置控制（键盘/拨码开关设定目标位置，平滑移动并保持静止） | ❌ | 顶层无键盘、无拨码开关、无串口指令任何目标输入端口；`alpha_target_q16` 恒为 0。LQI 的 K3/K5 项虽已具备位置伺服能力（积分分离 + 抗饱和限幅 ±0.25），但只能伺服到固定零点 |
| 2 | 轨迹与速度跟踪（按特定速度曲线或轨迹运动） | ❌ | 无轨迹生成器；Python 的正弦参考 `traj_amp=25°`、`traj_freq=0.2Hz`（`config.py#L75-L76`）未移植。更关键的是 LQR 第 4 路输入直接接 `dalpha_rad_s_q16`（[`j280_hw_top.v#L224`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top.v)），**缺少 `dalpha_ref` 速度前馈**——Python 模型用的是 `dalpha_err = dalpha - dalpha_ref`（`controller.py#L318`），硬件缺此项则动态轨迹跟踪必然滞后 |
| 3 | 动态姿态保持（移动过程中摆杆始终直立） | ❌ | 依赖拓展要求 1，未实现 |

---

## 三、阻断级工程问题（P0 —— 当前 bitstream 完全不可用）

### E1. 引脚约束文件为空

[`src/furuta_lqr_ctrl.cst`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/furuta_lqr_ctrl.cst) **0 字节**。PnR 报告 `impl/pnr/furuta_lqr_ctrl.rpt.txt` 明确记录：

```
<Physical Constraints File>: ---
<Timing Constraints File>: ---
I/O Port | 180/320 | 57%
```

所有引脚由工具随机分配，`Constraint` 列全部为 `N`。烧录后引脚与 J280 底板实际连线**完全不匹配**，电机、编码器、ADC、按键全部无法通信。

### E2. 综合顶层模块设错

`impl/gwsynthesis/furuta_lqr_ctrl.log` 记录：

```
NOTE (EX0101) : Current top module is "furuta_lqr_ctrl"
```

而 `impl/gwsynthesis/furuta_lqr_ctrl.prj` 的 FileList **只包含 1 个源文件**：

```xml
<FileList>
    <File path="...\src\furuta_lqr_ctrl.v" type="verilog"/>
</FileList>
```

即 `motor_pwm_driver.v`、`encoder_quad_reader.v`、`angle_sensor_reader.v`、`j280_hw_top.v` **四个模块从未参与过综合**。这也是 PnR 报告中 `theta_err[0..31]`、`calc_en` 等内部控制信号全部被当作物理引脚、占满 180 个 I/O 的原因。

文档 [`旋转倒立摆仿真算法与实物调参全指南.md#L408`](file:///c:/Users/28399/Desktop/赛道/doc/旋转倒立摆仿真算法与实物调参全指南.md) 已写明"设置 `j280_hw_top` 为 Top Module"，但工程中**未执行**。

### E3. bitstream 产物严重过期

| 文件 | 最后修改时间 |
| :--- | :--- |
| `impl/pnr/furuta_lqr_ctrl.fs`（bitstream） | **2026/9/10 21:12:14** |
| `impl/gwsynthesis/furuta_lqr_ctrl.vg`（网表） | 2026/9/10 21:11:59 |
| `src/motor_pwm_driver.v` | 2026/9/11 17:51:44 |
| `src/encoder_quad_reader.v` | 2026/9/11 17:59:08 |
| `src/angle_sensor_reader.v` | 2026/9/11 17:59:25 |
| `src/j280_hw_top.v` | **2026/9/11 18:03:06** |

现有 `.fs` 生成于 4 个外设模块创建**之前**，与当前源码无任何对应关系。即使烧录也只会得到一个"裸露的 LQI 计算核"。

### E4. 无时序约束 + 多处大位宽除法（最大实现风险）

工程无 `.sdc` 文件，Gowin 综合选项 `global_freq = 100.000` 仅为默认值，PnR 未做时序驱动优化。设计中存在多处需在 **50MHz（20ns）单周期内完成**的除法：

| 位置 | 表达式 | 风险 |
| :--- | :--- | :--- |
| `encoder_quad_reader.v#L196` | `alpha_rad_q16 <= pulse_mult_64 / K_ANGLE_DENOM;` | **64 位有符号除法**，组合逻辑深度极大，资源占用高，时序几乎必然不收敛 |
| `encoder_quad_reader.v#L204` | `... * FILTER_ALPHA) / 32'sd100` | 32 位除法 |
| `angle_sensor_reader.v#L218` | `... * FILTER_ALPHA) / 32'sd100` | 32 位除法 |
| `j280_hw_top.v#L202-L207` | `alpha_err_q16 / 32'sd1000`（×3 处比较） | 32 位除法，且在积分限幅判断中重复计算 3 次 |
| `motor_pwm_driver.v#L65` | `(duty_compensated * TIMER_PERIOD) / DUTY_MAX_VAL` | 32 位除法（常数除，可优化但未优化） |

**整改方向**：全部改为「乘法倒数 + 移位」。例如 `/4000` → `× 167773 >> 26`；`/100` → `× 65536 / 100 = × 655 >> 16`；`/1000` → `× 67 >> 16`（需做精度误差核算）；PWM 的 `/1000 × 2500` 直接化简为 `× 5 >> 1`。

---

## 四、代码级缺陷清单

### D1【高】TB6612 动态刹车功能永久失效

[`motor_pwm_driver.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/motor_pwm_driver.v) 中：

```verilog
assign stby_out = motor_en;          // L49：使能与待机直接绑定
...
end else if (!motor_en) begin        // L113：停机分支
    if (brake_mode) begin
        in1_out <= 1'b1;             // L119-L120：试图短接刹车
        in2_out <= 1'b1;
    end
```

按 TB6612FNG 真值表，`STBY = 0` 时芯片进入待机、输出高阻并**忽略 IN1/IN2**。而刹车逻辑恰恰只在 `!motor_en`（即 `STBY = 0`）分支内生效 → **刹车模式永远不可能触发**。

后果：摆杆跌落保护动作时（`motor_active = 0`），转臂只会自由滑行而非能耗制动，`sw_brake_mode` 拨码开关形同虚设。

**整改**：`stby_out` 应恒为 `1'b1`（或由独立的系统级急停信号控制），使刹车时 STBY 保持有效。

### D2【高】积分分离条件与算法模型不一致

| | 判据 | 出处 |
| :--- | :--- | :--- |
| Python 模型 | `abs(alpha_err) < 10°` 时累加积分，否则**清零** | `controller.py#L322-L326` |
| RTL 实现 | `is_in_balance`（`abs(theta_err) < 22°`）时累加积分 | `j280_hw_top.v#L200` |

两者门控变量完全不同（转臂位置误差 vs 摆杆倾角）。RTL 在大位移定点移动过程中会持续累加积分，与仿真验证过的抗饱和特性不符，实物可能出现低频超调晃动。

### D3【中】转臂位置无多圈卷绕处理

[`j280_hw_top.v#L176`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top.v) `alpha_err_q16 = alpha_rad_q16 - alpha_target_q16;`

旋转倒立摆的水平转臂可**连续单向多圈旋转**，`pulse_count` 为 32 位无界累加。起摆阶段若转臂朝一个方向甩多圈，`alpha_err` 将达到数十 rad，K3/K5 项彻底主导输出并长期饱和，摆杆平衡被位置环强行夺权。Python 模型同样无界，但仿真时长短、且起摆中已叠加转臂软限位项 `-(1.0α + 0.3α̇)`（`controller.py#L292`）而未暴露此问题。

**整改**：移植起摆软限位项，或对 `alpha_err` 做 ±π 卷绕 / 幅值钳位。

### D4【中】编码器 Z 相零位复位未实现

[`encoder_quad_reader.v#L40, L50`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/encoder_quad_reader.v) 中 `z_sync_reg` 被声明并完成两级同步采样，但**全模块无任何消费者**。文件头注释第 3 条声称"支持掉电/按键归零与 Z 相零位复位"，实际仅实现了按键 `clear_pos` 归零。属于**注释与实现不符**，且会引入综合优化警告。

### D5【中】`speed_pps` 位宽截断溢出

[`encoder_quad_reader.v#L193`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/encoder_quad_reader.v)

```verilog
speed_pps <= delta_pulse[15:0] * 16'sd1000;
```

16 位 × 16 位结果截断回 16 位。`delta_pulse` 仅需超过 32 就会溢出（32 × 1000 = 32000 已接近 16 位有符号上限 32767），而 1ms 内 32 脉冲对应约 2.9 rad/s，属正常工作区间。该输出当前未被顶层使用，但一旦用于监控或 GAO 观测将读到错误值。

### D6【中】θ 与 α 状态采样时刻错位

| 通道 | 触发源 | 时刻 |
| :--- | :--- | :--- |
| 编码器测速 / 测角 | `calc_en_pulse`（`j280_hw_top.v#L163`） | 1ms 网格整点 |
| LQI 计算 | `sample_done`（`j280_hw_top.v#L192`） | 1ms + SPI 传输 ≈ 7.2us |

两组状态量相差约 7us（0.72% 控制周期）。当前精度下影响有限，但在高速动态轨迹跟踪（拓展要求 2）时会引入相位偏差。建议统一由 `sample_done` 触发全部状态锁存。

### D7【低】`is_in_balance` 缺角速度判据

见 §2.2 基础要求 2。建议补充 `&& (dtheta_q16 < OMG_THRESH) && (dtheta_q16 > -OMG_THRESH)`，阈值取 4.0 rad/s（Q16 = 262144）。

### D8【低】综合截断警告未处理

```
WARN (EX3791) : Expression size 64 truncated to fit in target size 32
    ("...\src\furuta_lqr_ctrl.v":80)
```

对应 `v_cmd_q16 <= -((prod1 + prod2 + prod3 + prod4 + prod5) >>> 16);`。64 位累加和右移 16 位后仍可能超出 32 位范围（理论上限 ≈ 5 × 82 × 大状态值）。虽然第 3 级已有 PWM 饱和限幅兜底，但 `v_cmd_q16` 本身若卷绕，限幅判据将读到错误符号。建议显式增加中间饱和或位宽核算注释。

### D9【低】源码副本导致版本漂移风险

[`sim_run/furuta_lqr_ctrl.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/sim_run/furuta_lqr_ctrl.v) 与 [`sim_run/furuta_lqr_ctrl_tb.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/sim_run/furuta_lqr_ctrl_tb.v) 是 `src/` 下同名文件的物理副本。当前内容一致，但无同步机制，后续修改 RTL 时极易出现"仿真跑的是旧版本"。同理 [`赛道/simulation/src_verilog/`](file:///c:/Users/28399/Desktop/赛道/simulation/src_verilog) 下还存有第三份副本。建议改为 `run.do` 中直接指向 `../src/` 编译。

---

## 五、仿真验证覆盖度评估

### 5.1 RTL 侧（ModelSim）

| 测试文件 | 覆盖内容 | 评价 |
| :--- | :--- | :--- |
| [`furuta_lqr_ctrl_tb.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/furuta_lqr_ctrl_tb.v) | 6 个静态向量：零输入、±0.02rad 对称性、±0.5rad 饱和、多状态耦合；校验 3 拍流水线延迟 | 定点算术与限幅验证充分，**但为纯开环静态向量，无动态闭环** |
| [`j280_hw_top_tb.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top_tb.v) | 4 项集成测试：零点标定、编码器 4 倍频计数、LQI 闭环响应、跌落保护 | 外设互联验证有效，**但判据过松，掩盖了真实问题** |

**`transcript` 中 TEST 3 的隐藏问题**：

```
[TEST 3] 模拟摆杆轻微倾斜 (2060, 偏角约 +1.05度)
  当前状态量 -> theta_err: 1206 (Q16), alpha_err: 1647 (Q16)
  LQR 控制器输出占空比 pwm_duty: 1000          <-- 已完全饱和（满 12V）
  [PASS] LQI 控制器闭环运算与电机驱动输出正常响应
```

TB 判据仅为 `lqr_pwm_duty != 0`，因此**1.05° 微小倾角导致输出满占空比饱和**这一异常被判定为 PASS。

根因分析：1ms 差分测速将 1 LSB 量化（4096 counts / 360° = 0.088°）放大为

$$\Delta\dot{\theta} = \frac{0.001534\ \text{rad}}{0.001\ \text{s}} = 1.534\ \text{rad/s}$$

乘以 `K2 = -9.5375` 得到 **14.6V** 的电压指令，已超过 12V 供电上限。一阶 IIR（35%）只能衰减不能消除，稳态下 ADC LSB 交替抖动仍会产生数伏级指令抖振。

**建议**：实物调试时用高云 GAO 在线逻辑分析仪观测 `dtheta_q16` 的静态抖动幅度；必要时提高角度传感器位数 / 过采样、加强 D 通道滤波，或对 `K2·dtheta` 项单独限幅。此项在 Python 闭环仿真中因控制作用抑制了角度跳变而未暴露，**实物风险需专门验证**。

### 5.2 Python 侧（算法模型，非 RTL）

[`simulation/test_suite.py`](file:///c:/Users/28399/Desktop/赛道/simulation/test_suite.py) 6 项测试全部通过：

| # | 测试 | 对应赛题 |
| :---: | :--- | :--- |
| 1 | 物理动力学能量守恒（RK4 精度） | 模型可信度 |
| 2 | 下垂起摆 → 倒立自平衡捕获 | 基础要求 1、2 |
| 3 | 抗外力推力扰动 | 基础要求 3 |
| 4 | 转臂定点位置伺服（0° → 45°） | 拓展要求 1、3 |
| 5 | 连续正弦轨迹与速度跟踪 | 拓展要求 2 |
| 6 | Q12.16 定点 ↔ 浮点闭环等效性 | 定点化正确性 |

**这些结果证明算法方案本身是可行且达标的，但验证对象是 Python 数值模型，不能作为 FPGA 硬件达标的证据。** TEST 6 的定点等效性验证确实覆盖了 LQI 核的算术路径（这也是本报告确认 K 值 bit-exact 的依据），但起摆、状态机、轨迹跟踪三项的定点化验证**尚未进行**。

---

## 六、整改路线与优先级

### P0 —— 打通下板通路（阻断项，必须先做）

1. Gowin 工程中将 **`j280_hw_top` 设为 Top Module**，确认综合 FileList 包含全部 5 个源文件；
2. 编写 [`furuta_lqr_ctrl.cst`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/furuta_lqr_ctrl.cst)：按 J280 底板原理图分配 18 个端口（时钟、复位、2 拨码、2 按键、5 电机、3 编码器、3 SPI ADC、3 LED）；
3. 新增 `.sdc` 时序约束：`create_clock -name clk_50m -period 20.0`，并对编码器 A/B/Z、ADC MISO 等异步输入设置 `set_false_path` 或 `set_max_delay`；
4. **消除全部 32/64 位除法**，改为乘法倒数 + 移位（见 §3 E4 整改方向）；
5. 重新综合 + PnR，确认时序收敛、资源占用合理，并核对综合警告清零。

### P1 —— 补齐基础要求 1（起摆）

6. 新增 `swing_up_ctrl.v`：移植 Python 平滑自适应 Lyapunov 能量泵（`controller.py#L280-L294`），含相对能量计算 `E = 0.5·Jp·θ̇² + m2·g·l2·(cosθ − 1)`、`tanh` 连续化平滑因子（`swing_gamma=2.5`）、转臂软限位项 `-(1.0α + 0.3α̇)`、加速度限幅 `±30 rad/s²`、下垂死点扰动脉冲；
7. 新增 `ctrl_fsm.v`：HANGING / SWINGUP / BALANCE / PROTECT 四态机，捕获窗口 `|θ| < 22° AND |θ̇| < 4.0 rad/s`，切入时清零积分器，`|θ| > 45°` 时回落 SWINGUP；
8. **改造跌落保护门控**：`is_fall_down` 仅在 BALANCE 态封锁 PWM，SWINGUP 态必须放行大角度动力输出；
9. 解决大角度区 `theta_err_q16` 在 ±π 边界的符号跳变问题（起摆能量计算依赖 θ 连续性）。

### P2 —— 补齐拓展要求 1 / 2 / 3

10. 新增目标位置输入通道：拨码开关（4~8 位）或 4×4 矩阵键盘或 UART 指令，映射为 `alpha_target_q16`；
11. 新增 `traj_gen.v` 正弦轨迹发生器（幅值 25°、频率 0.2Hz，CORDIC 或查表法），输出 `alpha_ref` 与 `dalpha_ref`；
12. LQI 第 3、4 路输入改为误差形式 `alpha - alpha_ref`、`dalpha - dalpha_ref`，接入速度前馈。

### P3 —— 缺陷修复与验证加固

13. 修复 D1（STBY 冲突）、D2（积分分离条件）、D3（多圈卷绕）、D4（Z 相）、D5（`speed_pps`）、D6（采样时刻对齐）、D7（角速度判据）、D8（截断警告）、D9（源码副本）；
14. **建立闭环 HIL 仿真**：将 [`simulation/dynamics.py`](file:///c:/Users/28399/Desktop/赛道/simulation/dynamics.py) 的欧拉-拉格朗日动力学移植为 Verilog 行为级 testbench 模型（可用实数运算，不参与综合），使 RTL 能真正验证"起摆成功率、平衡时长、抗扰恢复时间"三项核心指标；
15. 强化 TB 断言：TEST 3 应校验 `pwm_duty` 的**数值区间**而非仅判非零，避免饱和被误判为 PASS；
16. 配置高云 GAO 在线逻辑分析仪，观测 `theta_err_q16`、`dtheta_q16`、`alpha_rad_q16`、`pwm_duty`、`mode` 五个信号的实时波形，用于实物调参。

---

## 七、待用户确认事项

以下信息无法从现有代码与文档推断，需确认后方可推进 P0 第 2 步与 SPI 适配：

1. **板载晶振频率**是否确为 50MHz？（当前所有模块的 `CLK_FREQ_HZ` 均按 50MHz 参数化，若实际为 27MHz / 24MHz，1ms 节拍、SPI 分频、PWM 频率将全部偏移）
2. **摆杆角度传感器的实际型号与接口**：是「电位器 + SPI ADC」（ADS7886 / MCP3201 / TLC549）还是「SPI 绝对式磁编码器」（AS5048A / AS5600 / TLE5012B）？两者帧格式差异极大，直接决定 `angle_sensor_reader.v` 的位对齐与是否需要 MOSI 命令字。
3. **电机驱动芯片型号**：是否为 TB6612FNG？（决定 D1 的整改方式与 STBY 引脚连接）
4. **J280 底板引脚分配表 / 原理图**：编写 CST 必需。
5. **编码器线数**是否确为 1000 线（4 倍频后 4000 CPR）？
6. **电机供电电压**是否确为 12V？（`VOLT_TO_PWM = 5461163` 基于 12V ↔ 1000 计数换算）

---

## 附录：证据索引

| 类别 | 文件 |
| :--- | :--- |
| RTL 源码 | `src/j280_hw_top.v`、`src/furuta_lqr_ctrl.v`、`src/angle_sensor_reader.v`、`src/encoder_quad_reader.v`、`src/motor_pwm_driver.v` |
| RTL 测试 | `src/j280_hw_top_tb.v`、`src/furuta_lqr_ctrl_tb.v`、`sim_run/transcript` |
| 工程配置 | `furuta_lqr_ctrl.gprj`、`src/furuta_lqr_ctrl.cst`（空）、`impl/gwsynthesis/furuta_lqr_ctrl.prj` |
| 综合 / PnR 产物 | `impl/gwsynthesis/furuta_lqr_ctrl.log`、`impl/pnr/furuta_lqr_ctrl.rpt.txt`、`impl/pnr/furuta_lqr_ctrl.fs` |
| 算法参考模型 | `赛道/simulation/controller.py`、`config.py`、`dynamics.py`、`test_suite.py`、`fpga_fixed_point_guide.py` |
| 技术文档 | `赛道/doc/旋转倒立摆仿真算法与实物调参全指南.md`、`doc/J280套件外设接口Verilog设计与原理详解.md`、`doc/J280套件硬件实物检测与校准实操指南.md`、`doc/旋转倒立摆实物闭环控制调参全流程指南.md` |
