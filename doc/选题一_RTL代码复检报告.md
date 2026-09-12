# 选题一 RTL 代码复检报告（第三轮 · 终审）

> **复检对象**：[`d:\Gowin_fpga\edu\project\furuta_lqr_ctrl`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl)
> **复检基线**：git commit `a8be230`（前序 `596e996`、`941fdd8`、`ddb4f60`），工作区干净
> **对标赛题**：[`选题一_基于FPGA的实时姿态控制系统.md`](file:///c:/Users/28399/Desktop/赛道/选题一_基于FPGA的实时姿态控制系统.md)
> **前序报告**：[`选题一_RTL代码合规性检查报告.md`](file:///c:/Users/28399/Desktop/赛道/doc/选题一_RTL代码合规性检查报告.md)（`e41c1b8`，声称"F1~F15 全部彻底整改"）
> **本报告前版**：第二轮复检报告（已随 `e41c1b8` 归档，本文覆盖更新为第三轮结果）
> **复检日期**：2026-09-12
> **证据来源**：源码逐行核查 + Gowin EDA V1.9.12.03 综合/PnR/时序报告实读 + ModelSim transcript 反推验算 + Python 数值独立复现

---

## 〇、复检结论

### 判定：**显著进步但仍未达标 —— 11 项确已修复，起摆主线因新引入的定标错误依然断裂**

本轮整改质量**明显高于上一轮**：F1 的 cos 查找表重建、F4 的 Q16 全精度阻尼、F10 的 DSP 降载（MULT36X36 彻底清零）都做得干净漂亮，F5/F6/F7/F8/F9/F12/F14/F15 亦全部落地并经 TB 实测验证。

但存在两个必须立即处理的问题：

1. **N1【致命】**——F2 的机械能判据**实现了，但定点定标错了 65536 倍**。为省 DSP 而把 θ̇ 预降到 Q12 时，输出移位量未同步调整（`>>> 8` 应为 `>>> 24`）。后果：摆杆角速度一旦超过 **0.067 rad/s**（≈3.8°/s，近乎静止的微动），能量判据即误判为"能量已达标"并**彻底切断泵能**。**基础要求 1（起摆控制）在物理上仍然不可能完成。**
2. **N3【严重】**——时序 Fmax 从上一轮的 **56.305 MHz 退化到 50.001 MHz**，最小 Setup Slack 仅 **0.001 ns（裕量 0.002%）**，最差 4 条路径全部落在新增的 `u_swing_ctrl` 内部。工程上等同于**没有裕量**，不具备交付条件。

而 ModelSim 6 项测试全 PASS 且打印 `[PASS] F2 修复成功`，属**假阳性**——本报告 §2.4 给出逐位反推证明。

### 三轮演进总览

| 维度 | 首轮 `ddb4f60` | 第二轮 `596e996` | **本轮 `a8be230`** | 走势 |
| :--- | :---: | :---: | :---: | :---: |
| P0 工程可下板性 | 0% | 100% | **100%** | ➡️ 保持 |
| 外设驱动 / LQI 核 | 90% | 95% | **98%** | ⬆️ |
| **基础要求 1 · 起摆** | **0%** | **30%** | **35%** | ⬆️ 微升（F1/F4/F6 真修好，但 N1+N2 阻断） |
| 基础要求 2 · 自平衡 | 60% | 85% | **95%** | ⬆️ F7 到位 |
| 基础要求 3 · 抗扰 | 未验证 | 未验证 | **未验证** | ➡️ 无 RTL 闭环证据 |
| 拓展要求 1 · 定点 | 0% | 85% | **90%** | ⬆️ 斜坡已加，N4 前馈偏小 |
| 拓展要求 2 · 轨迹 | 0% | 90% | **95%** | ⬆️ `sync_phase` 正确 |
| 拓展要求 3 · 动态保持 | 0% | 未验证 | **未验证** | ➡️ |
| **时序裕量** | 无约束 | 11.2% | **0.002%** | ⬇️ **严重退化** |
| DSP 资源 | 70% | **100%（满载）** | **88%** | ⬆️ 红线解除 |
| TB 有效性 | 开环 4 项 | 开环 6 项 | 开环 6 项 | ➡️ 仍无动力学模型 |

---

## 一、本轮已确认真实修复的 11 项

### 1.1 F1 —— cos(θ) 查找表索引错误【彻底修复 ✅】

**新实现**（[`swing_up_ctrl.v#L36-L37`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/swing_up_ctrl.v)）：

```verilog
wire [47:0] idx_mult = abs_theta_q16 * 32'd20535;
wire [5:0]  cos_idx  = (abs_theta_q16 >= 32'sd205887) ? 6'd63 : idx_mult[31:26];
```

**映射验算**：`idx = (θ×65536 × 20535) >> 26 = θ × 20535/1024 = θ × 20.0537`，理论值 `63/π = 20.0535`，**误差 0.001%** ✅

**位宽验算**：θ 上限 π → `abs_theta_q16 = 205887`，`205887 × 20535 = 4,227,889,545 < 2³² = 4,294,967,296` ✅ 不溢出；`>> 26` 得 **63.0**，恰好满量程 ✅

**表数据全区间验证**（0~180° 逐度 Python 扫描，与 `numpy.cos` 比对）：

| θ | 索引 | RTL cos | 真实 cos | 偏差 | 上一轮同点 |
| ---: | ---: | ---: | ---: | ---: | :--- |
| 0° | 0 | +1.0000 | +1.0000 | 0.0000 | +1.0000 ✅ |
| 30° | 10 | +0.8782 | +0.8660 | +0.0122 | — |
| 45° | 15 | +0.7330 | +0.7071 | +0.0259 | 0.3127（−0.394）❌ |
| **60°** | 21 | **+0.5000** | +0.5000 | **−0.0000** | **−0.1060（符号翻转）**❌ |
| 90° | 31 | +0.0249 | 0.0000 | +0.0249 | −0.8488 ❌ |
| **120°** | 42 | **−0.5000** | −0.5000 | **−0.0000** | **+0.9891（回绕+翻转）**❌ |
| **150°** | 52 | **−0.8533** | −0.8660 | +0.0128 | **+0.5859（翻转）**❌ |
| 170° | 59 | −0.9802 | −0.9848 | +0.0046 | +0.0548（翻转）❌ |
| **180°** | 63 | **−1.0000** | −1.0000 | **0.0000** | −0.8788 ❌ |

- **0~180° 全区间最大误差 0.04716（4.7%）**，源于 64 点表固有量化（每点跨 2.86°），对能量泵控制完全够用；
- **全区间无一处符号翻转，无索引回绕**；
- `idx=63` 已修正为 `−16384`（= cos π），上一轮的表末端失真已消除。

✅ **这是本轮质量最高的一处修复。**

### 1.2 F3 —— tanh 硬饱和改为 4 段折线【基本修复 ✅，残留 N5】

| x = 20·θ̇·cosθ | 上轮硬饱和 | 本轮折线 | 真实 tanh | 改善 |
| ---: | ---: | ---: | ---: | :---: |
| 0.25 | 0.250（+0.4%） | 0.2305（−5.9%） | 0.2449 | — |
| 0.5 | 0.500（**+8.2%**） | 0.4610（−0.2%） | 0.4621 | ✅ |
| 1.0 | 1.000（**+31.3%**） | 0.7525（−1.2%） | 0.7616 | ✅ |
| 2.0 | 1.000（+3.7%） | 0.9350（−3.0%） | 0.9640 | ✅ |

最大偏差由 **+31.3% 降至 ~6%**；×20 用 `(x<<<4)+(x<<<2)`、×30 用 `(x<<<5)-(x<<<1)`，**零 DSP 开销** ✅。残留段边界跳变见 N5。

### 1.3 F4 —— 转臂居中阻尼全精度 Q16【彻底修复 ✅】

```verilog
// 旧（截断为整数 rad，|α|<1 rad 时恒为 0）
wire signed [31:0] arm_limit_int = arm_limit_q16 >>> 16;

// 新（全程 Q16，L136-L140 / L219-L227）
wire signed [47:0] dalpha_damp_mult = $signed(dalpha_s0) * 32'sd19661;    // 0.3×65536=19660.8 ✅
wire signed [31:0] arm_limit_q16    = alpha_s0 + dalpha_damp_shift[31:0]; // Q16 ✅
wire signed [31:0] raw_acc_q16      = base_acc_q16_r - arm_limit_q16_r2;  // Q16 − Q16 ✅
wire signed [31:0] clamped_acc_q16  = clamp(raw_acc_q16, ±1966080);       // ±30 rad/s² ✅
wire signed [31:0] raw_pwm = ((clamped_acc_q16 <<< 4) - clamped_acc_q16) >>> 16;  // ×15 ✅
```

α = 0.3 rad 时现可正确输出 0.3 rad/s² 回中力（上一轮为 0）；满幅 30 rad/s² × 15 = 450，与输出限幅 ±450 严格自洽 ✅。

### 1.4 F5 / F6 —— FSM 两处【彻底修复 ✅】

| 项 | 修复 | 验证 |
| :--- | :--- | :--- |
| **F5** PROTECT 永久锁死 | [`ctrl_fsm.v#L127-L135`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/ctrl_fsm.v) 新增 `if (!soft_limit_err) current_state <= STATE_HANGING;` | TB TEST 6 实测：长按 KEY2 清零后 `soft_limit_err: 0, current_state: 1`，**自动恢复成功** ✅ |
| **F6** 扰动脉冲 75→250 | [`ctrl_fsm.v#L91`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/ctrl_fsm.v) `final_pwm_duty <= 16'sd250;`（3.0×1000/12 = 250 ✅） | TB 断言加严为 `== 16'sd250`，transcript 打印 `PWM: 250` ✅ |

### 1.5 F7 / F8 / F9 —— 顶层接线三处【彻底修复 ✅】

| 项 | 位置 | 改动与验证 |
| :--- | :--- | :--- |
| **F7** 捕获空窗 | `j280_hw_top.v#L322` | `.calc_en (sample_done)`——去掉 `&& lqr_en`，LQI 核**后台常开**。TB 实测切入平衡后 `PWM: 183`（非 0），空窗消除 ✅ |
| **F8** 相位突跳 | `#L239` + `ctrl_fsm.v#L30,L103` | 新增 `traj_sync_pulse`，SWINGUP→BALANCE 瞬间置 1，`traj_gen` 据此复位 `time_cnt_5s`（正弦零相位起振）；新增 `RAMP_STEP_Q16=572`（0.5°/ms，45° 用 90ms）斜坡发生器；正弦模式回写 `ramp_pos_q16 <= sine_pos_q16` 避免切回定点突变 ✅ 设计考虑周到 |
| **F9** 采样错位 | `#L212` | `u_encoder.calc_en` 改接 `sample_done`，θ/α 采样时刻统一 ✅ |

### 1.6 F10 —— DSP 资源红线解除【修复 ✅】

```
                 上一轮          本轮
DSP              20/20  100%     17.5/20  88%
  MULT36X36      6               0        ← 彻底清零
  MULT18X18      2               7
  MULTALU36X18   4               12
  MULTADDALU18X18 —              1
  ALU54D         4               1
```

通过把 LQI 核与编码器/角度解算的乘法降位宽至 18×18，**腾出约 2.5 个 DSP 余量**，解除容量红线 ✅。

### 1.7 F12 / F14 / F15【全部修复 ✅】

| 项 | 修复证据 |
| :--- | :--- |
| **F12** SDC 异步输入 | [`furuta_lqr_ctrl.sdc`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/furuta_lqr_ctrl.sdc) 新增 `set_false_path -from [get_ports {enc_a enc_b enc_z adc_miso}]`，4 个跨时钟域输入全部纳入约束 ✅ |
| **F14** 阻塞/非阻塞混用 | [`angle_sensor_reader.v#L195`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/angle_sensor_reader.v) `current_theta_q16` 已改为 `wire` + 三元表达式，`raw_dtheta_q16` 阻塞赋值亦消除，时序块内现为纯非阻塞 ✅ |
| **F15** 操作规程缺失 | [`J280套件硬件实物检测与校准实操指南.md`](file:///c:/Users/28399/Desktop/赛道/doc/J280套件硬件实物检测与校准实操指南.md) 新增第〇章「**现场上电与零点标定标准作业流程 (SOP - 必读)**」，五步完整（上电前 `sw_motor_en=0` → 扶直摆杆 → 长按 `KEY1` 约 0.5s 标定 → 转臂归零后短按 `KEY2` 清圈数 → 拨 `sw_motor_en=1` 自动起摆），且**引脚号 T2 / V5 / V4 与 CST 完全一致** ✅ |

---

## 二、N1【致命】机械能判据定点定标错误 65536 倍 → 起摆依然失效

### 2.1 缺陷定位与推导

**位置**：[`swing_up_ctrl.v#L79-L80, L118-L120`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/swing_up_ctrl.v)

```verilog
// Stage 0：为省 DSP，把 θ̇ 预降到 Q12（取 [21:4]，即 ÷16）
wire signed [17:0] dth_in_q12 = ... dtheta_rad_s_q16[21:4];
dth_sq_s0 <= dth_in_q12 * dth_in_q12;                  // 结果尺度 = Q24

// Stage 1：动能
wire signed [47:0] e_kin_scaled = dth_sq_s0 * 18'sd22;
wire signed [47:0] e_kin_shift  = e_kin_scaled >>> 8;  // ← 错误：应为 >>> 24
```

**正确推导**：

```
目标：E_kin_q16 = 0.5·Jp·θ̇² × 65536 = 21.855 × θ̇²  ≈ (22 × θ̇²)
其中 θ̇ = dtheta_q16 / 65536
→ E_kin_q16 = (22 × dtheta_q16²) >>> 32                 （Q16 输入版，即第二轮报告给出的公式）

本轮改用 Q12：dth_in_q12 = dtheta_q16 / 16
→ dth_sq_s0 = dtheta_q16² / 256
→ (22 × dtheta_q16²) >>> 32 = (22 × dth_sq_s0 × 256) >>> 32 = (22 × dth_sq_s0) >>> 24
```

**代码写的是 `>>> 8`，比正确值 `>>> 24` 少移 16 位 → 结果放大 2¹⁶ = 65536 倍。**

> 注：`e_pot` 的定标是**正确的**（`>>> 14` 对应 Q14 的 cos 表），说明错误仅出现在因降位宽而需额外补偿的动能项上——典型的"改了输入尺度、忘了改输出移位"疏漏。

### 2.2 数值验证（Python 逐位复现 RTL 行为）

参数取自 [`config.py`](file:///c:/Users/28399/Desktop/赛道/simulation/config.py)：`Jp = 0.000667`、`m2·g·l2 = 0.050×9.81×0.10 = 0.04905`

| θ̇ (rad/s) | θ | `e_pot`（RTL） | **`e_kin`（RTL `>>8`）** | `e_kin`（正确 `>>24`） | 真实 E_q16 | RTL 判定 | 应判定 |
| ---: | ---: | ---: | ---: | ---: | ---: | :---: | :---: |
| 0 | 180° | −6428 | 0 | 0 | −6429 | 泵能 ✅ | 泵能 |
| **0.067** | 180° | −6428 | **6451** | 0 | −6429 | **停止** ❌ | 泵能 |
| 0.5 | 180° | −6428 | **360,448** | 5 | −6424 | **停止** ❌ | 泵能 |
| 1.0 | 180° | −6428 | **1,441,792** | 22 | −6407 | **停止** ❌ | 泵能 |
| 4.0 | 180° | −6428 | **23,068,672** | 352 | −6079 | **停止** ❌ | 泵能 |
| 8.0 | 180° | −6428 | **1,476,372,480** | 22,527 | −5030 | **停止** ❌ | 泵能 |

- `e_pot` 与真实值 −6429 吻合（误差 1，来自 `>>14` 截断）→ **势能项定标正确** ✅
- `e_kin`（RTL）恰好是正确值的 **65536 倍** → **动能项定标错误** ❌
- **临界点：|θ̇| ≈ 0.067 rad/s（≈3.8°/s）**。超过此值 `e_kin > |e_pot|` → `e_total > 0` → `energy_deficit = 0` → 泵能被门控切断。

### 2.3 实物失效链条

```
① 下垂静止，θ̇ ≈ 0，|θ| = π > 2.5 rad
   → is_dead_center = 1 → base_acc_q16 = +327680 (5.0 rad/s²) → PWM = 74 (0.89V)
② 摆杆/转臂微动，|θ̇| 越过 0.067 rad/s
   → e_kin(错误放大 65536 倍) > |e_pot| → energy_deficit = 0
   → pump_gated_q16 = 0（泵能切断）
③ 此时 is_dead_center 亦不再满足（|θ̇| > 0.05 rad/s）
   → base_acc_q16 = 0
   → raw_acc_q16 = 0 − arm_limit_q16 = 纯阻尼项 → 电机反向制动
④ 摆杆被阻尼拉回下垂静止 → 回到 ①
```

**实物表现**：电机在下垂位置发出轻微抖动或低频嗡鸣，摆杆小幅晃动后回落，**永远无法起摆**。

这与 Python 模型行为完全相反——模型中 `E < 0` 时持续泵能、`E ≥ 0` 时才停止，摆杆能量单调爬升直至进入捕获窗口（`test_suite.py::test_swingup_and_balance` 已验证通过）。

### 2.4 ModelSim「PASS」为假阳性的反推证明

`sim_modelsim/transcript`（23:46:30）TEST 3 输出：

```
起摆稳态输出 PWM: 74
机械能亏损指示 energy_deficit: 1 (期望: 1, 机械能亏损需要泵能 F2)
[PASS] 起摆持续输出动力且机械能判据均有效 (F2 修复成功)!
```

**逐位反推 `PWM = 74` 的来源**：

```
is_dead_center 分支：base_acc_q16 = 327680            (= 5.0 rad/s² × 65536)
转臂 α = 16 counts → arm_limit_q16 = 16 × 102.94 ≈ 1647
raw_acc_q16 = 327680 − 1647 = 326033
raw_pwm     = (326033 × 15) >>> 16 = 4,890,495 >>> 16 = 74     ✅ 完全吻合
```

即 TB 采样时刻系统正处于 **①「死点扰动」分支**，`|θ̇| < 0.05 rad/s`，`e_kin ≈ 0`，`energy_deficit` 自然为 1。

**根因**：TB 是**开环**的——`simulated_adc_data` 固定为 `12'd0`（θ = π 恒定），θ̇ 只会从初始差分跳变单调衰减至 0，**永远不会真正增长**，因此永远碰不到 `|θ̇| > 0.067 rad/s` 的失效区间。

> 这正是第二轮报告 **F13（无动力学闭环 TB）** 所预警危害的**实证**：一个使基础要求 1 完全失效的致命定标错误，被 6 项全 PASS 的测试集完整掩盖，并在报告中写下「F2 修复成功」。**这已是连续第二轮出现"TB 全绿但功能失效"。**

### 2.5 修法

```verilog
// swing_up_ctrl.v#L119
- wire signed [47:0] e_kin_shift = e_kin_scaled >>> 8;
+ wire signed [47:0] e_kin_shift = e_kin_scaled >>> 24;  // Q24×22 → Q16（Q12 平方需补 2^8=256 倍）
```

**改动量：一个字符。**

**验收断言**（必须同时加入 TB，否则无法防回归）：

| 工况 | θ̇ | θ | `e_total_q16` 期望 | `energy_deficit` |
| :--- | ---: | ---: | ---: | :---: |
| 下垂静止 | 0 | 180° | −6428 | 1 |
| 下垂慢摆 | 0.5 rad/s | 180° | ≈ −6423 | **1**（当前 RTL 为 0 ❌） |
| 中速起摆 | 4.0 rad/s | 90° | ≈ −2865 | **1**（当前 RTL 为 0 ❌） |
| 倒立静止 | 0 | 0° | 0 | 0 |
| 高速过顶 | 4.0 rad/s | 0° | ≈ +350 | 0（防过冲，正确切断） |

---

## 三、N2【严重】动能钳位阈值与钳位值不匹配（虚高 16 倍）

**位置**：[`swing_up_ctrl.v#L79-L80`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/swing_up_ctrl.v)

```verilog
wire signed [17:0] dth_in_q12 = (dtheta_rad_s_q16 >  32'sd524287) ?  18'sd131071 :
                                ((dtheta_rad_s_q16 < -32'sd524288) ? -18'sd131072 : dtheta_rad_s_q16[21:4]);
```

| 量 | 数值 | 物理含义 |
| :--- | ---: | :--- |
| 判断阈值 `524287`（Q16） | 8.0 rad/s | 触发钳位的角速度 |
| 钳位值 `131071`（Q12） | **32.0 rad/s** | 钳位后代表的角速度 ← **不匹配** |

当 `8 < |θ̇| < 32 rad/s` 时，钳位把角速度**放大 4 倍**，动能随之虚高 **16 倍**。

**验证**：即使修好 N1（改 `>>>24`），θ̇ = 8 rad/s 时
- 钳位到 131071 → `e_kin = 22527`
- 真实值 `0.5 × 0.000667 × 64 × 65536 = ` **1399**
- **仍虚高 16.1 倍** ❌

**修法**：

```verilog
- (dtheta_rad_s_q16 >  32'sd524287) ?  18'sd131071 :
- ((dtheta_rad_s_q16 < -32'sd524288) ? -18'sd131072 : dtheta_rad_s_q16[21:4]);
+ (dtheta_rad_s_q16 >  32'sd524287) ?  18'sd32767 :
+ ((dtheta_rad_s_q16 < -32'sd524288) ? -18'sd32768 : dtheta_rad_s_q16[21:4]);
```

（±8.0 rad/s 的 Q12 表示为 ±32768）

**改动量：两个常数。** 修后 θ̇=8 → `d12=32767` → `e_kin = (32767²×22)>>24 = 1408`，与真实 1399 吻合（误差 0.6%）✅

> **N1 与 N2 必须同批修复**：只修 N1 则 8 rad/s 以上仍虚高 16 倍；只修 N2 则全域仍虚高 65536 倍。

---

## 四、N3【严重】时序裕量归零（Fmax 56.305 → 50.001 MHz）

### 4.1 实测数据对比

| 指标 | 上一轮 `596e996` | **本轮 `a8be230`** | 变化 |
| :--- | ---: | ---: | :---: |
| Constraint | 50.000 MHz | 50.000 MHz | — |
| **Actual Fmax** | **56.305 MHz** | **50.001 MHz** | ⬇️ **−11.2%** |
| **裕量** | **12.6%** | **0.002%** | ⬇️ **归零** |
| **最小 Setup Slack** | **2.240 ns** | **0.001 ns** | ⬇️ **−99.96%** |
| Logic Level | 17 | **18** | ⬆️ 更深 |
| Setup Violated Endpoints | 0 | 0 | ✅ |
| TNS（Setup / Hold） | 0.000 / 0.000 | 0.000 / 0.000 | ✅ |
| 分析 corner | Slow 0.95V 85C | Slow 0.95V 85C | — |

### 4.2 关键路径全部迁移到新增的 `u_swing_ctrl`

```
Path  Slack   From Node                                  To Node                              Data Delay
 1    0.001   u_swing_ctrl/dth_cos_mult_0_s0/DOUT[32]    u_swing_ctrl/base_acc_q16_r_30_s0/D    19.964 ns
 2    0.070   u_swing_ctrl/dth_cos_mult_0_s0/DOUT[32]    u_swing_ctrl/base_acc_q16_r_31_s0/D    19.895 ns
 3    0.087   u_swing_ctrl/dth_cos_mult_0_s0/DOUT[32]    u_swing_ctrl/base_acc_q16_r_29_s0/D    19.878 ns
 4    0.147   ...
```

上一轮关键路径在 `u_lqr_core/pwm_mult`（slack 2.240 ns），本轮**最差 4 条路径全部迁移到 `u_swing_ctrl`**，slack 压缩至 **0.001 ns**。

### 4.3 根因

Stage 1 的 DSP 输出寄存器 `dth_cos_mult_0_s0/DOUT` 到 Stage 2 的 `base_acc_q16_r` 之间，**串联了整条折线组合链**：

```
DSP 输出 → >>>14 → ×20 移位加 → 绝对值 → 4 段阈值比较 → 折线选择
        → 符号恢复 → ×30 移位减 → 取负 → energy_deficit 门控 mux
        → is_dead_center mux → base_acc_q16_r
```

共 **18 级逻辑**，数据延迟 19.964 ns，占满 20 ns 周期。

[`swing_up_ctrl.v#L12`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/swing_up_ctrl.v) 文件头声称「**全流程 4 级流水线优化，彻底切断乘法器级联，Fmax > 70MHz**」——实测 **50.001 MHz**，**与声称相差 40%**。流水线确实切断了乘法器之间的级联，但**未切断乘法器之后的这段长组合链**。

### 4.4 风险评级

TNS = 0、violations = 0，工具口径"时序收敛"，但 **0.001 ns 裕量在工程上等同于没有裕量**：

- 任何一次重新布局布线（哪怕只改一行无关代码）都可能翻负；
- 实测已在最恶劣 corner（Slow 0.95V / 85C），无进一步降额空间；
- 现场若出现电源纹波（电机启动大电流拉低电源轨，正是 `doc` 中「现象 6：电机一启动 FPGA 复位」记录过的故障模式），时序立即违例 → 起摆核输出错乱 → 电机失控甩杆。

**结论：当前 bitstream 不具备交付/下板条件。**

### 4.5 修法

在 Stage 2 内部再插一拍，把折线链切成两段：

```verilog
// 新增 Stage 1.5：寄存 ×20 结果与绝对值
reg signed [31:0] x_mult_r, abs_x_r;
always @(posedge clk or negedge rst_n)
    if (!rst_n)            begin x_mult_r <= 0; abs_x_r <= 0; end
    else if (pipe_valid1)  begin x_mult_r <= x_mult; abs_x_r <= abs_x; end
// Stage 2 只做：折线查表 → 符号恢复 → ×30 → 门控 mux
```

预期 18 级拆为 9+9 级，Fmax 可回到 **60~70 MHz**，裕量恢复至 20% 以上。代价是起摆核延迟从 4 拍增至 5 拍（100 ns，占 1 ms 控制周期的 0.01%，**无实质影响**）。

---

## 五、N4 / N5【中 / 低】

### N4【中】斜坡速度前馈小 10 倍

**位置**：[`traj_gen.v#L36-L37`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/traj_gen.v)

```verilog
localparam signed [31:0] RAMP_STEP_Q16 = 32'sd572;    // 0.5°/ms，90ms 平滑到 45°
localparam signed [31:0] RAMP_VEL_Q16  = 32'sd57200;  // 注释：对应前馈速度约 0.5 rad/s
```

| 量 | 计算 | 结果 |
| :--- | :--- | ---: |
| 位置步进 | 572 / 65536 | 0.008728 rad/ms = **0.5°/ms** ✅ |
| 45° 走完耗时 | 45 / 0.5 | **90 ms** ✅（注释正确） |
| **实际斜坡角速度** | 0.5°/ms = 500°/s | **8.727 rad/s** |
| 前馈应有 Q16 值 | 8.727 × 65536 | **571,998** |
| 代码 `RAMP_VEL_Q16` | 57200 / 65536 | **0.8728 rad/s** ❌ |

**前馈速度比实际斜坡速度小 10 倍**（注释「约 0.5 rad/s」与两者均不符）。后果：定点伺服期间 `dalpha_err = dalpha − dalpha_ref` 长期为正偏差，前馈补偿不足，转臂跟踪滞后、调节时间延长。

**修法**：`RAMP_VEL_Q16 = 32'sd571998`（**一个常数**）。

> **附带调参建议**：0.5°/ms = 500°/s 的转臂速度对倒立摆偏激进——Python 正弦轨迹峰值速度仅 `TRAJ_VMAX = 0.5483 rad/s`（31°/s）。实物起摆/平衡期间转臂以 500°/s 运动会产生显著反作用力矩，建议现场从 **0.1~0.2°/ms** 起调（`RAMP_STEP_Q16` = 114~229，`RAMP_VEL_Q16` = 114400~228800）。

### N5【低】折线段边界不连续（残留 ~1% 跳变）

| 边界 | 低段输出 | 高段输出 | 跳变 |
| :--- | ---: | ---: | ---: |
| x = 0.5 | 段1 → 0.4610 | 段2 → 0.4713 | **0.0103** |
| x = 1.0 | 段2 → 0.7525 | 段3 → 0.7475 | **0.0050** |

要连续，常数项应为：
- 段2：`0.4621 − 0.5625×0.5 = 0.18084` → Q16 = **11852**（当前 `12452`）
- 段3：`0.7525 − 0.1875 = 0.5650` → Q16 = **37027**（当前 `36700`）

跳变约 1%，相比上一轮硬饱和的 31% 已大幅改善，仅影响起摆平顺度，不阻断功能。**修法：两个常数。**

---

## 六、遗留未修项

### 6.1 F2 的实质达成度

`energy_deficit` 已从硬编码 `1'b1` 改为**真实计算并寄存输出**，顶层 `.energy_deficit(swing_energy_deficit)` 已正确连接（`j280_hw_top.v#L312`），门控 `pump_gated_q16 = energy_deficit_r ? pump_acc_q16 : 32'sd0`（L192）逻辑结构**完全正确**。

**架构对了，数值错了**——仅 N1/N2 两处定标问题。修完后 F2 即可判定达成。

### 6.2 F13 —— TB 有效性【仍未修，且已造成实际后果】

| 问题 | 本轮状态 |
| :--- | :--- |
| `.ARM_SOFT_LIMIT_Q16(32'sd3000)` 参数覆盖 | ❌ **仍存在**（[`j280_hw_top_tb.v#L51`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top_tb.v)），真实阈值 `823548`（±2 圈）**依然从未被验证** |
| 开环激励、无动力学模型 | ❌ 未改。6 项测试仍靠 `simulated_adc_data` 人工设值 + `repeat` 注入编码器脉冲 |
| 断言强度 | ⬆️ 部分改善：TEST 3 加严为 `== 16'sd250`（F6）、新增 `energy_deficit == 1'b1`（F2）、TEST 5 验证斜坡、TEST 6 验证 PROTECT 自恢复 |

**后果已实证**：见 §2.4，N1 这一致命缺陷被 6 项全 PASS 完整掩盖。**这是连续第二轮出现同类问题，应视为最高优先级的流程性缺陷。**

**必须补的两项**：

1. **短期（低成本）**：新增 `swing_up_ctrl` 单元级 TB，直接扫描 θ̇ ∈ {0, 0.5, 1, 2, 4, 6, 8} × θ ∈ {0°, 45°, 90°, 135°, 180°} 共 35 组向量，断言 `energy_deficit` 与浮点模型一致（§2.5 表）。此 TB **无需动力学模型**即可捕获 N1/N2。
2. **中期（根治）**：把 [`dynamics.py`](file:///c:/Users/28399/Desktop/赛道/simulation/dynamics.py) 的欧拉-拉格朗日方程移植为 Verilog 行为级模型（`real` 类型 + RK4，用 `` `ifndef SYNTHESIS `` 包裹不参与综合），构建真正的 HIL 闭环，验证：起摆成功率、起摆耗时、平衡保持时长、抗推力恢复时间。

### 6.3 工程卫生 —— 仿真二进制产物被提交入库

RTL 仓库 `.gitignore` 仅忽略 `sim_run/work/`、`sim_run/transcript` 等旧路径，**未覆盖新增的 `sim_modelsim/`**。该目录（含 `work/_lib*.qdb`、`.qpg`、`.qtl` 等二进制仿真库与 `transcript`）已随 `a8be230` **提交入库**，`git status` 现为干净。

二进制仿真产物入库会持续膨胀仓库且无 diff 价值。**修法**：`.gitignore` 追加

```
sim_modelsim/work/
sim_modelsim/transcript
sim_modelsim/*.ini
sim_modelsim/*.wlf
```

并 `git rm -r --cached sim_modelsim/work sim_modelsim/transcript`（保留本地文件）。

### 6.4 D9 —— 源码多副本【仍未处理】

`sim_run/` 与 [`赛道/simulation/src_verilog/`](file:///c:/Users/28399/Desktop/赛道/simulation/src_verilog) 下的旧副本仍存在，后者**已落后 2 轮**（缺 `swing_up_ctrl.v` / `ctrl_fsm.v` / `traj_gen.v`）。前序报告 L3 把 `simulation/src_verilog` 称为「镜像工程」并列作整改对象，但该目录实际未同步。

好消息：本轮 ModelSim 已改为 `vlog -work work ../src/*.v`（transcript 可证），**直接编译 `src/` 而非副本** ✅。建议直接删除两处过期副本，避免误导。

---

## 七、修正后的达标判定（三轮对照）

| 赛题条目 | 首轮 | 第二轮 | 前序报告声称 | **本轮实测** | 关键依据 |
| :--- | :---: | :---: | :---: | :---: | :--- |
| 设计要求 · 信号采集 | 70% | 90% | 100% | **95%** | SPI/编码器/滤波完备，F9/F12/F14 已修；SPI 帧格式仍未按具体 ADC 型号适配（§十） |
| 设计要求 · 主控计算平台 | 90% | 95% | 100% | **98%** | LQI 核 bit-exact，F7 后台常开，F10 DSP 降至 88% |
| 设计要求 · PWM 执行驱动 | 80% | 100% | 100% | **100%** | D1 三态验证通过 |
| 设计要求 · 摆臂定点调节 | 0% | 85% | 100% | **90%** | KEY2 四模式 + 斜坡已实现；N4 前馈小 10 倍 |
| **基础要求 1 · 起摆控制** | **0%** | **30%** | **100%** | ❌ **35%** | **F1/F4/F6 真修好；但 N1（定标错 65536 倍）+ N2（钳位错 16 倍）使能量门控在 \|θ̇\|>0.067 rad/s 时误切断泵能，起摆物理上仍不可能** |
| 基础要求 2 · 自平衡 | 60% | 85% | 100% | **95%** | 双重捕获判据 + F7 消除空窗，实测切入后 PWM=183 有效 |
| 基础要求 3 · 稳定性抗扰 | 未验证 | 未验证 | 100% | ⚠️ **未验证** | F13：无动力学闭环 TB，抗扰无 RTL 级证据 |
| **拓展要求 1 · 定点位置** | **0%** | 85% | 100% | **90%** | 0.5°/ms 斜坡 + 90ms 到位已验证；N4 前馈偏差 |
| **拓展要求 2 · 轨迹跟踪** | **0%** | 90% | 100% | **95%** | 128 点 LUT 验算正确，`sync_phase` 零相位同步到位 |
| **拓展要求 3 · 动态姿态保持** | **0%** | 未验证 | 100% | ⚠️ **未验证** | 依赖基础要求 1 与闭环验证，两者均未达成 |
| **工程可下板性** | 0% | 100% | 100% | ⚠️ **85%** | CST/SDC/顶层/bitstream 均正常；**N3 时序裕量 0.002%，不可交付** |
| 硬件资源 | 70% | ⚠️ 满载 | 100% | ✅ **100%** | DSP 88%，Logic 5%，Register 2%，Latch = 0 |

---

## 八、与前序报告声称不符之处

| # | 前序报告（`e41c1b8`）声称 | 复检实况 |
| :---: | :--- | :--- |
| 1 | L8「F1~F15 共 15 项缺陷…**现已全部彻底整改并代码落地**」 | 11 项确已修复；**F2 实现但定标错误（N1/N2），实质未达成**；F13 未修 |
| 2 | L25「**基础要求 1 · 起摆控制 = 100%**」，依据「实现定点机械能判据 E 并门控泵能（F2）」 | **约 35%**。能量判据定点缩放错 65536 倍，`|θ̇|>0.067 rad/s` 即误判能量达标并切断泵能，**起摆物理上不可能完成** |
| 3 | L8 / L31「时序完全收敛，**Fmax ≥ 50.00MHz**，Slack 全部为正」 | 字面成立但**掩盖了从 56.305 MHz 退化至 50.001 MHz、Slack 从 2.240 ns 压缩至 0.001 ns** 的事实。0.002% 裕量不构成工程可交付状态 |
| 4 | `swing_up_ctrl.v#L12`「4 级流水线优化，**Fmax > 70MHz**」 | 实测 **50.001 MHz**，相差 40% |
| 5 | L8「ModelSim 6 大顶层系统测试用例 **100% PASS**」作为 F2 修复证据 | 该 PASS 为**假阳性**（§2.4 已逐位反推）。开环 TB 结构上无法覆盖 N1 的失效区间 |
| 6 | L27「基础要求 3 · 稳定性抗扰 = **100%**」，依据「TB 硬性断言覆盖」 | TEST 4 验证的是「ADC 码值突变为 2600 时状态机回退到 SWINGUP」，属**状态转移逻辑测试**，非抗扰能力测试。仍**未验证** |
| 7 | L30「拓展要求 3 · 动态姿态保持 = **100%**」 | 依赖基础要求 1 与闭环验证，两者均未达成 → **未验证** |

---

## 九、整改清单（按实施顺序，含精确改动量）

### 第一批：恢复起摆功能（必须同批完成）

| # | 缺陷 | 文件 / 行 | 改动量 | 验收方式 |
| :---: | :---: | :--- | :---: | :--- |
| 1 | **N1** | `swing_up_ctrl.v#L119` | **1 个字符**：`>>> 8` → `>>> 24` | §2.5 断言表 5 组工况 |
| 2 | **N2** | `swing_up_ctrl.v#L79-80` | **2 个常数**：`131071`→`32767`、`131072`→`32768` | θ̇=8 rad/s 时 `e_kin ≈ 1408`（真值 1399） |
| 3 | **F13-a** | 新增 `swing_up_ctrl_tb.v` | 新建单元 TB | 扫描 θ̇×θ 共 35 组向量，断言 `energy_deficit` 与浮点模型一致 |

### 第二批：恢复时序裕量（阻断交付）

| # | 缺陷 | 文件 / 行 | 改动量 | 验收方式 |
| :---: | :---: | :--- | :---: | :--- |
| 4 | **N3** | `swing_up_ctrl.v` Stage 2 | **插入 1 级寄存器**（`x_mult_r` / `abs_x_r`）+ 对应 `pipe_valid` | 重跑 PnR，要求 **Fmax ≥ 60 MHz、最小 Slack ≥ 3 ns** |

### 第三批：精度与品质

| # | 缺陷 | 文件 / 行 | 改动量 |
| :---: | :---: | :--- | :---: |
| 5 | **N4** | `traj_gen.v#L37` | **1 个常数**：`57200` → `571998` |
| 6 | **N5** | `swing_up_ctrl.v#L178,L180` | **2 个常数**：`12452`→`11852`、`36700`→`37027` |
| 7 | 斜坡速率调参 | `traj_gen.v#L36` | 建议 `RAMP_STEP_Q16` 由 572 降至 114~229（0.1~0.2°/ms），现场实测后定 |

### 第四批：验证体系与工程卫生

| # | 项 | 内容 |
| :---: | :--- | :--- |
| 8 | **F13-b** | 移植 `dynamics.py` 为 Verilog 行为级被控对象模型（`real` + RK4，`` `ifndef SYNTHESIS ``），构建 HIL 闭环；验证起摆成功率 / 耗时 / 平衡保持时长 / 抗推力恢复时间 |
| 9 | **F13-c** | TB 增加**不覆盖参数**的软限位用例：注入 ≥8000 脉冲，验证真实阈值 `823548` |
| 10 | 工程卫生 | `.gitignore` 追加 `sim_modelsim/` 规则，`git rm -r --cached` 移除已入库的二进制仿真产物 |
| 11 | **D9** | 删除 `sim_run/` 与 `赛道/simulation/src_verilog/` 下的过期源码副本 |

> **回归要求**：每批修改后必须重跑「ModelSim 全测试集 + Gowin 综合 + PnR + 时序分析」四步，并把 **Fmax / 最小 Slack / DSP 占用**三项数值写入报告。特别注意：第一批与第二批都会改变 `swing_up_ctrl` 的组合逻辑深度，**必须重新确认时序**。

---

## 十、待确认的硬件信息（下板前必须澄清）

以下 5 项经三轮检查仍无法从代码与文档推断：

1. **板载晶振是否确为 50 MHz** —— 全部模块 `CLK_FREQ_HZ` 按 50 MHz 参数化。若实为 27/24 MHz，则 1ms 节拍、SPI 2.5MHz 分频、PWM 20kHz 载波**全部偏移**，且 N3 的时序裕量将进一步恶化。
2. **摆杆角度传感器确切型号** —— [`angle_sensor_reader.v#L130`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/angle_sensor_reader.v) 仍硬编码 `adc_latch <= shift_reg[11:0]`，且无 MOSI/命令字输出：
   - 若为 **MCP3201**：首 bit 为 null，12 位数据落在 `shift_reg[12:1]` → **当前位对齐错误**；
   - 若为 **AS5048A / TLE5012B**：**必须发送命令字**，当前单向 SPI 无法工作；
   - 若为 **ADS7886**：12 位 MSB-first 紧接 CS 下降沿，需核对是否多采 4 bit。
3. **电机驱动是否确为 TB6612FNG** —— 决定 `stby_out = motor_en | brake_mode` 的正确性与 STBY 引脚连接。
4. **J280 底板原理图 / 引脚分配表** —— CST 现用 20 个引脚在 PG484 封装中**合法**（PnR 已接受、Bank 2/6 的 `BankVccio = 3.3` 与 `LVCMOS33` 匹配、`clk_50m` 落在 `GCLKT_2` 专用时钟脚），但**是否对应底板实际连线无法由 EDA 工具验证**。前序报告称依据「高云 3PA1030 核心板原理图与 DS102 数据手册」编写——DS102 是**芯片级**数据手册，不含底板连线信息，需确认引脚映射的真实出处。
5. **电机供电是否确为 12 V** —— `VOLT_TO_PWM = 5461163` 与 `swing_up_ctrl` 的 `×15` 系数（= `J0/(km×12V)×1000`）均基于 12V。若为 24V，输出增益翻倍，两者需同步重算。

---

## 附录 A：F1 cos 查找表验证脚本

```python
import numpy as np, re
src = open(r"...\src\swing_up_ctrl.v", encoding="utf-8").read()
tbl = {int(m.group(1)): int(m.group(2)+"1")*int(m.group(3))
       for m in re.finditer(r"6'd(\d+):\s*cos_lut_q14\s*=\s*(-?)\s*16'sd(\d+)", src)}
w = 0
for deg in range(0, 181):
    q16 = int(round(np.radians(deg)*65536))
    idx = 63 if q16 >= 205887 else ((q16*20535) >> 26) & 0x3F
    w = max(w, abs(tbl[idx]/16384.0 - np.cos(np.radians(deg))))
print("table entries:", len(tbl), " max err:", w)
# 输出： table entries: 64   max err: 0.04716
```

## 附录 B：N1 能量定标验证脚本与输出

```python
Jp = 0.000667; m2gl2 = 0.050*9.81*0.10          # 取自 config.py
for dth in [0.0, 0.067, 0.5, 1.0, 4.0, 8.0]:
    dq16 = int(round(dth*65536))
    d12  = 131071 if dq16 > 524287 else (-131072 if dq16 < -524288 else (dq16 >> 4) & 0x3FFFF)
    sq   = d12*d12
    e_kin_rtl = (sq*22) >> 8      # 当前代码
    e_kin_fix = (sq*22) >> 24     # 正确移位
    cos_q14   = tbl[63]           # θ = 180°
    e_pot     = ((cos_q14-16384)*3214) >> 14
    E_true_q16 = (0.5*Jp*dth**2 + m2gl2*(np.cos(np.pi)-1.0))*65536
```

输出：

```
dth=0.000 th=180: e_pot= -6428  e_kin_RTL(>>8)=          0  e_kin_fix(>>24)=    0  trueE_q16=-6429.1
dth=0.067 th=180: e_pot= -6428  e_kin_RTL(>>8)=       6451  e_kin_fix(>>24)=    0  trueE_q16=-6429.0
dth=0.500 th=180: e_pot= -6428  e_kin_RTL(>>8)=     360448  e_kin_fix(>>24)=    5  trueE_q16=-6423.6
dth=1.000 th=180: e_pot= -6428  e_kin_RTL(>>8)=    1441792  e_kin_fix(>>24)=   22  trueE_q16=-6407.2
dth=4.000 th=180: e_pot= -6428  e_kin_RTL(>>8)=   23068672  e_kin_fix(>>24)=  352  trueE_q16=-6079.4
dth=4.000 th= 90: e_pot= -3134  e_kin_RTL(>>8)=   23068672  e_kin_fix(>>24)=  352  trueE_q16=-2864.8
dth=4.000 th=  0: e_pot=     0  e_kin_RTL(>>8)=   23068672  e_kin_fix(>>24)=  352  trueE_q16= +349.7
dth=8.000 th=180: e_pot= -6428  e_kin_RTL(>>8)= 1476372480  e_kin_fix(>>24)=22527  trueE_q16=-5030.3
```

判读：`e_pot` 与 `trueE_q16`（θ̇=0 行）吻合 → 势能项正确；`e_kin_RTL` 恒为 `e_kin_fix` 的 **65536 倍** → 动能项移位错误。

## 附录 C：工具实测输出摘录

**C1 综合（`impl/gwsynthesis/furuta_lqr_ctrl.log`）**

```
Analyzing Verilog file '...\src\furuta_lqr_ctrl.v'      （8 个文件全部 Analyzing）
Compiling module 'j280_hw_top' / 'swing_up_ctrl' / 'ctrl_fsm' / 'traj_gen' / ...
NOTE  (EX0101) : Current top module is "j280_hw_top"
GowinSynthesis finish                    （无 WARN / 无 ERROR）
```

**C2 布局布线（`impl/pnr/furuta_lqr_ctrl.log`）**

```
Reading constraint file: "...\src\furuta_lqr_ctrl.cst"
Physical Constraint parsed completed
Running timing analysis...... [95%] Timing analysis completed
Bitstream generation completed
```

**C3 资源占用（`impl/pnr/furuta_lqr_ctrl.rpt.txt`）**

```
Logic      | 2390/54720  |  5%   (1669 LUT, 721 ALU, 0 ROM16)
Register   |  760/42000  |  2%   (Latch = 0 ✅ 无意外锁存器)
CLS        | 1390/27360  |  6%
I/O Port   |   20/320    |  7%   ✅ 与顶层端口数一致
DSP        | 17.5/20     | 88%   ✅ (MULT18X18×7, MULTALU36X18×12, MULTADDALU18X18×1, ALU54D×1)
```

**C4 时序（`impl/pnr/furuta_lqr_ctrl_tr_content.html`）**

```
Max Frequency Summary:
  Clock     Constraint      Actual Fmax     Logic Level   Entity
  clk_50m   50.000(MHz)     50.001(MHz)     18            TOP
Total Negative Slack: Setup 0.000 (0 endpoints) / Hold 0.000 (0 endpoints)
Setup Paths Table (最差 4 条):
  1  0.001  u_swing_ctrl/dth_cos_mult_0_s0/DOUT[32] → u_swing_ctrl/base_acc_q16_r_30_s0/D  Delay 19.964
  2  0.070  u_swing_ctrl/dth_cos_mult_0_s0/DOUT[32] → u_swing_ctrl/base_acc_q16_r_31_s0/D  Delay 19.895
  3  0.087  u_swing_ctrl/dth_cos_mult_0_s0/DOUT[32] → u_swing_ctrl/base_acc_q16_r_29_s0/D  Delay 19.878
  4  0.147  ...
Setup Delay Model: Slow 0.95V 85C C8/I7
```

**C5 ModelSim（`sim_modelsim/transcript`，End time 23:46:30）**

```
vlog -work work ../src/*.v          ← 已直接编译 src/，不再使用副本 ✅
[TEST 1] 零点标定            → PASS  (zero_offset_reg: 2048)
[TEST 2] 编码器 4 倍频        → PASS  (pulse_count: 16)
[TEST 3] FSM 起摆→平衡        → PASS  (state: 1, PWM: 250)                   ← F6 验证通过 ✅
                                PASS  (state: 1, PWM: 74, energy_deficit: 1)  ← N1 假阳性，见 §2.4
                                PASS  (dtheta: 2343, state: 2, PWM: 183)      ← F7 验证通过 ✅
[TEST 4] 跌落回退 SWINGUP     → PASS  (state: 1)
[TEST 5] KEY2 切模式 + 斜坡   → PASS  (traj_mode: 1, alpha_ref 0 → 51472)     ← F8 验证通过 ✅
[TEST 6] 软限位 + 自恢复      → PASS  (state: 3 → 清零后 state: 1)             ← F5 验证通过 ✅
                                 （注：ARM_SOFT_LIMIT_Q16 被覆盖为 3000，真实阈值未验证 ❌）
Errors: 0, Warnings: 0
```

## 附录 D：三轮检查时间线

| 轮次 | RTL commit | doc commit | 时间 | 结论 |
| :---: | :--- | :--- | :--- | :--- |
| 首轮 | `ddb4f60` | — | 09-11 22:33 | 不满足：起摆/拓展要求全缺，P0 四项阻断 |
| 第二轮 | `596e996` | `0bf5312` | 09-11 23:29 | 部分达标：P0 真实解决；提出 F1~F15 共 15 项缺陷 |
| **第三轮** | **`a8be230`** | **`e41c1b8`** | **09-12** | **11 项已修；N1（致命）/N2/N3 三项新缺陷，起摆仍未达成** |

---

## 终审结论重申

本轮整改**方向正确、执行质量明显提升**：F1 的 cos 表重建（含 `20535/1024 ≈ 63/π` 的无除法单调映射）与 F4 的全程 Q16 改造体现了扎实的定点数工程能力；F10 把 DSP 从满载降至 88%、MULT36X36 彻底清零，解除了容量红线；F5/F6/F7/F8/F9/F12/F14/F15 八项均干净落地并经 TB 实测验证。

但三个问题使其**尚不具备验收条件**：

1. **N1 + N2 使基础要求 1 依然断裂。** 机械能判据的架构、门控、连线全部正确，唯独在"为省 DSP 而把 θ̇ 降到 Q12"时漏调了输出移位量，导致动能被放大 65536 倍。摆杆只要有 3.8°/s 的微动，系统就误判"能量已足"并停止泵能，转而施加阻尼把摆杆拉回下垂——**起摆永远无法启动**。修复仅需 **1 个字符 + 2 个常数**。

2. **N3 使 bitstream 不可交付。** 最小 Setup Slack 0.001 ns，最差 4 条路径全部集中在新增的起摆核内，实测 Fmax 50.001 MHz 与源码头注声称的「>70MHz」相差 40%。在电机启动电流拉低电源轨的现场环境下，此时序裕量等同于失效。修复需 **插入 1 级流水线寄存器**。

3. **F13 是流程性根因，必须优先补齐。** 上述两个致命问题本应在仿真阶段暴露，却因 TB 全为开环激励而连续两轮被 PASS 掩盖。**在建立 `swing_up_ctrl` 单元级向量 TB（短期）与动力学 HIL 闭环（中期）之前，任何「测试全部通过」的结论都不应作为验收依据。**

**建议下一步**：按 §九 第一批（N1 + N2 + 单元 TB）→ 第二批（N3 + 重跑 PnR）顺序推进，两批合计代码改动量约 **10 行**，预期即可使基础要求 1 从 35% 跃升至可实物验证状态。
