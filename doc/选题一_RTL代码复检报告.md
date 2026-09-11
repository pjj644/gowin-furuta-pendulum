# 选题一 RTL 代码复检报告（第二轮）

> **复检对象**：[`d:\Gowin_fpga\edu\project\furuta_lqr_ctrl`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl)
> **复检基线**：git commit `596e996`（前序 `941fdd8`、`ddb4f60`）
> **对标文件**：[`选题一_基于FPGA的实时姿态控制系统.md`](file:///c:/Users/28399/Desktop/赛道/选题一_基于FPGA的实时姿态控制系统.md)
> **前序报告**：[`选题一_RTL代码合规性检查报告.md`](file:///c:/Users/28399/Desktop/赛道/doc/选题一_RTL代码合规性检查报告.md)（已更新至 `0bf5312`，声称"完全满足 · 100% 合规达标"）
> **复检日期**：2026-09-11
> **证据来源**：源码逐行核查 + Gowin EDA V1.9.12.03 综合/PnR/时序报告实读 + ModelSim transcript 实读 + Python 数值独立验算

---

## 〇、复检结论

### 判定：**部分达标 —— P0 工程阻断项已真实解决，基础要求 1（起摆控制）实质未实现**

前序报告"完全满足 · 100% 合规达标"的结论**不成立**，应下修为：

| 维度 | 复检判定 |
| :--- | :--- |
| P0 工程可下板性 | ✅ **真实达成**（本轮最大且确实的进展，全部证据可复核） |
| 外设驱动与 LQI 计算核 | ✅ 质量良好，D1/D2/D4/D5/D7/D8 确已修复 |
| **基础要求 1 · 起摆控制** | ❌ **未达成**——存在 2 处致命缺陷（F1 查表索引错误、F2 能量判据缺失），能量泵在物理上无法把摆杆甩起 |
| 基础要求 2 · 自平衡 | ⚠️ 基本达成，存在捕获空窗风险（F7） |
| 基础要求 3 · 稳定性抗扰 | ⚠️ **仍无 RTL 级闭环证据**（F13） |
| 拓展要求 1/2/3 | ⚠️ 算法实现正确，但均未验证，且存在参考值突跳风险（F8） |
| 资源与时序余量 | ⚠️ DSP **100% 占满**（F10）、时序裕量仅 **11.2%**（F11）——两条容量红线 |

**核心矛盾**：本轮整改在"工程通路"上是扎实的（引脚、时序、除法器、顶层全部落地并可复现），但在"起摆算法移植"上存在**声称与实现不符**的情况——前序报告 L70 声称"计算瞬时机械势能 E_pot 与动能 E_kin"，代码中并不存在任何能量计算；L71 与 L73 声称的 tanh 平滑与转臂居中阻尼，实现均与声称的数学形式不等价。

---

## 一、已确认真实修复的项（复核通过）

### 1.1 P0 阻断级工程问题 —— 4/4 全部解决

| 编号 | 原问题 | 复检证据 | 结论 |
| :---: | :--- | :--- | :---: |
| E1 | CST 文件 0 字节，无任何引脚约束 | [`furuta_lqr_ctrl.cst`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/furuta_lqr_ctrl.cst) 现 106 行；PnR 报告 `<Physical Constraints File>` 正确指向该文件；20 个端口 `Constraint` 列全部为 `Y` | ✅ |
| E2 | 综合顶层为 `furuta_lqr_ctrl`，4 个外设模块未参与综合 | 综合 log：`NOTE (EX0101) : Current top module is "j280_hw_top"`；8 个模块全部 `Compiling module`；[`furuta_lqr_ctrl.gprj`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/furuta_lqr_ctrl.gprj) 已注册 8 个 RTL + cst + sdc | ✅ |
| E3 | bitstream 产物过期（早于源码） | `.fs` 生成于 `2026/9/11 23:10:29`，晚于最后一个源文件 `j280_hw_top_tb.v`（23:10:07）与 `j280_hw_top.v`（23:09:29） | ✅ |
| E4 | 无 SDC + 6 处 32/64 位运行时除法 | SDC 已生效（PnR 报告 `<Timing Constraints File>` 正确列出，`clk_50m` 周期 20.000ns 生效）；除法全部消除，见下表 | ✅ |

**E4 除法器消除的数学复核**（逐处独立验算，全部正确）：

| 位置 | 原实现 | 新实现 | 验算 |
| :--- | :--- | :--- | :--- |
| `encoder_quad_reader.v#L175,L184,L187` | `pulse_mult_64 / 4000`（**64 位除法**） | `× 6746406 >>> 16` | 2π×65536/4000 = 102.94370；102.94370×65536 = **6746406** ✅ |
| `encoder_quad_reader.v#L177,L186` | `(...) * 35 / 100` | `× 22938 >>> 16` | 0.35×65536 = **22938** ✅ |
| `angle_sensor_reader.v#L183,L194` | `(...) * 35 / 100` | `× 22938 >>> 16` | 同上 ✅ |
| `motor_pwm_driver.v#L70-L72` | `(duty × 2500) / 1000`（运行时除法） | `localparam SCALE_FACTOR_Q16 = (TIMER_PERIOD*65536)/DUTY_MAX_VAL` 编译期求值 = 163840，再 `>>16` | 1000×163840 = 1.6384e8 < 2³² ✅；`>>16` 得 2500 ✅ |
| `j280_hw_top.v#L274` | `alpha_err / 1000`（×3 处比较） | `(x>>>10)+(x>>>16)+(x>>>17)` | 1/1024+1/65536+1/131072 = **0.00099945**，对 0.001 误差 **0.055%** ✅ |
| `angle_sensor_reader.v#L220` | `× 32'sd1000` | 未改（本就是乘法，非除法） | 无需整改 ✅ |

### 1.2 时序收敛 —— 实测通过

`impl/pnr/furuta_lqr_ctrl_tr_content.html` 实测数据：

```
Numbers of Setup Violated Endpoints : 0
Numbers of Hold  Violated Endpoints : 0
Total Negative Slack (Setup)        : 0.000   Endpoints: 0
Total Negative Slack (Hold)         : 0.000   Endpoints: 0
Max Frequency Summary:
  Clock     Constraint      Actual Fmax     Logic Level   Entity
  clk_50m   50.000(MHz)     56.305(MHz)     17            TOP
Setup Delay Model : Slow 0.95V 85C C8/I7   （最恶劣 corner，结果可信）
```

综合 log 无任何 `WARN`/`ERROR`，原 `EX3791`（64→32 位截断）警告已消失。

### 1.3 代码级缺陷 —— 6/9 确已修复

| 编号 | 原问题 | 复检证据 | 结论 |
| :---: | :--- | :--- | :---: |
| D1 | TB6612 刹车永久失效（`stby_out = motor_en` 与刹车分支互斥） | [`motor_pwm_driver.v#L53`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/motor_pwm_driver.v) 改为 `assign stby_out = motor_en \| brake_mode;`。三态验证：运行→STBY=1；停机+刹车→STBY=1 且 IN1=IN2=1（短路制动生效）；停机+滑行→STBY=0（高阻） | ✅ 正确 |
| D2 | 积分分离用摆角 22° 而非转臂误差 10° | [`j280_hw_top.v#L270,L276`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top.v) `INT_ERR_THRESH_Q16 = 11439`（0.1745 rad = 10.0°✅），门控改为 `alpha_err_q16` 区间判定，与 `controller.py#L322` 对齐；且 `reset_integral_pulse` 在捕获瞬间清零 | ✅ 正确 |
| D4 | `z_sync_reg` 无消费者 | [`encoder_quad_reader.v#L132-L143`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/encoder_quad_reader.v) 新增 `USE_Z_INDEX` 参数 + `z_rise` 上升沿检测，顶层传 `.USE_Z_INDEX(1)`；综合 log 显示 `encoder_quad_reader(USE_Z_INDEX=1)` 已参数化编译 | ✅ |
| D5 | `speed_pps` 16 位截断溢出 | 扩为 32 位（`speed_pps <= 32'sd0` / `delta_pulse * 32'sd1000`） | ✅ |
| D7 | 平衡区仅判角度 | [`ctrl_fsm.v#L52`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/ctrl_fsm.v) `can_enter_balance = (abs_th < 25166) && (abs_dth < 262144)`，即 22°+4.0 rad/s 双重判据，与 `config.py#L57-L58` 一致 | ✅ 正确 |
| D8 | EX3791 截断警告 | [`furuta_lqr_ctrl.v#L75-L89`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/furuta_lqr_ctrl.v) 新增 64 位中间量 `v_cmd_64` + ±13107200（±200V）显式饱和；警告已消失 | ✅ |
| D3 | 转臂无多圈保护 | 已加软限位 `ARM_SOFT_LIMIT_Q16 = 823548`（±2 圈 = ±12.566 rad ✅） | ⚠️ 实现但引入 F5 |
| D6 | θ 与 α 采样时刻错位 | **未真正修复**，见 F9 | ❌ |
| D9 | 源码多副本漂移 | `sim_run/` 与 `src/` 副本仍存在，新增 `sim_modelsim/`（未跟踪） | ⚠️ 部分 |

### 1.4 结构性冲突已解除（原报告最关键的一条）

原顶层 `is_fall_down` 在 |θ|>45° 时全局封锁 PWM，导致下垂态永远无法起摆。现 [`ctrl_fsm.v#L92-L104`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/ctrl_fsm.v) 中 `STATE_SWINGUP` 分支直接输出 `pwm_swing_duty`，**不受跌落判据门控**；跌落检测 `is_fall_down` 仅在 `STATE_BALANCE` 分支内生效并回退至 SWINGUP（L108-L111）。

✅ **此项整改方向完全正确，是本轮的实质性架构改进。**

### 1.5 `traj_gen.v` 算法实现正确（拓展要求 2）

独立验算 128 点正弦/余弦 LUT 与索引映射：

| 校验点 | 期望 | 实测 | 结论 |
| :--- | :--- | :--- | :---: |
| `idx_calc = (time_cnt × 26844) >> 20`，128/5000×2²⁰ | 26843.5 | 26844（误差 <0.0002%） | ✅ |
| `time_cnt` 0~4999 @1ms | 5 s 周期 = 0.2 Hz | 与 `config.py#L76` 一致 | ✅ |
| idx=16 → sin | sin(45°)=0.7071 | 11585/16384 = 0.70709 | ✅ |
| idx=8 → sin | sin(22.5°)=0.3827 | 6270/16384 = 0.38269 | ✅ |
| idx=32 → cos | cos(π/2)=0 | 0 | ✅ |
| idx=64 → cos | cos(π)=−1 | −16384/16384 = −1.0 | ✅ |
| `TRAJ_AMP_Q16 = 28594` | 25° = 0.43633 rad | 0.436325 | ✅ |
| `TRAJ_VMAX_Q16 = 35933` | A·2πf = 0.5483 rad/s | 0.548295 | ✅ |
| `sine_pos_mult >>> 14`（Q16×Q14→Q30→Q16） | 尺度正确 | ✅ | ✅ |

**注意**：该模块的 LUT 索引映射是**正确的**——恰好反证了 `swing_up_ctrl.v` 的同类映射是错误的（见 F1），两者出自同一轮整改却质量不一致。

---

## 二、致命缺陷（阻断基础要求 1）

### F1【致命】`swing_up_ctrl.v` 的 cos(θ) 查找表索引映射错误

**位置**：[`swing_up_ctrl.v#L36`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/swing_up_ctrl.v)

```verilog
wire [5:0] cos_idx = (abs_theta_q16 >= 32'sd205887) ? 6'd63 : abs_theta_q16[16:11];
```

**根因**：`abs_theta_q16[16:11]` 等价于 `floor(θ_rad × 32)`，索引步进为 **1/32 rad = 1.79°**；而 64 点表覆盖 0~π 的隐含步进是 **π/63 = 2.86°**。两者不匹配，导致读出的 cos 值对应错误角度。更严重的是当 θ ≥ 2.0 rad（114.6°）时，索引值 ≥64 被 6 位位宽**截断回绕**，映射到完全无关的表项。

**数值验证**（Python 独立复现 RTL 位级行为，逐点比对真实 cos）：

| θ | RTL 索引 | RTL cos 值 | 真实 cos θ | 偏差 | 判定 |
| ---: | ---: | ---: | ---: | ---: | :--- |
| 0° | 0 | 1.0000 | 1.0000 | +0.0000 | ✅ |
| 5° | 2 | 0.9951 | 0.9962 | −0.0011 | ✅ 可用 |
| 10° | 5 | 0.9697 | 0.9848 | −0.0151 | ⚠️ |
| 20° | 11 | 0.8558 | 0.9397 | −0.0839 | ⚠️ |
| 30° | 16 | 0.7015 | 0.8660 | −0.1645 | ❌ |
| 45° | 25 | 0.3127 | 0.7071 | −0.3944 | ❌ |
| **60°** | 33 | **−0.1060** | +0.5000 | −0.6060 | ❌ **符号翻转** |
| **90°** | 50 | **−0.8488** | 0.0000 | −0.8488 | ❌ **完全错误** |
| 114° | 63 | −0.8788 | −0.4067 | −0.4720 | ❌ |
| **120°** | **3** | **+0.9891** | −0.5000 | +1.4891 | ❌ **索引回绕 + 符号翻转** |
| **150°** | **19** | **+0.5859** | −0.8660 | +1.4520 | ❌ **索引回绕 + 符号翻转** |
| **170°** | **30** | **+0.0548** | −0.9848 | +1.0396 | ❌ **索引回绕 + 符号翻转** |
| 179° | 35 | −0.2128 | −0.9998 | +0.7870 | ❌ |
| 180° | 63 | −0.8788 | −1.0000 | +0.1212 | ❌ |

**表数据本身末端亦失真**：`idx=63` 存 `−14398`（−0.8788），而 cos(π) 应为 `−16384`（−1.0）；`idx=57` 为表内最小值 `−15396`（−0.9397），此后回升，说明表并非 cos(0~π) 的均匀采样。

**后果**：起摆控制律为 `a_pump = −30·sat(20·θ̇·cosθ)`，方向因子依赖 cos θ 的符号。摆杆起摆必须从下垂 180° 经 90° 摆至 0°，而上表显示 **θ > 55° 后 cos 值已严重失准、θ > 114° 后符号相反**。即：**在起摆行程的大部分区间，能量泵在向系统抽取能量（阻尼）而非注入能量（泵能）**，摆杆不仅甩不起来，反而会被主动抑制在下垂位置附近。

**修法**：
```verilog
// π/63 的 Q16 值 = 3.14159265/63*65536 = 3271.6 → 用除法或倒数乘法
wire [6:0] idx_lin = (abs_theta_q16 * 32'd63) / 32'd205887;   // 编译期常数，可优化
// 或改用 128 点表 + 与 traj_gen.v 相同的正确映射方式
```
同时按 `cos(k·π/63), k=0..63` 重新生成表值（`idx=63` 必须为 `−16384`）。

---

### F2【致命】机械能判据 E 完全未实现，泵能永不停止

**位置**：[`swing_up_ctrl.v#L159`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/swing_up_ctrl.v)

```verilog
assign energy_deficit = 1'b1; // 内部常态指示
```

**核查结果**：

1. 全模块**不存在**任何 `E_pot`、`E_kin`、`E` 的计算逻辑——`0.5·Jp·θ̇²` 与 `m2·g·l2·(cosθ−1)` 两项均未实现；
2. `energy_deficit` 被**硬编码为常量 1**，且不参与任何内部决策；
3. 顶层例化时 `.energy_deficit ()` **悬空未接**（[`j280_hw_top.v#L309`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top.v)）。

**与算法模型对比**（[`controller.py#L283-L290`](file:///c:/Users/28399/Desktop/赛道/simulation/controller.py)）：

```python
if E < 0:
    smooth_sign = np.tanh(20.0 * dth * np.cos(th)) if abs(dth) > 0.03 else 0.0
    ...
    a_pump = -cfg.swing_acc_amp * smooth_sign
else:
    a_pump = 0.0          # <-- 能量达标即停止泵能，RTL 完全缺失此分支
```

**后果**：RTL 中泵能**无条件持续进行**。当摆杆机械能已到达或超过倒立点（E ≥ 0）时，Python 模型会停止泵能 letting 摆杆自然减速进入捕获窗口；RTL 则继续全力注入能量 → **摆杆必然过冲**，以远超 4.0 rad/s 的角速度扫过平衡区，被 `can_enter_balance` 的角速度判据拒绝捕获，反复冲过而永不停留。

这直接违反基础要求 1 的验收条件："**摆杆最终实现停在垂直位置附近**"，以及基础要求 3 的"平衡后摆杆会缓慢停在垂直位置附近，**无明显震荡**"。

**与前序报告的偏差**：前序报告 L70 明确声称"64 点对称 cosθ 查找表**计算瞬时机械势能 E_pot 与动能 E_kin**"——**该描述与代码不符**。cos 表仅用于方向因子，未参与任何能量计算。

**修法**（Q16 定标已完整推导，可直接实施）：

目标式 `E = 0.5·Jp·θ̇² + m2·g·l2·(cosθ − 1)`，取 `config.py` 参数 `Jp = 0.000667`、`m2 = 0.050`、`g = 9.81`、`l2 = 0.10`：

| 项 | 浮点系数 | 推导 | Q16 实现 |
| :--- | :--- | :--- | :--- |
| 动能 | `0.5·Jp = 0.0003335` | θ̇ 输入为 Q16，故 θ̇² 为 Q32，需 `>>>32` 还原；再 ×65536 升到 Q16 → 系数 `0.0003335×65536 = 21.855 ≈ 22` | `(22 * dtheta_q16 * dtheta_q16) >>> 32` |
| 势能 | `m2·g·l2 = 0.04905` | ×65536 → `3214`；cosθ 来自 **Q14** 表，故 `(cosθ−1) = (cos_q14 − 16384)/16384`，需 `>>>14` | `(3214 * (cos_q14 - 16384)) >>> 14` |

```
E_q16 = ((22 * dtheta_q16 * dtheta_q16) >>> 32) + ((3214 * (cos_q14 - 32'sd16384)) >>> 14)
energy_deficit = (E_q16 < 0)
```

**边界自检**（用于验证实现正确性）：

| 工况 | θ̇ | θ | E_q16 期望 | 物理含义 |
| :--- | ---: | ---: | ---: | :--- |
| 下垂静止 | 0 | π | **−6429** | E<0，需泵能 ✅ |
| 倒立静止 | 0 | 0 | **0** | E=0，停止泵能 ✅ |
| 高速过顶 | 8 rad/s | 0 | **+1397** | E>0，必须停止泵能（防过冲）✅ |

E_q16 全程范围 `[−6429, +1397]`，32 位余量充足，无溢出风险。

随后在 `base_pump_acc` 处门控：`energy_deficit ? pump_shifted : 32'sd0`（对应 Python 的 `else: a_pump = 0.0`）。

> **注意 DSP 容量**：动能项的 `dtheta_q16²` 需 1 个 32×32 乘法器，势能项的 `3214×(...)` 为 32×16（可映射至 MULT18X18）。F10 显示 DSP 已 100% 占满，**新增这 1~2 个乘法器将无法综合**，必须先按 §7 第 1 步完成 DSP 置换。

---

## 三、严重缺陷

### F3【严重】tanh 平滑饱和被替换为硬线性饱和

**位置**：[`swing_up_ctrl.v#L121-L123`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/swing_up_ctrl.v)

```verilog
wire signed [31:0] sat_in_mult = (dth_cos_q16_r <<< 4) + (dth_cos_q16_r <<< 2);  // ×20 ✅
wire signed [31:0] sat_in_q16  = (sat_in_mult >  32'sd65536) ?  32'sd65536 :
                                 ((sat_in_mult < -32'sd65536) ? -32'sd65536 : sat_in_mult);
```

×20 的移位实现（16+4）正确，但随后是 **`clamp(±1.0)` 硬线性饱和**，而非声称的 `tanh`。

| x = 20·θ̇·cosθ | tanh(x) | RTL sat(x) | 偏差 |
| ---: | ---: | ---: | ---: |
| 0.1 | 0.0997 | 0.100 | +0.3% |
| 0.5 | 0.4621 | 0.500 | **+8.2%** |
| 1.0 | 0.7616 | 1.000 | **+31.3%** |
| 2.0 | 0.9640 | 1.000 | +3.7% |

**后果**：在 x∈[0.3, 1.5] 这一常用区间，RTL 泵能强度比算法模型高 8%~31%，**丧失了 tanh 的核心价值——小信号区的连续平滑过渡**。前序报告 L17 与文档均称"平滑自适应连续化能量泵（tanh 连续过渡，消除冲击）"，与实现不符。实物表现为起摆后期机械抖振加剧。

**修法**：tanh 可用分段线性近似（3~4 段折线）或 `x/(1+|x|)` 有理近似（需 1 个除法器，与 E4 冲突）；推荐**分段线性查表**，与现有 LUT 架构一致，零 DSP 开销。

---

### F4【严重】转臂居中阻尼在 |α| < 1 rad 区间完全失效

**位置**：[`swing_up_ctrl.v#L88-L89`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/swing_up_ctrl.v)

```verilog
wire signed [31:0] arm_limit_q16 = alpha_rad_q16 + dalpha_damp_shifted[31:0];  // Q16 rad
wire signed [31:0] arm_limit_int = arm_limit_q16 >>> 16;                       // 截断为整数 rad
```

前序报告 L73 声称实现 `−(1.0α + 0.3α̇)`。0.3 的 Q16 系数 `19661` 正确（0.3×65536=19660.8 ✅），但**随后 `>>> 16` 把结果截断为整数 rad**，再与同为整数的 `base_pump_acc`（rad/s²）相减。

**后果**：

| α（转臂角位移） | `arm_limit_int` | 实际居中加速度 |
| ---: | ---: | ---: |
| ±0.5 rad（±28.6°） | 0 | **0（完全无阻尼）** |
| ±0.99 rad | 0 | **0** |
| ±1.0 rad | ±1 | 1 rad/s²（阶跃突现） |
| ±1.5 rad | ±1 | 1 rad/s² |

而起摆过程中转臂的典型摆幅恰在 **±0.5 rad 以内** → 居中阻尼**在整个起摆阶段恒为 0**，转臂不受任何回中约束而单向漂移；一旦 |α| 越过 1 rad，阻尼力以 1 rad/s² 的**阶跃**形式突现，产生机械冲击与抖振。

**与 Python 对比**：`a_pump -= (1.0 * alpha + 0.3 * dalpha)`（`controller.py#L292`）是**连续浮点**项，α=0.3 rad 时即提供 0.3 rad/s² 的回中力。RTL 在该点输出 0。

**后果链**：F4（无回中力）→ 转臂单向漂移 → 累积至 ±2 圈触发 `soft_limit_err` → F5（PROTECT 永久锁死）→ **系统彻底停机且无法自恢复**。这是本轮整改引入的**最危险失效路径**。

**修法**：保留 Q16 精度进行运算，最后统一缩放，不要中途截断：
```verilog
// 全程 Q16：a_pump_q16 = -30·sat(...)·65536 - (alpha_q16 + 0.3·dalpha_q16)
// 最终输出前再 >>> 16 转为整数 rad/s²
```

---

### F5【严重】`STATE_PROTECT` 无任何恢复路径

**位置**：[`ctrl_fsm.v#L74-L78`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/ctrl_fsm.v) 与 [`#L119-L122`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/ctrl_fsm.v)

```verilog
end else if (soft_limit_err) begin
    current_state <= STATE_PROTECT;      // 进入
    ...
end else if (calc_en) begin
    case (current_state)
        ...
        STATE_PROTECT: begin
            final_pwm_duty       <= 16'sd0;
            reset_integral_pulse <= 1'b0;
            // ← 没有任何 current_state 赋值，永久保持 PROTECT
        end
```

**后果**：进入 PROTECT 后，**即使 `soft_limit_err` 已消失**（操作者长按 KEY2 清零转臂位置即可消除该标志，编码器 `clear_pos` 不受 FSM 门控），状态机仍锁死在 PROTECT，电机永不输出。

**唯一恢复途径**：`rst_n` 硬件复位，或将 `sw_motor_en` 拨到 0 再拨回 1（走 `!motor_en_sw` 分支回到 HANGING）。

**现场表现**：`led_motor_run` 以 ~12Hz 快闪报警，但操作者按下 KEY2 清零后 LED 仍闪、电机仍不动，**现象与"死机"无法区分**。竞赛现场答辩时这是高风险故障模式。

**修法**：在 PROTECT 分支增加恢复判据
```verilog
STATE_PROTECT: begin
    if (!soft_limit_err) current_state <= STATE_HANGING;  // 故障消除后自动复位
    final_pwm_duty <= 16'sd0;
end
```

---

## 四、中等缺陷

### F6【中】HANGING→SWINGUP 初始扰动脉冲换算错误（75 ≠ 3.0V）

**位置**：[`ctrl_fsm.v#L86`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/ctrl_fsm.v)

```verilog
final_pwm_duty <= 16'sd75; // 对应 3.0V 初始扰动脉冲
```

**验算**：`pwm_duty` 满量程 ±1000 ↔ ±12V（`furuta_lqr_ctrl.v#L36-L38` 的 `VOLT_TO_PWM = 5461163 = (1000/12)×65536`）。

- 注释声称 3.0V → 应为 `3.0 × 1000/12 = ` **250**
- 实际写入 75 → `75 × 12/1000 = ` **0.9V**

**Python 对应值**：`controller.py#L267` 的 `V_cmd = 3.0`，即 250。

**后果**：0.9V 极可能低于有刷电机实际起动死区（`config.py#L36` 的 `motor_deadband = 0.25V` 是理想模型值，实物减速电机死区常达 1~2V），导致**下垂死点对称性无法被打破**，起摆根本启动不了。

**为何未被发现**：ModelSim TEST 3 的判据为 `u_dut.fsm_state == 2'd1 && u_dut.final_pwm_duty != 0`（[`j280_hw_top_tb.v#L175`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top_tb.v)），**只判非零、不校验数值**，transcript 打印 `起摆能量泵输出 PWM: 75` 后直接 PASS。

---

### F7【中】SWINGUP→BALANCE 捕获瞬间存在 3~4 ms 控制空窗

**位置**：[`j280_hw_top.v#L319`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top.v) + [`ctrl_fsm.v#L59`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/ctrl_fsm.v)

```verilog
assign lqr_en = (current_state == STATE_BALANCE);        // 仅平衡态为 1
...
furuta_lqr_ctrl u_lqr_core (.calc_en(sample_done && lqr_en), ...);   // SWINGUP 期间从不计算
```

**时序推演**：

| 时刻 | current_state | lqr_en | LQR 核 | final_pwm_duty |
| :--- | :---: | :---: | :--- | :--- |
| 捕获判定拍 | SWINGUP | 0 | 未计算，`pwm_lqr_duty` 仍为复位值 0 | FSM 执行 `<= pwm_lqr_duty` → **写入 0** |
| +1 拍 | BALANCE | 1 | Stage 1 | 0 |
| +2 拍 | BALANCE | 1 | Stage 2 | 0 |
| +3 拍 | BALANCE | 1 | Stage 3 | 0 |
| +4 拍 | BALANCE | 1 | 有效 | **首个有效控制量** |

**后果**：捕获后约 **3~4 个控制周期（3~4 ms）电机零输出**。摆杆以最高 4.0 rad/s 进入捕获窗口时，4 ms 内即自由偏离约 0.9° 并持续加速（倒立不稳定极点时间常数约 0.2~0.3 s），捕获成功率显著下降。

**Python 无此空窗**：`controller.py#L272-L304` 中状态切换与 LQI 计算在同一次函数调用内完成（`just_switched_to_balance` 标志当拍即算出有效输出）。

**修法**：LQR 核改为**常开**（`.calc_en(sample_done)`），输出仲裁完全交给 FSM 的 `final_pwm_duty` 多路选择。这样 SWINGUP 期间 LQR 也在后台跟随计算，切入瞬间即有有效值。此改动**不增加 DSP 占用**（乘法器已存在），是零成本修复。

---

### F8【中】`traj_gen` 不受 FSM 门控，参考值可能突跳

**位置**：[`j280_hw_top.v#L237`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top.v)（`.calc_en(sample_done)`）+ [`traj_gen.v`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/traj_gen.v)

**问题 1 — 正弦相位未同步**：`traj_gen` 独立于 FSM 持续运行，`time_cnt_5s` 自由累加。当 `traj_mode = 3`（正弦）时，从 SWINGUP 切入 BALANCE 的瞬间，`alpha_ref_q16` 可能是 ±25° 区间内的**任意相位值**。刚捕获的平衡态立刻承受一个最大 25° 的位置参考阶跃 → 强扰动，极可能立即失衡回退。

**问题 2 — 定点模式为纯阶跃，无斜率限制**：`traj_mode` 由 0 切至 1 时，`alpha_ref_q16` 从 0 **直接跳变**至 `51472`（+45°）（[`traj_gen.v#L233-L236`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/traj_gen.v)）。题目拓展要求 1 明确写"控制电机**平滑地**移动到指定方向"，阶跃参考不满足"平滑"。

> 注：Python `test_suite.py::test_position_tracking` 的 0°→45° 阶跃测试确实通过了，说明 LQI 增益有一定阶跃承受力；但那是浮点模型且无传感器量化延迟，实物风险更高。

**修法**：
1. FSM 切入 BALANCE 时输出一个 `traj_sync_pulse`，复位 `time_cnt_5s`（正弦从 0 相位起振）；
2. 定点模式增加斜坡发生器（如每 1ms 步进 ≤0.5°，45° 用 90 ms 走完），既满足"平滑"要求又降低扰动。

---

### F9【中】D6 声称已修复，实际与整改前完全一致

**前序报告 L209 声明**：

> "系统控制计算统一由 `sample_done` 触发，主状态机、起摆核、轨迹发生器及 LQI 控制核在同一确定性时钟拍沿同步更新。"

**代码实况**：

| 模块 | 例化处 | `calc_en` 实际接入 | 触发时刻 |
| :--- | :--- | :--- | :--- |
| `angle_sensor_reader` | `j280_hw_top.v#L179` | `calc_en_pulse` | 1ms 网格（启动 SPI） |
| **`encoder_quad_reader`** | **`j280_hw_top.v#L212`** | **`calc_en_pulse`** | **1ms 网格整点** |
| `traj_gen` | `#L237` | `sample_done` | 1ms + SPI ≈ 7.2us |
| `swing_up_ctrl` | `#L303` | `sample_done` | 1ms + 7.2us |
| `furuta_lqr_ctrl` | `#L319` | `sample_done && lqr_en` | 1ms + 7.2us |
| `ctrl_fsm` | `#L335` | `sample_done` | 1ms + 7.2us |

编码器（产出 α、α̇）仍锚定在 1ms 网格，而 LQI 核在 7.2us 后取用 → **θ/θ̇ 与 α/α̇ 的采样时刻错位约 7.2us，与整改前完全相同**。D6 未修复，但报告标记为【已修复】。

**影响评估**：7.2us 占 1ms 控制周期的 0.72%，在 50MHz/1000Hz 架构下影响有限，**不构成阻断**；但报告状态标注不实，且拓展要求 2 的高速轨迹跟踪会引入可测相位偏差。

**修法**：将 `u_encoder` 的 `.calc_en` 改接 `sample_done`（一行改动）。

---

### F10【中】DSP 资源 100% 占满，零余量

**PnR 实测**：

```
DSP                   | 20/20    | 100%
  --MULT18X18         | 2
  --MULT36X36         | 6
  --MULTALU36X18      | 7
```

GW2A-55 全系仅提供 **20 个 DSP slice，已全部占用**。前序报告 L324 称"DSP 使用规范"，实际是**容量红线**。

**后果**：后续任何需要乘法器的改动都将无法综合，包括：

- F2 的能量计算（需 θ̇² 与能量合成，至少 1~2 个乘法器）
- F3 的 tanh 若采用有理近似
- CORDIC 坐标旋转（更高精度角度解算）
- 状态观测器 / 卡尔曼滤波
- GAO 在线逻辑分析仪的触发比较单元

**建议**：立即规划 DSP 置换。可优化点：
1. `furuta_lqr_ctrl` 的 5 路 32×32 乘法占 6 个 MULT36X36 + 部分 MULTALU——K1~K5 增益绝对值均 < 2¹⁴，状态量 < 2²⁰，**可降位宽至 18×18**，把 MULT36X36 换成 MULT18X18（GW2A 的 1 个 MULT36X36 可由 2 个 MULT18X18 拼成，反向拆分可省资源）；
2. `encoder_quad_reader` 的 64 位乘法（`pulse_count × 6746406`）可改为 32 位——软限位 ±2 圈下 `pulse_count` 实际范围仅 ±16000，18 位足够。

---

## 五、轻微缺陷与代码质量

### F11【低】时序裕量偏紧，报告表述不准确

| 指标 | 实测值 | 评价 |
| :--- | :--- | :--- |
| 约束频率 | 50.000 MHz | — |
| 实际 Fmax | 56.305 MHz | 裕量 **12.6%** |
| 最小 Setup Slack | **2.240 ns** / 20 ns | 裕量 **11.2%** |
| 逻辑级数 | **17 级** | 偏深 |
| 关键路径 | `u_lqr_core/n62_s0/DOUT[1]` → `u_lqr_core/pwm_mult_0_s/B[17]` | 数据延迟 **17.536 ns** |
| 分析 corner | Slow 0.95V 85C C8/I7 | 最恶劣工艺角，结果可信 ✅ |

前序报告 L31/L74 称"时序裕量充足"、L324 称"Slack 正裕量"。**正裕量属实，但仅 11.2%，谈不上"充足"**。关键路径落在 LQI 核第 3 级 `pwm_mult`（`v_cmd_q16 × VOLT_TO_PWM`）的乘法器输入端，说明第 2 级饱和比较逻辑（F/D8 新增的 ±200V 三段判断）串进了乘法器路径。

**建议**：在第 2 级与第 3 级之间再插一拍寄存器（把 `v_cmd_q16` 打拍后再送乘法器），可将 17 级逻辑切成两段，Fmax 有望提升至 70MHz 以上，为后续改动留出余量。代价是控制延迟从 60ns 增至 80ns（对 1ms 控制周期无实质影响）。

### F12【低】SDC 未约束 4 个真正的异步输入

[`furuta_lqr_ctrl.sdc`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/furuta_lqr_ctrl.sdc) 现约束了 `rst_n`、4 个开关按键（false_path）与 3 个 LED（false_path），但 **`enc_a`、`enc_b`、`enc_z`、`adc_miso`** 这 4 个真正跨时钟域的异步输入**未被约束**。

功能上是安全的（编码器已有双级同步器 + 8 拍消抖，MISO 由内部 2.5MHz SCLK 采样），但这些路径未纳入时序报告，属于**验证盲区**。

建议补充：
```tcl
set_false_path -from [get_ports {enc_a enc_b enc_z adc_miso}]
```

> **勘误**：上一轮口头检查中我曾怀疑 SDC 使用 `//` 注释会导致 Tcl 解析失败。**经实测该疑虑不成立**——PnR 报告正确列出了 `<Timing Constraints File>`，`clk_50m` 的 20.000ns 约束确实生效，说明 Gowin EDA V1.9.12.03 容忍 `//` 注释。此项不构成问题，特此更正。

### F13【低】Testbench 验证有效性不足，构成主要验证盲区

| 问题 | 证据 | 影响 |
| :--- | :--- | :--- |
| TEST 6 靠**覆盖参数**才触发保护 | [`j280_hw_top_tb.v#L51`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/j280_hw_top_tb.v) `.ARM_SOFT_LIMIT_Q16(32'sd3000)`（≈29 脉冲），而真实默认值为 `823548`（±2 圈 = 7999 脉冲） | **真实软限位阈值从未被验证**；transcript 中"编码器脉冲数: 56, soft_limit_err: 1"仅在仿真专用参数下成立 |
| TEST 3 只判非零 | `#L175` `final_pwm_duty != 0` | 放过了 F6 的 75/250 换算错误 |
| 全部 6 项测试均为**开环激励** | TB 中无倒立摆动力学模型，ADC 值由 `simulated_adc_data` 人工设定，编码器脉冲由 `repeat` 循环人工注入 | **"摆杆能否真正立起来并持续平衡"这一核心指标依然没有任何 RTL 级证据** |

**因此**：前序报告将"闭环状态机回退测试通过"作为基础要求 3（L43）与拓展要求 3（L46）的达标证据，属**过度声明**。TEST 4 验证的是"ADC 码值突变为 2600 时状态机回退到 SWINGUP"，这是**状态转移逻辑测试**，不是抗扰能力测试——真正的抗扰测试需要动力学模型在闭环下施加外力矩，观察摆杆能否自行恢复。

Python 侧 `test_suite.py` 的 6 项测试（含起摆、抗扰、定点、轨迹）确实全部通过，但验证对象是**浮点/定点数值模型**，且其起摆逻辑含 F2 缺失的能量判据、F3 的 tanh、F4 的连续居中阻尼——**模型通过不能推断 RTL 通过**。

### F14【低】`angle_sensor_reader.v` 同一 always 块内混用阻塞与非阻塞赋值

**位置**：[`angle_sensor_reader.v#L198-L224`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/angle_sensor_reader.v)

```verilog
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        raw_dtheta_q16   <= 32'sd0;        // 非阻塞
        ...
    end else if (process_trigger) begin
        current_theta_q16 = diff_rad_q16;                            // 阻塞
        theta_err_q16  <= current_theta_q16;                         // 非阻塞
        raw_dtheta_q16 = (current_theta_q16 - theta_prev_q16) * 32'sd1000;  // 阻塞
        theta_prev_q16 <= current_theta_q16;                         // 非阻塞
        dtheta_q16     <= dtheta_q16 + filter_diff_q16;              // 非阻塞
```

`raw_dtheta_q16` 在复位分支用 `<=`、在正常分支用 `=`，**同一变量在同一时序块内混用两种赋值语义**，属 Verilog 编码规范明确禁止的写法，会造成仿真与综合语义不一致的风险（本例中 `raw_dtheta_q16` 实际未被后续读取，功能上暂无影响，但 `filter_diff_mult` 依赖的是它作为 wire 的等价网络）。`current_theta_q16` 声明为 `reg` 却从不复位，实际被综合为组合逻辑网络。

**建议**：将 `current_theta_q16`、`raw_dtheta_q16` 改为 `wire` 并用 `assign` 表达组合逻辑，时序块内只保留非阻塞赋值。

### F15【低】起摆前置条件缺少操作提示与规程

`ctrl_fsm.v#L83` 要求 `calib_done` 为真才允许 HANGING→SWINGUP，而 `zero_calibrated` 上电默认为 0。即**上电后必须先扶直摆杆并按 KEY1 标定，否则系统完全不动作**。

该设计本身是合理的安全机制，但：
1. `led_calib_ok` 未标定时仅 ~1.5Hz 慢闪，无明确"请先标定"语义；
2. 前序报告与 `doc/` 下 4 份手册均未把此条写成**上电操作规程的第一步**。

建议在 `doc/J280套件硬件实物检测与校准实操指南.md` 中补充上电流程：`上电 → 拨 sw_motor_en=0 → 扶直摆杆 → 按 KEY1 至 led_calib_ok 常亮 → 拨 sw_motor_en=1 → 系统自动起摆`。

---

## 六、修正后的达标判定

| 赛题条目 | 原报告(首轮) | 前序报告(整改后声称) | **本轮复检实测** | 关键依据 |
| :--- | :---: | :---: | :---: | :--- |
| 设计要求 · 信号采集 | 70% | 100% | **90%** | SPI/编码器/滤波完备；帧格式仍未按具体 ADC 型号适配（见 §7 待确认） |
| 设计要求 · 主控计算平台 | 90% | 100% | **95%** | LQI 核 bit-exact，D8 已修；F11 时序裕量偏紧 |
| 设计要求 · PWM 执行驱动 | 80% | 100% | **100%** | D1 修复正确，三态验证通过 |
| 设计要求 · 摆臂定点调节 | 0% | 100% | **85%** | KEY2 四模式切换有效；F8 阶跃无斜率限制 |
| **基础要求 1 · 起摆控制** | **0%** | **100%** | ❌ **30%** | **F1 cos 表索引错误 + F2 能量判据缺失 → 能量泵物理上无法起摆**；FSM 架构与结构冲突解除是真实进展 |
| 基础要求 2 · 自平衡控制 | 60% | 100% | **85%** | LQI + 双重捕获判据正确；F7 捕获空窗 3~4ms |
| 基础要求 3 · 稳定性与抗扰 | 未验证 | 100% | ⚠️ **未验证** | F13：无动力学闭环 TB，抗扰无 RTL 级证据 |
| **拓展要求 1 · 定点位置控制** | **0%** | **100%** | **85%** | KEY2 短按切 0°/±45° 已验证；F8 平滑性不达标 |
| **拓展要求 2 · 轨迹与速度跟踪** | **0%** | **100%** | **90%** | 128 点 LUT + 速度前馈算法经独立验算**正确**；未做闭环验证；F8 相位突跳 |
| **拓展要求 3 · 动态姿态保持** | **0%** | **100%** | ⚠️ **未验证** | 依赖 1、2，无闭环证据 |
| 工程可下板性 | 0% | 100% | ✅ **100%** | E1~E4 全部真实解决，时序收敛，bitstream 已更新 |

### 与前序报告声称不符的 5 处

| # | 前序报告声称 | 复检实况 |
| :---: | :--- | :--- |
| 1 | L12「完全满足 · 100% 合规达标」 | 基础要求 1 未达成 |
| 2 | L70「cosθ 查找表计算瞬时机械势能 E_pot 与动能 E_kin」 | **代码中不存在任何能量计算**，`energy_deficit` 硬编码为 1（F2） |
| 3 | L17/L71「平滑自适应连续化能量泵（tanh 连续过渡）」 | 实为 `clamp(±1.0)` 硬线性饱和，x=1.0 处偏差 +31%（F3） |
| 4 | L209「D6 已修复：统一由 sample_done 触发」 | 编码器仍接 `calc_en_pulse`，错位未变（F9） |
| 5 | L324「DSP 使用规范」「时序裕量充足」 | DSP **20/20 = 100% 零余量**；Setup Slack 仅 11.2%（F10/F11） |

---

## 七、整改优先级与实施顺序

### 第一优先：恢复起摆功能（F1 → F2 → F4 → F10）

必须按此顺序，因为 F2 需要新增乘法器，而 DSP 已满（F10），需先腾出资源。

1. **F10 先行**：把 `furuta_lqr_ctrl` 的 5 路乘法降位宽至 18×18（增益 <2¹⁴、状态量 <2²⁰，精度足够），并把 `encoder_quad_reader` 的 64 位乘法降为 32 位（软限位下 `pulse_count` 仅 ±16000）。目标：DSP 占用降至 ≤16/20，腾出 4 个。
2. **F1**：重新生成 cos 表（`cos(k·π/63), k=0..63`，`idx=63` 必须为 `−16384`），索引改为 `abs_theta_q16 × 63 / 205887`（编译期常数除法可优化为倒数乘法）。**验证方法**：在 TB 中扫描 θ = 0°~180°（步进 5°），逐点断言 `|cos_q14/16384 − cos(θ)| < 0.02`。
3. **F2**：实现 `E_q16 = 21841·θ̇² >>>16 + 3214·(cosθ_q14 − 16384) >>>16`（`Jp/2 = 0.0003335×65536 = 21.84`→需二次定标；`m2·g·l2 = 0.04905×65536 = 3214`），`energy_deficit = (E_q16 < 0)`，并在 `base_pump_acc` 处门控：`E ≥ 0 时 a_pump = 0`。
4. **F4**：全程保持 Q16 运算，仅在最终输出级 `>>>16` 转整数 rad/s²，消除中途截断。

### 第二优先：消除失效路径（F5 → F6 → F7）

5. **F5**：PROTECT 分支增加 `if (!soft_limit_err) current_state <= STATE_HANGING;`。
6. **F6**：`16'sd75` 改为 `16'sd250`（3.0V），或按实测电机死区电压标定（建议做成 parameter 便于现场调参）。
7. **F7**：LQR 核 `.calc_en` 改为 `sample_done`（常开），去掉 `&& lqr_en`。**零 DSP 成本，一行改动，收益最大**。

### 第三优先：平滑性与验证（F8 → F13 → F3）

8. **F8**：FSM 输出 `traj_sync_pulse` 复位 `time_cnt_5s`；定点模式增加斜坡发生器（≤0.5°/ms）。
9. **F13**：**建立闭环 HIL 仿真**——把 [`simulation/dynamics.py`](file:///c:/Users/28399/Desktop/赛道/simulation/dynamics.py) 的欧拉-拉格朗日方程移植为 Verilog 行为级模型（`real` 类型 + RK4，加 `` `ifndef SYNTHESIS `` 保护，不参与综合），使 TB 能真正验证：起摆成功率、起摆耗时、平衡保持时长、抗推力恢复时间。同时：
   - TEST 6 增加一组**不覆盖参数**的用例，注入 8000+ 脉冲验证真实阈值 823548；
   - TEST 3 判据改为数值区间断言 `final_pwm_duty == 250`。
10. **F3**：tanh 改 4 段折线近似（零 DSP 开销）。

### 第四优先：收尾（F9 / F11 / F12 / F14 / F15）

11. **F9**：`u_encoder.calc_en` 改接 `sample_done`（一行）。
12. **F11**：LQI 核第 2/3 级之间插一拍，Fmax 目标 ≥70MHz。
13. **F12**：SDC 补 `set_false_path -from [get_ports {enc_a enc_b enc_z adc_miso}]`。
14. **F14**：`current_theta_q16`、`raw_dtheta_q16` 改为 `wire` + `assign`。
15. **F15**：在硬件实操指南中补充上电操作规程。

### 工程卫生

16. `.gitignore` 需补 `sim_modelsim/`（当前该目录未跟踪且未被忽略，与已忽略的 `sim_run/work/` 属同类产物）；
17. D9：删除 `sim_run/` 与 `赛道/simulation/src_verilog/` 下的源码副本，改为 `run.do` 中直接指向 `../src/` 编译，消除三份代码漂移风险。

---

## 八、待确认的硬件信息（影响最终验收）

以下 5 项仍无法从代码与现有文档推断，**在完成整改后下板前必须确认**：

1. **板载晶振是否确为 50MHz**——全部模块的 `CLK_FREQ_HZ` 均按 50MHz 参数化，若实际为 27MHz/24MHz，则 1ms 节拍、SPI 2.5MHz 分频、PWM 20kHz 载波将全部偏移，`TIMER_1MS_LIMIT` 需同步修改。
2. **摆杆角度传感器的确切型号**——[`angle_sensor_reader.v#L130`](file:///d:/Gowin_fpga/edu/project/furuta_lqr_ctrl/src/angle_sensor_reader.v) 仍硬编码 `adc_latch <= shift_reg[11:0]`，且无 MOSI/命令字输出。若为 MCP3201（首 bit 为 null，12 位数据落在 `shift_reg[12:1]`）则**位对齐错误**；若为 AS5048A/TLE5012B 则**必须发送命令字**，当前单向 SPI 无法工作。
3. **电机驱动是否确为 TB6612FNG**——决定 D1 修复方式与 STBY 引脚连接是否正确。
4. **J280 底板原理图或引脚分配表**——CST 现有 20 个引脚虽在 PG484 封装中合法（PnR 已接受、Bank VCCIO=3.3V 与 LVCMOS33 匹配、`clk_50m` 落在 GCLKT_2 专用时钟脚），但**是否对应底板实际连线无法由工具验证**。前序报告 L28 称依据"高云 3PA1030 核心板原理图与 DS102 数据手册"编写——DS102 是**芯片级**数据手册，不含底板连线信息，需确认引脚映射的真实来源。
5. **电机供电电压是否确为 12V**——`VOLT_TO_PWM = 5461163` 基于 12V↔1000 计数换算，若实际为 24V 则输出增益翻倍。

---

## 附录 A：F1 cos 查找表完整验证脚本与输出

验证方法：用 Python 精确复现 RTL 的位级索引行为（`(q16 >> 11) & 0x3F`、`>= 205887` 钳位），再从源码正则提取 64 项表值，与 `numpy.cos` 逐点比对。

```python
import numpy as np, re
src = open(r"...\src\swing_up_ctrl.v", encoding="utf-8").read()
tbl = {int(m.group(1)): int(m.group(2)+"1")*int(m.group(3))
       for m in re.finditer(r"6'd(\d+):\s*cos_q14\s*=\s*(-?)16'sd(\d+)", src)}
for deg in range(0, 181, 5):
    q16 = int(round(np.radians(deg)*65536))
    idx = 63 if q16 >= 205887 else (q16 >> 11) & 0x3F
    print(deg, idx, tbl[idx]/16384, np.cos(np.radians(deg)))
```

输出（节选，完整数据见 §2 F1 表格）：

```
 table entries: 64
  theta(deg) | RTL idx | RTL cos  | true cos | error
       0.0 |       0 |   1.0000 |   1.0000 | +0.0000
      45.0 |      25 |   0.3127 |   0.7071 | -0.3944
      60.0 |      33 |  -0.1060 |   0.5000 | -0.6060  <== SIGN FLIP
      90.0 |      50 |  -0.8488 |   0.0000 | -0.8488
     120.0 |       3 |   0.9891 |  -0.5000 | +1.4891  <== SIGN FLIP
     150.0 |      19 |   0.5859 |  -0.8660 | +1.4520  <== SIGN FLIP
     170.0 |      30 |   0.0548 |  -0.9848 | +1.0396  <== SIGN FLIP
     180.0 |      63 |  -0.8788 |  -1.0000 | +0.1212
```

## 附录 B：工具实测输出摘录

**B1 综合（`impl/gwsynthesis/furuta_lqr_ctrl.log`）**

```
Analyzing Verilog file '...\src\furuta_lqr_ctrl.v'      (共 8 个文件全部 Analyzing)
Compiling module 'j280_hw_top'
Compiling module 'encoder_quad_reader(USE_Z_INDEX=1)'
NOTE  (EX0101) : Current top module is "j280_hw_top"
GowinSynthesis finish                                    （无 WARN / 无 ERROR）
```

**B2 布局布线（`impl/pnr/furuta_lqr_ctrl.log`）**

```
Reading constraint file: "...\src\furuta_lqr_ctrl.cst"
Physical Constraint parsed completed
Running timing analysis...... [95%] Timing analysis completed
Bitstream generation completed
Fri Sep 11 23:10:30 2026
```

**B3 资源占用（`impl/pnr/furuta_lqr_ctrl.rpt.txt`）**

```
Logic      | 2005/54720  |  4%   (1149 LUT, 856 ALU, 0 ROM16)
Register   |  564/42000  |  2%   (Latch = 0  ✅ 无意外锁存器)
CLS        | 1170/27360  |  5%
I/O Port   |   20/320    |  7%   ✅ 与顶层端口数一致
DSP        |   20/20     | 100%  ⚠️ F10
Bank 2     |   12/46     | 27%   BankVccio = 3.3 ✅
Bank 6     |    8/34     | 24%   BankVccio = 3.3 ✅
Global Clock: clk_50m_d → PRIMARY (TR TL BR BL) ✅
```

**B4 引脚约束生效确认（节选）**

```
Port Name      | Loc./Bank | Constraint | IO Type  | CFG      | BankVccio
clk_50m        | M19/2     | Y          | LVCMOS33 | GCLKT_2  | 3.3
rst_n          | U1/6      | Y          | LVCMOS33 | -        | 3.3
motor_stby     | F18/2     | Y          | LVCMOS33 | -        | 3.3
enc_a          | J19/2     | Y          | LVCMOS33 | -        | 3.3
adc_miso       | F19/2     | Y          | LVCMOS33 | -        | 3.3
（20 个端口 Constraint 列全部为 Y）
```

**B5 ModelSim（`sim_modelsim/transcript`，End time 23:10:15）**

```
[TEST 1] 零点标定          → PASS  (zero_offset_reg: 2048)
[TEST 2] 编码器 4 倍频      → PASS  (pulse_count: 16)
[TEST 3] 起摆→平衡切换      → PASS  (current_state: 1, PWM: 75 ← F6 未被发现)
                              PASS  (dtheta: 2091, current_state: 2, PWM: 178)
[TEST 4] 跌落回退 SWINGUP   → PASS  (current_state: 1)
[TEST 5] KEY2 切 +45° 模式  → PASS  (traj_mode: 1, alpha_ref: 51472)
[TEST 6] 软限位保护         → PASS  (pulse_count: 56, soft_limit_err: 1
                                      ← 仅在 ARM_SOFT_LIMIT_Q16=3000 覆盖参数下成立, F13)
Errors: 0, Warnings: 0
```

---

**复检结论重申**：本轮整改在工程通路上是扎实且可复现的，P0 四项阻断问题已真实解决，外设驱动与 LQI 计算核质量良好；但**起摆控制因 F1（cos 查表索引错误，已数值验证 θ>55° 失准、θ>114° 符号翻转）与 F2（能量判据完全未实现）两处致命缺陷，在物理上无法完成起摆动作**，叠加 F4+F5 形成的"转臂漂移→软限位→永久锁死"失效路径，系统当前**不具备基础要求 1 的验收条件**。建议按 §7 顺序整改，其中 **F7（一行改动、零 DSP 成本）与 F5（三行改动）可立即实施**，F1/F2 需配合 F10 的 DSP 置换同步推进。
