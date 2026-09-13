# AGENTS.md

> **给接手本项目的 AI agent 的入口文档。**
> 目标：让一个从未接触本项目的 agent 在十分钟内知道代码在哪、怎么验证、现在什么状态、什么绝对不能做。
> 人类读者请从 [`doc/README.md`](doc/README.md) 开始。

---

## 0. 三十秒速览

| | |
| :--- | :--- |
| **项目** | 全国大学生嵌入式芯片与系统设计竞赛 2026 · FPGA 创新设计赛道 · **选题一：基于 FPGA 的实时姿态控制系统** |
| **被控对象** | 旋转倒立摆（Furuta pendulum）—— J280 姿态控制系统竞赛套件 |
| **目标器件** | 高云 **GW2A-LV55PG484C8/I7**（GW2A-55C，PBGA484，speed grade 8） |
| **EDA 工具** | Gowin EDA **V1.9.12.03**（综合/PnR/bitstream）+ ModelSim **SE-64 10.7**（仿真） |
| **当前状态** | ✅ 赛题基础要求 1/2/3 **已闭环实测达成**；拓展要求 1/2/3 **全部闭环实测达成**（10/10 项判据全部 PASS） |
| **权威源码** | `furuta_lqr_ctrl\`（本目录下**嵌套的独立 git 仓库**，`feat/r1-traj-precision @ 14645fa`） |
| **文档仓库** | `C:\Users\28399\Desktop\GoWin`（`.gitignore` 已排除 RTL 子仓库） |
| **路径状态** | 仓库已迁移至纯 ASCII 路径 `C:\Users\28399\Desktop\GoWin`，**Gowin 与 ModelSim 均已可直接原生运行**；`furuta_lqr_ctrl\eda.ps1` 自动直连（见 §1.2） |
| **待办总览** | R1/R4 已闭环达成；剩余板级硬件澄清 R6~R10，见 [`doc/agent/选题一_RTL代码合规性检查报告.md`](doc/agent/选题一_RTL代码合规性检查报告.md) §五 |

---

## 1. 目录结构、两个 git 仓库与「单一事实来源」

一个工作区目录树，内含**两个独立的 git 仓库**（物理嵌套，但**不是** submodule）：

```
C:\Users\28399\Desktop\GoWin\            ← 文档仓库（git main）
├── AGENTS.md                          本文件
├── .gitignore                         已排除 furuta_lqr_ctrl/
├── doc\{human,agent}\                 文档（分类标准见 doc/README.md）
├── simulation\                        Python 黄金模型 + ⛔ 已失效的 RTL 副本
├── 赛题要求和芯片数据手册\
├── images\
└── furuta_lqr_ctrl\                   ← RTL 主仓库（独立 git main @ 1aa5f2c）
    ├── src\          15 个文件：8 可综合模块 + CST + SDC + 3 TB + 2 HIL
    ├── sim_modelsim\ run_all.bat / compile.do / modelsim.ini
    ├── eda.ps1       ★ EDA 统一入口（自动维护 ASCII junction）
    ├── build.tcl     Gowin 构建脚本（自定位，不含绝对路径）
    ├── furuta_lqr_ctrl.gprj
    └── impl\         综合产物（已忽略）
```

### 1.1 ⛔ 不要 `git add furuta_lqr_ctrl`

它是**嵌套的独立仓库**，不是 submodule。文档仓库的 `.gitignore` 已排除它。
若强行 add，git 会记成一个 gitlink（伪 submodule），克隆者只得到一个空目录，
且 RTL 的 17 个提交历史（五轮迭代的完整证据链）无法随之传递。
**两个仓库各自 `git status` / `add` / `commit` / `log`。**

### 1.2 EDA 路径状态与统一入口 `eda.ps1`

本项目曾因目录名含中文（`...\赛道\...`）导致 Gowin（读成 `??` 报 SP0002 错误）与 ModelSim（SQLite 打不开报错）无法直接运行。
**现状**：工程根目录已重命名迁移至纯 ASCII 路径 **`C:\Users\28399\Desktop\GoWin`**。实测两个 EDA 工具均已支持直接原生运行。

为了统一工具链调用并提供跨环境兼容性，推荐统一经由 PowerShell 脚本 `eda.ps1` 执行：
- 若当前工作目录为纯 ASCII 路径（当前即为 `C:\Users\28399\Desktop\GoWin\furuta_lqr_ctrl`），`eda.ps1` **自动检测并直接使用真实路径**，无需任何额外配置；
- 若未来工程被移动到含中文或特殊字符的路径，`eda.ps1` 内置的 NTFS junction（`C:\fpga_build`）**自动回退兜底机制**将自动接管。

```powershell
cd C:\Users\28399\Desktop\GoWin\furuta_lqr_ctrl
.\eda.ps1 build      # 综合 + PnR + 时序 + bitstream
.\eda.ps1 regress    # ModelSim 全量回归（5 步）
.\eda.ps1 doctor     # 自检：路径 ASCII 性 / 工具链 / 关键文件 / modelsim.ini
.\eda.ps1 path       # 打印当前使用的工作路径
```

详见 [`furuta_lqr_ctrl/eda.ps1`](furuta_lqr_ctrl/eda.ps1) 头部说明与踩坑记录 A9。

### 1.3 ⛔ `simulation/src_verilog/` 是历史副本，已漂移，禁止修改

该目录下有 12 个 `.v` 文件，是项目早期从 RTL 仓库拷来的快照，**不参与综合、不参与验证、不与 RTL 主仓库同步**。它们与权威源码已存在实质差异（例如缺少 HIL 平台、缺少后续五轮的全部缺陷修复）。

在这里改代码 = 改动完全丢失 + 制造第三份漂移副本。**所有 RTL 修改必须落在 `furuta_lqr_ctrl\src\`。**

`simulation/` 里**仍然权威**的部分是 Python 黄金模型：

| 文件 | 作用 |
| :--- | :--- |
| `config.py` | 全部物理常数与 LQR 权重的唯一定义处 |
| `dynamics.py` | 欧拉-拉格朗日动力学（HIL plant model 的对标来源） |
| `controller.py` | LQI + 能量泵起摆 + FSM 的浮点参考实现 |
| `test_suite.py` | 6 项算法级验证，HIL 判据阈值的来源 |
| `fpga_fixed_point_guide.py` | 定点化推导与代码生成 |

---

## 2. 必读文档（按此顺序）

| 顺序 | 文档 | 为什么先读它 |
| :---: | :--- | :--- |
| **1** | [`doc/agent/踩坑记录.md`](doc/agent/踩坑记录.md) | 已经付出真实代价的坑。**不读就会重犯**，其中至少三个坑各消耗过一整轮迭代 |
| **2** | [`doc/agent/选题一_RTL代码合规性检查报告.md`](doc/agent/选题一_RTL代码合规性检查报告.md) | 第五轮报告 = **当前权威状态**。§四 达标判定、§五 待办 R1~R15、§7.2 复现命令 |
| **3** | [`赛题要求和芯片数据手册/选题一_基于FPGA的实时姿态控制系统.md`](赛题要求和芯片数据手册/选题一_基于FPGA的实时姿态控制系统.md) | 需求原文。设计要求 4 项 / 基础要求 3 项 / 拓展要求 3 项 |
| 4 | `doc/human/*` | 原理与实操，**按需查阅**，不必通读 |

> ⚠️ `doc/agent/选题一_RTL代码复检报告.md` 是**第三轮历史存档**，其结论已被第五轮报告取代。**不要据此判断现状。**

---

## 3. 系统架构

### 3.1 控制链路

```
                    ┌──────────── 50 MHz 系统时钟 ────────────┐
                    │                                        │
 摆杆角度 ─SPI 2.5MHz─► angle_sensor_reader ─┐               │
   (12-bit ADC)         · 360° 解卷绕        │               │
                        · 最短角位移解卷绕    │               │
                        · 一阶 IIR (β=0.35)  ├─► j280_hw_top ─┼─► motor_pwm_driver ─► TB6612FNG ─► 直流电机
                        · 差分测速           │   · ctrl_fsm   │     · 20 kHz            · STBY/IN1/IN2
                                             │   · traj_gen   │     · 影子寄存器         · 刹车 / 滑行
 转臂角度 ─正交 A/B──► encoder_quad_reader ──┘   · swing_up   │
   (4000 CPR)           · 4 倍频鉴相            · furuta_lqr │
                        · 8 拍数字消抖           · LQI 积分器 │
                        · M 法测速                            │
                                                             │
 KEY1(标定) KEY2(模式) sw_motor_en ──────────────────────────┘
```

### 3.2 模块清单

| 模块 | 文件 | 职责 | 关键点 |
| :--- | :--- | :--- | :--- |
| `j280_hw_top` | `src/j280_hw_top.v` | **综合顶层**，20 个 I/O | LQI 积分器、软限位、按键消抖、模式仲裁 |
| `angle_sensor_reader` | `src/angle_sensor_reader.v` | SPI 主机 + 角度解算 | 3 拍流水；`process_trigger` 优先级高于 `filter_step`（见 R13） |
| `encoder_quad_reader` | `src/encoder_quad_reader.v` | 正交解码 + 测速 | 4 倍频；`default: count_step = 0` **静默丢计数**（H1 的受害方） |
| `motor_pwm_driver` | `src/motor_pwm_driver.v` | 20 kHz PWM + H 桥 | 影子寄存器；`stby_out = motor_en \| brake_mode` |
| `ctrl_fsm` | `src/ctrl_fsm.v` | 四态状态机 | HANGING(0) / SWINGUP(1) / BALANCE(2) / PROTECT(3) |
| `swing_up_ctrl` | `src/swing_up_ctrl.v` | Lyapunov 能量泵起摆 | 64 点 cos 表、tanh 4 段折线、能量判据 |
| `traj_gen` | `src/traj_gen.v` | 定点/正弦轨迹 + 斜坡 | 128 点正弦 LUT、`sync_phase` 零相位同步 |
| `furuta_lqr_ctrl` | `src/furuta_lqr_ctrl.v` | LQI 全状态反馈核 | **4 级流水，80 ns 确定性延迟** |

### 3.3 双速率与定点格式

- **系统时钟 50 MHz**（20 ns），**控制周期 1 ms**（`TIMER_1MS_LIMIT = 500`）
- HIL plant 内部用 **0.2 ms RK4** 微步（每控制周期 5 步）
- 主定点格式 **Q12.16**（`*_q16`）；混用 Q10 / Q12 / Q14 / Q24 / Q26
- **禁止使用除法器**：一律用「乘法倒数 + 移位」，例如 `/1000` → `* 0x6666 >>> 16`

### 3.4 关键常量速查（改动前务必核对）

| 常量 | 值 | 物理含义 | 位置 |
| :--- | ---: | :--- | :--- |
| 捕获窗口 | \|θ\|<22° **且** \|θ̇\|<4 rad/s | SWINGUP→BALANCE 双重条件 | `ctrl_fsm.v` |
| 动能钳位阈值 / 值 | 2097151 / 131071 | **±32 rad/s**（物理需 17.151） | `swing_up_ctrl.v` |
| E=0 物理临界 θ̇ | **17.151 rad/s** | `sqrt(4·m2·g·l2/Jp)` | 派生量 |
| cos 表索引 | `(q16 × 20535) >>> 26` | 20535/1024 ≈ 63/π | `swing_up_ctrl.v` |
| `e_kin` | `((d12² ) × 22) >>> 24` | d12 = Q12 角速度 | `swing_up_ctrl.v` |
| `e_pot` | `((cos_q14 − 16384) × 3214) >>> 14` | 相对倒立顶点 | `swing_up_ctrl.v` |
| tanh 折线段 3 斜率 | **36428** | 与段 2 常数**耦合**，不可单独改 | `swing_up_ctrl.v` |
| 积分限幅 | ±16384 | ±0.25 rad | `j280_hw_top.v` |
| 积分分离窗口 | ±11439 | ±10°，**窗口外冻结不清零**（M1） | `j280_hw_top.v` |
| 软限位阈值 | 823548 | ±2 圈 = ±720° | `j280_hw_top.v` |
| `RAMP_STEP_Q16` | **57** | 0.05°/ms，45° 需 904 ms | `traj_gen.v` |
| `RAMP_VEL_Q16` | **57000** | 必须满足 `= STEP × 1000` | `traj_gen.v` |
| PWM 死区 | **250** | 不是 75（F6） | `motor_pwm_driver.v` |
| `VOLT_TO_PWM_18` | 85333 | 基于 **12 V** 供电（见 R10） | `j280_hw_top.v` |

---

## 4. 构建与验证

### 4.1 仿真回归（约 5 分钟）

```powershell
cd C:\Users\28399\Desktop\GoWin\furuta_lqr_ctrl
.\eda.ps1 regress
$LASTEXITCODE          # 0=全通过  1=有判据失败  2=vsim 进程崩溃
```

`eda.ps1 regress` 会调用 `sim_modelsim\run_all.bat`（并自动检测路径兼容性）。

**退出码语义**（务必检查，不要只看输出）：

| 码 | 含义 |
| :---: | :--- |
| **0** | 5 步全部执行且各自报成功 |
| **1** | **有判据失败** —— 构建不可信 |
| **2** | vsim 进程本身崩溃（与判决失败是两回事） |

脚本对每一步做**双重校验**：既检查无失败标记（`[FAIL]` / `[ERROR]` / `** Error` / `HIL RESULT: FAIL` / `DIVERGED`），也检查该步的成功标记存在（`[ALL PASS]` / `ALL PASSED` / `100%` / `HIL RESULT:`）。分步日志写入 `_s1.log`~`_s5.log`（已被 `.gitignore` 忽略）。

长时间验收（65 s 物理时长，约 25 分钟）：

```powershell
$env:HIL_ARGS = "+HIL_CTRL_CYCLES=65000"
.\eda.ps1 regress
```

### 4.2 综合、布局布线、时序、bitstream（约 1 分钟）

```powershell
cd C:\Users\28399\Desktop\GoWin\furuta_lqr_ctrl
.\eda.ps1 build
$LASTEXITCODE          # 0 = 成功
```

脚本会自动打印验收要点（top module / warning 数 / Fmax / 违例 / bitstream 时间戳）
并与基线对比。底层调用 `gw_sh.exe build.tcl`。

产出（`impl/` 整个目录被 `.gitignore` 忽略）：

| 文件 | 看什么 |
| :--- | :--- |
| `impl/gwsynthesis/furuta_lqr_ctrl.log` | `Current top module is "j280_hw_top"`；有无 WARN/ERROR |
| `impl/pnr/furuta_lqr_ctrl.rpt.txt` | 资源占用、引脚约束（Constraint 列应全为 Y） |
| `impl/pnr/furuta_lqr_ctrl_tr_content.html` | **Fmax / Setup Slack / 关键路径** |
| `impl/pnr/furuta_lqr_ctrl.fs` | bitstream |

> ⚠️ **任何参与综合的文件被修改后，必须重跑 4.2。**
>
> `.gprj` 共 15 个条目，其中 **8 个 Verilog 为 `enable="1"`**（即 §3.2 的八个模块）加上 CST 与 SDC；**5 个为 `enable="0"`**（四个 testbench + `furuta_plant_model.v`，`furuta_hil_tb.v` 也在其中）。改 `enable="0"` 的文件不影响综合结果，无需重跑 Gowin；HIL 两个文件另外以 `` `ifndef SYNTHESIS `` 包裹作为双重保险。

### 4.3 验收基线（`main @ 1aa5f2c` 实测，重跑后应逐项吻合）

> 下列数字在仓库从 `D:\Gowin_fpga\edu\project\` 迁入本目录**前后完全一致**（经 junction
> 路径重测验证），证明迁移未改变任何电气结果。

```
综合      top = j280_hw_top，无 WARN，无 ERROR
Fmax      62.352 MHz        （约束 50.000 MHz）
Slack     3.962 ns 最小      （裕量 19.8%）
违例      Setup 0 / Hold 0
Logic     2348/54720  (5%)      Register 851/42000 (3%)   Latch 0
DSP       17.5/20     (88%)     余量 2.5 个
I/O       20/320      (7%)      引脚约束 20/20
关键路径  u_encoder/pulse_mult_36 → alpha_int_q16，data delay 16.003 ns

回归      Step 1 编译       13 个 vlog 全 Errors: 0, Warnings: 0
          Step 2 swing_up   52 用例 / 0 失败 / [ALL PASS]
          Step 3 j280_top   9 PASS / 0 FAIL / ALL PASSED
          Step 4 lqr_ctrl   6 PASS / 0 FAIL / 100%
          Step 5 HIL        10 PASS / 0 FAIL / HIL RESULT: PASS (10/10)
          ----------------------------------------------------
          合计 77 PASS / 0 FAIL；run_all.bat 退出码 = 0
          赛题基础要求 1/2/3、拓展要求 1/2/3 全部闭环实测达成！
```

**HIL 十项判据**：

| # | 判据 | 状态 | 实测 |
| :---: | :--- | :---: | :--- |
| 0 | 零点标定与 SOP 前置 | ✅ | `zero_offset_reg = 2048` |
| 0a | 待机态 FSM | ✅ | `fsm_state = 0 (HANGING)` |
| 0b | 传感器与 plant 同步 | ✅ | 偏差峰值 **0 脉冲 / 0 Q16** |
| A | **起摆（基础要求 1）** | ✅ | **2816 ms 自主捕获**，跌落 0 次 |
| A2 | 解卷绕有效性 | ✅ | θ 跨越 ±180° **6 次**，假尖峰 **0 次** |
| B | **平衡保持（基础要求 2）** | ✅ | 1000 ms 无跌落，\|θ\| 峰值 21.744° |
| C | **抗扰恢复（基础要求 3）** | ✅ | 0.04 N·m/50 ms → 8.228°，55 ms 恢复 |
| D | **定点伺服（拓展 1/3）** | ✅ | 稳态误差 **0.1927°** < 0.20°（R1 闭环达成） |
| E | **正弦跟踪（拓展 2）** | ✅ | 0.2Hz 正弦 5s 全周期跟踪，RMS **5.45°**，\|θ\| 峰值 **1.82°** < 8.0° |
| — | 数值完整性 | ✅ | `diverged = 0`，RK4 步数与物理时长一致 |

### 4.4 Python 黄金模型

```powershell
cd C:\Users\28399\Desktop\GoWin\simulation
python test_suite.py        # 6 项算法级验证
python run_simulation.py    # 全流程仿真 + 出图
```

---

## 5. 硬性约定（违反会造成真实损失）

> 每一条都对应本项目已经发生过一次的事故。

| # | 约定 | 违反的后果（已发生过） |
| :---: | :--- | :--- |
| **1** | **不得为了让测试通过而放宽判据/容差** | 五次「TB 全绿但功能失效」的假绿事故 |
| **2** | **不得只信退出码或 `Errors: 0, Warnings: 0`**。`$finish` 使 vsim 退出码恒为 0；那是 ModelSim 自己的计数器，**不统计 `$display` 判决字符串**。必须 grep `[FAIL]` | 上一轮 `c5be373` 打破了 `j280_hw_top_tb` TEST 5，无人察觉 |
| **3** | **多个 TB 必须各自独立 vsim 进程**。单个 `.do` 里串跑会在第一个 `$finish` 处静默终止，后续 TB 全部跳过而退出码仍为 0 | TB 2/3/4 曾被整体跳过 |
| **4** | **`quit -f` 会终止整个 do 脚本**，不能放在 `run -all` 之后再串下一个 TB | 同上 |
| **5** | **不得修改 `simulation/src_verilog/`** | 改动丢失 + 制造第三份漂移副本 |
| **6** | **声称「已修复」必须附实测证据**：数值验算、工具报告原文、或仿真日志 | 曾有报告声称 100% 达标，实际存在致命缺陷 F1/F2 |
| **7** | **改 Q 格式必须同步移位量** | `>>>8` 未随 Q12 降位宽改为 `>>>24`，动能虚高 **65536 倍**，起摆 runaway |
| **8** | **耦合常数不得当独立修正量分别改** | tanh 段 2/段 3 常数耦合，只改一个使 x=1.0 处跳变从 0.0050 **恶化**到 0.0092 |
| **9** | **每完成一个功能单独 commit**，不要把多项不相关改动堆在一个提交里 | 用户明确要求 |
| **10** | **分支策略**：仅在委派 subagent 执行时开独立功能分支；主 agent 自己直接改时**不开分支**，在当前分支修改并 commit。用户明确要求时才开分支 | 用户 2026-09-12 修正 |
| **11** | **改参与综合的文件后必须重跑 Gowin** 并核对 Fmax/Slack | 曾有改动使 Fmax 从 57.685 崩到 50.194 MHz（裕量 0.39%） |
| **12** | **不得把用户已有的未提交改动混入本次提交** | 用户明确要求 |
| **13** | **不要主动创建文档**（README、总结、说明等），除非用户明确要求 | 用户明确要求 |

---

## 6. 硬件依赖：EDA 工具无法验证的事

以下 5 项**必须在实物下板前澄清**，任何仿真都不能替代：

| # | 待确认 | 若不成立 |
| :---: | :--- | :--- |
| **R6** | 板载晶振是否确为 **50 MHz** | 1 ms 节拍、SPI 2.5 MHz 分频、PWM 20 kHz 载波全部偏移 |
| **R7** | 摆杆角度传感器**确切型号** | `angle_sensor_reader.v` 硬编码 `adc_latch <= shift_reg[11:0]` 且无 MOSI。**MCP3201** → 位对齐错；**AS5048A/TLE5012B** → 必须发命令字，当前单向 SPI 无法工作；**ADS7886** → 需核对多采 4 bit |
| **R8** | 电机驱动是否确为 **TB6612FNG** | 决定 `stby_out` 逻辑与 STBY 引脚连接 |
| **R9** | **J280 底板原理图 / 引脚分配表** | CST 的 20 个引脚在 PG484 封装内合法（PnR 已接受、Vccio 匹配、clk 落在 GCLKT_2），但**是否对应底板实际连线无法由 EDA 验证**。DS102 是芯片级手册，不含底板连线 |
| **R10** | 电机供电是否确为 **12 V** | `VOLT_TO_PWM_18 = 85333` 与起摆核 `×15` 系数均基于 12 V；若为 24 V 则增益翻倍 |

**上电必须遵循 SOP**（见 [`doc/human/J280套件硬件实物检测与校准实操指南.md`](doc/human/J280套件硬件实物检测与校准实操指南.md) 第〇章）：

```
上电前 sw_motor_en = 0  →  扶直摆杆  →  长按 KEY1 标定零点
  →  转臂归零后短按 KEY2  →  拨 sw_motor_en = 1 自动起摆
```

---

## 7. 建议的下一步

按成本递增，**先做最便宜的验证**：

| 优先 | 动作 | 成本 | 目的 |
| :---: | :--- | :--- | :--- |
| **1** | **R1-B 案**：`TAIL_SETTLE_MS` 1500 → 2500 | 改 1 个 TB 常量，5 分钟 | 判断那 0.03° 是慢振荡未衰减完，还是前馈阶跃残余 |
| **2** | **R1-A 案**：斜坡末端 `ramp_vel_q16` 用 2~3 拍线性降到 0 | 改 `traj_gen.v` 约 10 行 + 重跑综合 | 消除 −3.94 V（33% 轨）的单拍命令阶跃 |
| 3 | **R2**：跑 65 s 长时间平衡 | `set HIL_ARGS=+HIL_CTRL_CYCLES=65000`，约 25 分钟；`BAL_HOLD_PLAN` 需同步提到 ≥60000 | 回答赛题「**长时间**保持稳定」 |
| 4 | **R4**：新增 TEST E 验证 `traj_mode=3` 正弦跟踪 | 中等 | 拓展要求 2 目前只有算法级验算 |
| 5 | **R3**：起摆成功率统计（`sensor_noise_urad` 接口已预留） | 中等 | 回答「起摆是否可靠」 |

**不要做的事**：不要放宽 `TRAJ_POS_LIMIT = 0.20`（见约定 1）。R1 的 0.2302° 中约 ±0.09° 是 4000 CPR 编码器的物理量化下限，但剩余部分应通过 A/B 案消除，而不是改判据。

---

## 8. 历史脉络（五轮迭代）

| 轮次 | RTL commit | 结论 |
| :---: | :--- | :--- |
| 首轮 | `ddb4f60` | 不满足：起摆/拓展全缺，P0 四项工程阻断（CST 为空、顶层设错、产物过期、6 处大位宽除法） |
| 二轮 | `596e996` | P0 真实解决；提出 F1~F15（含两个致命：cos 表索引错、能量判据完全未实现） |
| 三轮 | `a8be230` | 11 项已修；提出 N1~N5（含致命定标错 65536 倍） |
| 四轮 | `5dec1f8` | N1~N4 已修；提出 P1~P6 |
| **五轮** | **`1aa5f2c`** | **HIL 闭环建成；起摆/平衡/抗扰首次实测 PASS；49 项编号缺陷全部关闭；余 R1~R15；仓库迁入文档目录并适配工具链** |

累计识别 **49 项编号缺陷（E/D/F/N/P/B/H/M 系列），全部关闭**；另有 **15 项剩余事项 R1~R15**，其中 3 项为待处理缺陷/隐患（R1 精度、R11 PWM 积分窗口少 1 拍、R13 流水可被抢占）。

完整追溯见 [`doc/agent/选题一_RTL代码合规性检查报告.md`](doc/agent/选题一_RTL代码合规性检查报告.md) §六。

---

## 9. 一条最重要的经验

> **这个项目里，「测试全绿」曾经五次都是假的。**
>
> 原因从来不是同一个：有时是 TB 根本没跑，有时是断言只比对 1 bit，有时是测量窗口选错，有时是被测对象的传感器模型自己丢了 1.42° 的数据，有时是 PASS 文本在为一条从未执行的检查背书。
>
> 所以在动手改任何功能之前，先问一句：**「如果我把这个功能整段删掉，哪一条断言会变红？」**
>
> 如果答不上来，那条功能就没有回归保护 —— 而你会在下一轮才发现。
