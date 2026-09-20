# 基于高云 FPGA 的旋转倒立摆实时姿态控制系统 (Gowin Furuta Pendulum LQI Control System)

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![FPGA](https://img.shields.io/badge/FPGA-Gowin%20GW2A--55C-orange.svg)](http://www.gowinsemi.com.cn/)
[![Language](https://img.shields.io/badge/Language-Verilog--2001%20%7C%20Python-green.svg)]()
[![ModelSim Tests](https://img.shields.io/badge/ModelSim%20Regression-78%2F78%20PASS-brightgreen.svg)]()
[![HIL Closed-Loop](https://img.shields.io/badge/HIL%20Closed--Loop-10%2F10%20PASS-success.svg)]()
[![Timing Slack](https://img.shields.io/badge/Timing%20Fmax-56.9~63.7%20MHz%20(No%20Violations)-blue.svg)]()

> **2026 年全国大学生嵌入式芯片与系统设计竞赛 · FPGA 创新设计赛道 · 选题一：基于 FPGA 的实时姿态控制系统**  
> 适配硬件：J280 姿态控制系统竞赛套件（水平旋转臂 + 垂直摆杆） + 高云 **GW2A-LV55PG484C8/I7**（GW2A-55C）核心板。

---

## 📖 项目简介

本项目是面向旋转倒立摆（Furuta Pendulum）的工业级全闭环 FPGA 实时数字姿态控制系统。针对官方 2026 竞赛 FAQ 对齐的实物物理参数（90g 摆杆/90g 转臂/WDD35D4 电位器/4000 CPR 增量编码器），系统实现了全硬件流水化的非线性起摆、5 阶 LQI 零静差自平衡、定点伺服与正弦连续动态轨迹跟踪。

系统不仅包含完整的 **可综合 RTL Verilog 工程**，还首创了**全引脚级数字闭环硬件在环（HIL, Hardware-in-the-Loop）实时动力学仿真平台**与 **Python 浮点黄金动力学模型**，实现了从算法推导、定点仿真、硬件在环回归到 FPGA 综合布局布线的完整全生命周期验证闭环。

---

## 🏛️ 系统架构

### 1. 硬件控制链路拓扑

```text
                     ┌────────────────── 50 MHz 系统主频 ──────────────────┐
                     │                                                      │
摆杆角度 (θ) ──────►│ angle_sensor_reader                                  │
  · 12-bit SPI ADC   │   · 360° 越界最短角位移解卷绕                         │
  · WDD35D4 电位器   │   · ±35 rad/s 盲区跳变安全限幅                       │
                     │   · 一阶数字 IIR 低通滤波 (β=0.35) ──┐               │
                     │                                      │               │
转臂角度 (α) ──────►│ encoder_quad_reader                  │               │
  · 4000 CPR 正交    │   · 4 倍频鉴相 + 8 拍数字消抖        ├─► j280_hw_top ┼─► motor_pwm_driver ──► 直流减速电机
  · 增量式光电编码器  │   · 1ms 定时 M 法高精度测速          │   (顶层逻辑)   │     · 20 kHz 超音频载波   (TB6612FNG H桥)
                     │                                      │   · ctrl_fsm  │     · 无毛刺影子寄存器
KEY1 (零点标定) ────►│                                      │   · traj_gen  │     · 刹车 / 滑行双保护
KEY2 (轨迹模式) ────►│                                      │   · swing_up  │
sw_motor_en (使能) ─►│                                      │   · furuta_lqr│
                     └──────────────────────────────────────┴───────────────┴─────────────────────────────┘
```

### 2. 核心模块职责划分

| 模块名称 | 源文件 | 核心功能与架构特性 |
| :--- | :--- | :--- |
| **`j280_hw_top`** | `src/j280_hw_top.v` | **系统顶层**。负责 20 个物理 I/O 引脚仲裁、四阶平滑软启动接入、32 位全精度无死区积分器、按键消抖与软限位保护。 |
| **`angle_sensor_reader`** | `src/angle_sensor_reader.v` | 2.5MHz 硬件 SPI 主机读取 12 位 ADC；实现全周最短角位移解卷绕与一阶 IIR 滤波；内嵌 **$\pm 35\,\text{rad/s}$ 盲区安全限幅**。 |
| **`encoder_quad_reader`** | `src/encoder_quad_reader.v` | 4 倍频正交鉴相与 8 级去抖；1ms 定时测速；支持 $\pm 2$ 圈（$\pm 8000$ 脉冲）防绞线软限位。 |
| **`furuta_lqr_ctrl`** | `src/furuta_lqr_ctrl.v` | **5 阶 LQI 状态反馈计算核**。采用 Q10 最优增益，**4 级流水线，80ns 纯硬件确定性解算延迟**，0 运行时除法器。 |
| **`swing_up_ctrl`** | `src/swing_up_ctrl.v` | 基于 Lyapunov 能量泵的自适应起摆控制；内嵌 64 点余弦查找表与 4 段连续 tanh 折线逼近。 |
| **`ctrl_fsm`** | `src/ctrl_fsm.v` | 4 态安全主控制状态机：`HANGING(0)` $\to$ `SWINGUP(1)` $\to$ `BALANCE(2)` $\to$ `PROTECT(3)`。 |
| **`traj_gen`** | `src/traj_gen.v` | 目标轨迹发生器：支持 $0^\circ$ 原点、$\pm 45^\circ$ 斜坡伺服（$0.05^\circ/\text{ms}$ 平滑退饱和）与 $0.2\text{Hz}$ 连续正弦跟踪。 |
| **`motor_pwm_driver`** | `src/motor_pwm_driver.v` | 20kHz 超音频 PWM 发生器；内置影子寄存器防止占空比毛刺；支持 TB6612FNG 动态能耗制动。 |

---

## ⚡ 核心算法与设计亮点

### 1. 转臂阻尼与位移解耦的四阶分阶平滑软启动 (`catch_cnt`)
- **工程挑战**：90g 大摆杆在起摆到达顶点的瞬间，转臂角速度动量较大（实测达 $+4.9\text{ rad/s}$）。若在切入瞬间直接施加全额位置刚度（$K_3 \alpha_{err}$），会立即将电机推向 $\pm 12\text{V}$ 饱和轨，导致维持倒立直立的平衡力矩被完全削顶剥夺而失控甩落；但若简单弱化角速度阻尼，转臂将瞬间飞车。
- **架构创新**：
  1. **角速度阻尼回路（$K_4 \dot\alpha$）全程保持 100% 刚性反馈**，确保捕获瞬间拥有充足的制动力矩压制飞车；
  2. **位置偏差（$K_3 \alpha_{err}$）实施 768ms 四阶渐进软接入**：
     - $0 \sim 255\text{ ms}$：接入 $25\%$；
     - $256 \sim 511\text{ ms}$：接入 $50\%$；
     - $512 \sim 767\text{ ms}$：接入 $75\%$；
     - $\ge 768\text{ ms}$：接入 $100\%$ 全刚度反馈。
  - **实测收益**：起摆接杆超调被精确压制在 **$21.861^\circ < 22.0^\circ$**，完全消除跌落，稳态静差收敛至 **$0.0303^\circ$**。

### 2. 345° 传感器物理盲区跳变防御
- 官方 FAQ 明确指出 WDD35D4 导电塑料电位器有效电气行程约 $345^\circ$，剩余 $15^\circ$ 为绝缘缝隙。实物旋转越过盲区时，ADC 采样跳变会引发高达 $90+\text{ rad/s}$ 的虚假速度毛刺；
- RTL 在 `angle_sensor_reader.v` 部署了 $\pm 35\,\text{rad/s}$ 物理安全限幅门限，彻底消除虚假速度毛刺对 IIR 滤波和动能计算的冲击，防止能量泵起摆中途误停。

### 3. 全精度累加器消除极限环静差
- 顶层显式设置 `DEAD_BAND_VAL = 0`，杜绝死区跳跃在平衡零点附近激发高频极限环抖颤（Limit Cycle）；
- 采用 32 位全精度累加器凭借极高直流增益自适应克服电机静态摩擦，使定点伺服稳态静差达到 **$0.0303^\circ$**（已小于 4000 CPR 编码器的单脉冲分辨率 $0.09^\circ$，实质等于 0 脉冲，达传感器物理极限）。

---

## 📊 赛题指标与闭环实测对比

经 ModelSim 全量回归与 Gowin EDA 综合布局布线实测，赛题所有基础与拓展要求**100% 闭环达标**：

| 赛题要求 | 验收指标与测试项目 | 判定标准 | 实测结果 | 结论 |
| :--- | :--- | :--- | :--- | :---: |
| **基础要求 1** | 下垂自适应能量泵平滑起摆 | 自主起摆并捕获，无反复跌落 | **3892 ms 自主捕获进入平衡，跌落 0 次** | ✅ **PASS** |
| **基础要求 2** | 倒立自平衡稳定保持 | 直立倒立保持 $\ge 1000\text{ ms}$，$\|\theta\| < 22^\circ$ | **保持 1000 ms 无跌落，$\|\theta\|$ 峰值 $21.861^\circ$** | ✅ **PASS** |
| **基础要求 3** | 外力扰动自适应抗扰恢复 | 承受 $0.04\text{ N}\cdot\text{m} / 50\text{ms}$ 扰动冲击 | **最大偏角 $5.497^\circ < 15.0^\circ$，$41\text{ ms}$ 快速恢复** | ✅ **PASS** |
| **拓展要求 1** | 水平转臂定点伺服控制 | 平滑到达 $+45^\circ$，稳态静差 $< 0.20^\circ$ | **稳态静差仅 $0.0303^\circ$（设计裕量高达 85%）** | ✅ **PASS** |
| **拓展要求 2** | 动态连续正弦轨迹跟踪 | 跟踪 $0.2\text{Hz}$ 正弦 5 秒，摆杆直立平衡 | **RMS 误差 $3.8563^\circ < 10.0^\circ$，$\|\theta\|$ 峰值 $1.632^\circ < 8.0^\circ$** | ✅ **PASS** |
| **拓展要求 3** | 运动过程中姿态平稳度 | 移动全程摆杆始终保持直立 | **移动全过程摆杆偏角 $\|\theta\| \le 4.012^\circ$** | ✅ **PASS** |
| **时序性能** | 高云 FPGA 50MHz 时钟约束 | Setup / Hold 零违例，裕量充足 | **Fmax = 56.9 ~ 63.7 MHz，Setup Slack 3.96 ~ 4.30 ns** | ✅ **PASS** |

---

## 🚀 快速上手与验证测试指南

本项目提供完全一致的自动化回归测试链路，环境配置完成即可一键复现。

### 0. 依赖工具链
- **Python**: 3.8+ (依赖 `numpy`, `scipy`, `matplotlib`, `openpyxl`)
- **ModelSim**: SE-64 10.7 或兼容版本
- **Gowin EDA**: V1.9.12.03 或更高版本

---

### 测试 1：Python 算法级仿真验证（耗时 ~5 秒）

验证欧拉-拉格朗日动力学微步 RK4 积分器、LQI 矩阵增益求解、起摆与正弦跟踪黄金参考模型：

```bash
cd simulation
python test_suite.py
```

**预期输出**：
```text
======================================================================
  全国大学生嵌入式芯片与系统设计竞赛 - 选题一物理仿真自动测试 (强化版)
======================================================================
[TEST 1] 运行物理动力学无阻尼自由摆动测试...       [PASS]
[TEST 2] 运行下垂起摆与倒立自平衡测试...           [PASS] (在 t = 3.982s 成功切入平衡)
[TEST 3] 运行抗外力推力扰动测试...                 [PASS] (恢复耗时 0.020s)
[TEST 4] 运行转臂定点位置伺服控制测试...           [PASS] (静差 0.1577 deg)
[TEST 5] 运行连续正弦轨迹与速度跟踪测试...         [PASS] (RMS 误差 4.60 deg)
[TEST 6] 运行 FPGA Q12.16 逐比特定点等效性测试...  [PASS] (最大漂移 0.0777 deg)
🎉 全部 6 项核心测试全部通过！系统算法与硬件仿真完全达标！
```

---

### 测试 2：ModelSim SE 自动化全量回归测试（耗时 ~5 分钟）

运行包含 13 个 Verilog 源文件编译、3 个单元 TB、1 个顶层 TB 以及 **闭环硬件在环 HIL 仿真（含连续 20000 周期 RK4 动力学闭环）**：

```powershell
cd furuta_lqr_ctrl
.\eda.ps1 regress
```

**双重校验退出码机制**：
- `$LASTEXITCODE = 0`：所有 5 步全部成功，78 项断言全部 PASS；
- `$LASTEXITCODE = 1`：有判据失败（拒绝假绿）；
- `$LASTEXITCODE = 2`：仿真进程崩溃。

**预期输出**：
```text
================================================================
  VERDICT SUMMARY
================================================================
  Step 1 compile          : see _s1.log  (13 个 vlog 全部 Errors: 0, Warnings: 0)
  Step 2 swing_up_ctrl_tb : see _s2.log  (53 用例全部 [ALL PASS])
  Step 3 j280_hw_top_tb   : see _s3.log  (9 PASS / 0 FAIL / ALL PASSED)
  Step 4 furuta_lqr_ctrl  : see _s4.log  (6 PASS / 0 FAIL / 100% 通过)
  Step 5 furuta_hil_tb    : see _s5.log  (10 PASS / 0 FAIL / HIL RESULT: PASS 10/10)
  Problems detected       : 0
================================================================
*** REGRESSION PASSED - all 5 steps executed and self-reported success. ***
run_all.bat exit code = 0
```

---

### 测试 3：Gowin EDA 一键综合与 Bitstream 生成（耗时 ~1 分钟）

使用 Gowin EDA 命令行工具 `gw_sh` 自动化执行语法检查、综合、映射、布局布线与时序分析：

```powershell
cd furuta_lqr_ctrl
.\eda.ps1 build
```

**产物位置**：
- 综合报告：`furuta_lqr_ctrl/impl/gwsynthesis/furuta_lqr_ctrl.log`
- 布局布线与引脚：`furuta_lqr_ctrl/impl/pnr/furuta_lqr_ctrl.rpt.txt`
- 时序报告：`furuta_lqr_ctrl/impl/pnr/furuta_lqr_ctrl.tr.html`
- FPGA Bitstream：`furuta_lqr_ctrl/impl/pnr/furuta_lqr_ctrl.fs`

---

## 🔌 硬件引脚分配与实物上电 SOP

### 1. 核心引脚映射表 (GW2A-LV55PG484C8/I7)

| 信号名称 | FPGA 引脚 | 电平标准 | 连接外设 / 说明 |
| :--- | :---: | :---: | :--- |
| `clk_50m` | **M19** | LVCMOS33 | 50MHz 板载有源晶振输入 (专用时钟管脚 GCLKT_2) |
| `rst_n` | **AB3** | LVCMOS33 | 硬件异步复位按键 (低电平复位) |
| `adc_cs_n` | **E19** | LVCMOS33 | WDD35D4 电位器 SPI ADC 片选 |
| `adc_sclk` | **E20** | LVCMOS33 | SPI 时钟 (2.5MHz) |
| `adc_miso` | **F19** | LVCMOS33 | SPI 数据串行输入 |
| `enc_a` | **D22** | LVCMOS33 | 4000 CPR 增量编码器 A 相输入 |
| `enc_b` | **E22** | LVCMOS33 | 4000 CPR 增量编码器 B 相输入 |
| `motor_pwm` | **H19** | LVCMOS33 | 直流电机 PWM 脉冲输出 (20kHz 载波) |
| `motor_dir` | **H18** | LVCMOS33 | 直流电机旋转方向信号 |
| `motor_in1` | **G17** | LVCMOS33 | TB6612 H 桥驱动输入 1 |
| `motor_in2` | **G18** | LVCMOS33 | TB6612 H 桥驱动输入 2 |
| `motor_stby` | **F18** | LVCMOS33 | TB6612 驱动使能 (1: 运行/动态能耗刹车, 0: 待机) |
| `key_calib` | **V5** | LVCMOS33 | KEY1 按键 (短按锁存倒立绝对零点) |
| `key_mode` | **V4** | LVCMOS33 | KEY2 按键 (短按切轨迹，长按 >0.7s 转臂归零) |
| `sw_motor_en`| **T2** | LVCMOS33 | 电机使能开关 (1: 开启自动起摆/平衡, 0: 紧急停机) |
| `sw_brake_mode`| **R2** | LVCMOS33 | 刹车模式开关 (1: 动态能耗制动, 0: 自由滑行) |

### 2. 实物安全上电八步规程 (SOP)
1. **上电前待机**：确保使能拨码 `sw_motor_en = 0`，刹车拨码 `sw_brake_mode = 1`；
2. **通电自检**：主板上电，观察 `led_calib_ok`（N3 引脚）慢闪（~1.5Hz），提示待机等待标定；
3. **扶正摆杆**：用手轻扶摆杆至**绝对垂直向上**位置（可用直角三角板辅助）；
4. **一键锁存**：长按 `KEY1`（>20ms），观察 `led_calib_ok` 变为**常亮**，完成绝对零位锁存；
5. **转臂归零**：手动将水平臂置于前方基准，长按 `KEY2`（$\ge 0.7\text{s}$），编码器脉冲计数值清零；
6. **释放下垂**：松开双手，让摆杆在重力作用下自然下垂于稳定点；
7. **合闸起摆**：将 `sw_motor_en` 推至 `1`，系统自动进入平滑能量泵起摆并在最高点无缝吸合自平衡；
8. **模式切换**：平衡后短按 `KEY2` 可顺序体验 $+45^\circ \to -45^\circ \to 0.2\text{Hz}$ 动态正弦跟踪！

---

## 📂 仓库目录导航

```text
.
├── README.md               ← 本文件 (项目总览、架构、快速上手与测试复现指南)
├── AGENTS.md               ← AI Agent 与开发者交接指引 (包含 49+ 项编号缺陷闭环追溯)
├── .gitignore
├── furuta_lqr_ctrl/        ← FPGA RTL 主工程 (独立完整的硬件工程源码)
│   ├── src/                8 个核心可综合模块 + CST 引脚约束 + SDC 时序约束 + 3 个单测 TB + 2 个 HIL 平台
│   ├── sim_modelsim/       ModelSim 自动化编译回归入口 (run_all.bat / compile.do / modelsim.ini)
│   ├── eda.ps1             EDA 统一运行入口脚本 (支持直接原生运行及 ASCII 兜底)
│   ├── build.tcl           Gowin 自动化构建综合脚本
│   └── furuta_lqr_ctrl.gprj 高云工程主配置文件
├── simulation/             ← Python 黄金动力学模型与算法工具链
│   ├── config.py           官方 90g 物理参数字典唯一定义处
│   ├── dynamics.py         非线性欧拉-拉格朗日动力学连续方程模型
│   ├── controller.py       LQI 全状态反馈与 Lyapunov 能量泵浮点参考实现
│   ├── test_suite.py       6 项核心控制算法自动化验证套件
│   ├── run_simulation.py   全流程闭环仿真与波形出图脚本
│   └── fpga_fixed_point_guide.py 定点化推导与 Verilog 代码生成工具
├── doc/                    ← 完整技术文档矩阵
│   ├── human/              原理精析、实操指南、实物检测与调参手册
│   └── agent/              合规审查报告、踩坑记录与缺陷全生命周期审计
└── 赛题要求和芯片数据手册/   ← 竞赛赛题指南、高云官方 FAQ 问答、芯片数据手册
```

---

## 📄 开源许可证

本项目基于 [MIT License](LICENSE) 开源发布。欢迎用于学术研究、学科竞赛及嵌入式控制系统参考学习。
