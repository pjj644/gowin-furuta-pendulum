# =============================================================================
# Gowin EDA 命令行构建脚本（综合 + 布局布线 + bitstream + 时序分析）
#
# 用法（PowerShell）:
#   cd <本仓库根目录>                     # 即 .../GoWin/furuta_lqr_ctrl
#   $env:PATH="D:\Gowin_fpga\Gowin\Gowin_V1.9.12.03_x64\IDE\bin;$env:PATH"
#   gw_sh.exe build.tcl
#
# 本脚本用 [info script] 自定位工程文件，**不含任何指向本仓库的绝对路径**，
# 因此仓库整体移动位置后无需修改本文件。（唯一的外部绝对路径是 Gowin EDA
# 的安装位置，由调用方通过 PATH 提供。）
#
# 产出（均在 impl/ 下，该目录已被 .gitignore 忽略）:
#   impl/gwsynthesis/furuta_lqr_ctrl.log              综合日志（查 top module 与 WARN）
#   impl/gwsynthesis/furuta_lqr_ctrl.vg               网表
#   impl/pnr/furuta_lqr_ctrl.rpt.txt                  资源占用与引脚约束
#   impl/pnr/furuta_lqr_ctrl_tr_content.html          时序详情（Fmax / Slack / 关键路径）
#   impl/pnr/furuta_lqr_ctrl.fs                       bitstream
#
# 验收要点（基线 0f594e9 的实测值，重跑后应逐项吻合）:
#   Current top module is "j280_hw_top"   综合无 WARN / 无 ERROR
#   Actual Fmax            62.352 MHz     （约束 50.000 MHz）
#   最小 Setup Slack        3.962 ns      （裕量 19.8%）
#   Setup/Hold 违例端点      0 / 0
#   DSP                    17.5/20 (88%)  Logic 5%  Register 3%  Latch 0
#   引脚约束                20/20         Constraint 列全为 Y
# =============================================================================

set proj_dir [file dirname [info script]]
open_project "$proj_dir/furuta_lqr_ctrl.gprj"
run all
exit
