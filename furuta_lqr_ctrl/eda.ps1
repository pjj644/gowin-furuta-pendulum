<#
.SYNOPSIS
    EDA 工具链统一入口 —— 解决本仓库位于非 ASCII（中文）路径下时 Gowin 与 ModelSim 均无法工作的问题。

.WHY THIS EXISTS
    本项目曾位于含中文目录名（...\赛道\furuta_lqr_ctrl）下。实测两个 EDA 工具均会失败：

      Gowin EDA V1.9.12.03
        Open project C:/Users/28399/Desktop/??/... finished     <- 中文被读成 ??
        ERROR (SP0002) : Corrupted project file:
            ".../impl/gwsynthesis/furuta_lqr_ctrl.prj"
        退出码 1。清除 impl/ 后重试仍然失败，故不是陈旧产物问题。

      ModelSim SE-64 10.7
        sqlite3_open_v2: No such file or directory
        init_dbinfo() DATABASE ERROR: (sqlite3_open
            C:/Users/28399/Desktop/????/.../work/_lib.qdb): unable to open database file
        mtilibWrite(): INTERNAL ERROR: Unexpected null object encountered
        *** 但 vlog 退出码仍为 0 ***  <- 失败却报成功，与 run_all.bat 里记录的
                                          "$finish 使退出码失明" 属同一类陷阱

    现状：工程现已迁移至纯 ASCII 路径（C:\Users\28399\Desktop\GoWin\furuta_lqr_ctrl）。
    两个 EDA 工具均可直接原生运行，eda.ps1 会自动检测并直接使用真实路径；
    纯 ASCII 的 NTFS junction 机制（C:\fpga_build）作为自动回退兜底保留。

    已实测验证（通过 junction 路径 C:\fpga_build）：
      综合   exit 0, top = j280_hw_top, Fmax 62.352 MHz, Slack 3.962 ns, 违例 0/0,
             bitstream 正确写入真实目录
      回归   Step2 50 PASS / Step3 9 PASS / Step4 6 PASS / Step5 8 PASS 1 FAIL,
             退出码 1（如实报红），与迁移前逐项吻合

.USAGE
    .\eda.ps1 build      综合 + PnR + 时序 + bitstream，并打印验收要点与基线对比
    .\eda.ps1 regress    ModelSim 全量回归（5 步），并打印判决摘要
    .\eda.ps1 doctor     自检：junction 状态、路径 ASCII 性、两条工具链是否可用
    .\eda.ps1 path       只打印应当使用的 ASCII 工作路径

.NOTES
    junction 位置默认 C:\fpga_build，可用环境变量 FPGA_ASCII_LINK 覆盖。
    每次调用都会校验并（必要时）重建 junction，因此本脚本是幂等的。
#>

[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('build', 'regress', 'doctor', 'path')]
    [string]$Command = 'doctor'
)

$ErrorActionPreference = 'Stop'

# --- 工具链位置（如安装路径不同，改这两行或用环境变量覆盖）--------------------
$GowinBin   = if ($env:GOWIN_BIN)     { $env:GOWIN_BIN }     else { 'D:\Gowin_fpga\Gowin\Gowin_V1.9.12.03_x64\IDE\bin' }
$ModelSimBin= if ($env:MODELSIM_BIN)  { $env:MODELSIM_BIN }  else { 'D:\modelsim\win64' }
$LinkPath   = if ($env:FPGA_ASCII_LINK){ $env:FPGA_ASCII_LINK } else { 'C:\fpga_build' }

$RealPath = $PSScriptRoot          # 本仓库真实位置（可能含中文）

# -----------------------------------------------------------------------------
# 确保 ASCII junction 存在且指向本仓库
# -----------------------------------------------------------------------------
function Initialize-AsciiLink {
    if ([IO.Path]::GetFullPath($RealPath) -ieq [IO.Path]::GetFullPath($LinkPath)) {
        return $RealPath            # 仓库本身就在 ASCII 路径上，无需 junction
    }
    if ($RealPath -match '^[\x00-\x7F]+$') {
        return $RealPath            # 纯 ASCII，直接用真实路径
    }

    if (Test-Path $LinkPath) {
        $item = Get-Item $LinkPath -Force
        if ($item.LinkType -ne 'Junction') {
            throw "$LinkPath 已存在但不是 junction（LinkType=$($item.LinkType)）。" +
                  "请手动移走该目录，或用 `$env:FPGA_ASCII_LINK 指定其它 ASCII 路径。"
        }
        # 幂等：无条件重建，确保指向当前仓库（删除 junction 不会影响目标内容）
        $item.Delete()
    }
    New-Item -ItemType Junction -Path $LinkPath -Target $RealPath | Out-Null
    return $LinkPath
}

# -----------------------------------------------------------------------------
function Invoke-Build {
    param([string]$Work)
    Push-Location $Work
    try {
        $env:PATH = "$GowinBin;$env:PATH"
        Write-Host "`n=== Gowin synthesis + PnR + timing + bitstream ===" -ForegroundColor Cyan
        Write-Host "work dir : $Work"
        # 子进程的 stdout 用 Out-Host 送到控制台，**不要让它成为函数返回值**，
        # 否则 `exit (Invoke-Build ...)` 会收到一个数组而不是退出码。
        & gw_sh.exe build.tcl | Out-Host
        $code = $LASTEXITCODE
        Write-Host "gw_sh exit code = $code"

        $synLog = Join-Path $Work 'impl\gwsynthesis\furuta_lqr_ctrl.log'
        if (Test-Path $synLog) {
            $top  = Select-String -Path $synLog -Pattern 'Current top module is "([^"]+)"'
            $warn = Select-String -Path $synLog -Pattern 'WARN|ERROR'
            Write-Host ("top module : " + $(if ($top) { $top.Matches[0].Groups[1].Value } else { 'NOT FOUND' }))
            Write-Host ("warnings   : " + $(if ($warn) { "$($warn.Count)  <-- 应为 0" } else { '0' }))
        }

        $tr = Join-Path $Work 'impl\pnr\furuta_lqr_ctrl_tr_content.html'
        if (Test-Path $tr) {
            $t = ((Get-Content $tr -Encoding utf8 -Raw) -replace '<[^>]+>', ' ' -replace '\s+', ' ')
            $k = $t.IndexOf('Actual Fmax')
            if ($k -ge 0) { Write-Host ("timing     : " + $t.Substring($k, 70).Trim()) }
            $v = $t.IndexOf('Setup Violated Endpoints')
            if ($v -ge 0) { Write-Host ("violations : " + $t.Substring($v, 60).Trim()) }
        }
        $fs = Join-Path $Work 'impl\pnr\furuta_lqr_ctrl.fs'
        Write-Host ("bitstream  : " + $(if (Test-Path $fs) { (Get-Item $fs).LastWriteTime } else { 'MISSING' }))
        Write-Host "`n基线（0f594e9）: top=j280_hw_top  Fmax=62.352 MHz  Slack=3.962 ns  违例=0/0" -ForegroundColor DarkGray
        return $code
    } finally { Pop-Location }
}

# -----------------------------------------------------------------------------
function Invoke-Regress {
    param([string]$Work)
    $sim = Join-Path $Work 'sim_modelsim'
    Push-Location $sim
    try {
        Write-Host "`n=== ModelSim full regression (5 steps, ~5 min) ===" -ForegroundColor Cyan
        Write-Host "work dir : $sim"
        # 同上：run_all.bat 的全部输出经 Out-Host 上屏，不污染返回值。
        & .\run_all.bat | Out-Host
        $code = $LASTEXITCODE
        Write-Host "`nrun_all.bat exit code = $code   (0=全通过  1=有判据失败  2=vsim 进程崩溃)" -ForegroundColor $(
            if ($code -eq 0) { 'Green' } else { 'Red' })
        Write-Host "基线（0f594e9）: 73 PASS / 1 FAIL，退出码 1（TEST D 稳态误差 0.2302° > 0.20°，见 R1）" -ForegroundColor DarkGray
        return $code
    } finally { Pop-Location }
}

# -----------------------------------------------------------------------------
function Invoke-Doctor {
    param([string]$Work)
    Write-Host "`n=== eda.ps1 doctor ===" -ForegroundColor Cyan
    $isAscii = $RealPath -match '^[\x00-\x7F]+$'
    Write-Host ("real path      : $RealPath")
    Write-Host ("pure ASCII     : $isAscii" + $(if (-not $isAscii) { '   <-- EDA 工具无法直接在此路径运行' } else { '' }))
    Write-Host ("ascii work path: $Work")
    $li = Get-Item $LinkPath -Force -EA SilentlyContinue
    Write-Host ("junction       : " + $(if ($li) { "$LinkPath -> $($li.Target)  (LinkType=$($li.LinkType))" } else { 'not needed / not created' }))

    $g = Join-Path $GowinBin 'gw_sh.exe'
    $m = Join-Path $ModelSimBin 'vsim.exe'
    Write-Host ("gw_sh.exe      : " + $(if (Test-Path $g) { "OK  $g" } else { "MISSING  $g" }))
    Write-Host ("vsim.exe       : " + $(if (Test-Path $m) { "OK  $m" } else { "MISSING  $m" }))

    foreach ($f in @('furuta_lqr_ctrl.gprj', 'build.tcl', 'src\j280_hw_top.v',
                     'sim_modelsim\run_all.bat', 'sim_modelsim\compile.do')) {
        $p = Join-Path $Work $f
        Write-Host ("  {0,-32} {1}" -f $f, $(if (Test-Path $p) { 'OK' } else { 'MISSING' }))
    }

    $ini = Join-Path $Work 'sim_modelsim\modelsim.ini'
    if (Test-Path $ini) {
        $w = Select-String -Path $ini -Pattern '^work\s*=' | Select-Object -First 1
        Write-Host ("modelsim.ini   : " + $w.Line + $(if ($w.Line -match '=\s*work\s*$') { '   (正确)' } else { '   <-- 被 vmap 改写过，应 git checkout 恢复' }))
    }
    return 0
}

# -----------------------------------------------------------------------------
$Work = Initialize-AsciiLink

switch ($Command) {
    'path'    { Write-Output $Work; exit 0 }
    'build'   { exit (Invoke-Build   -Work $Work) }
    'regress' { exit (Invoke-Regress -Work $Work) }
    'doctor'  { exit (Invoke-Doctor  -Work $Work) }
}
