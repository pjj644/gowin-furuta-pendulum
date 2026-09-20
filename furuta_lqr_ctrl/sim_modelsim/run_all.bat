@echo off
setlocal enabledelayedexpansion
REM ===========================================================================
REM  Full ModelSim regression for J280 rotary inverted pendulum (Gowin GW2A-55)
REM
REM  TWO independent traps had to be solved to make this script trustworthy:
REM
REM  (1) Each testbench ends with $finish, which terminates the enclosing
REM      do-script. So the four TBs MUST be launched as four separate vsim
REM      processes; running them inside one .do file silently skips TB 2..4
REM      while still exiting 0.
REM
REM  (2) $finish makes vsim exit with code 0 EVEN WHEN THE TB PRINTS [FAIL].
REM      "Errors: 0, Warnings: 0" is ModelSim's own elaboration/runtime counter
REM      and does NOT count $display verdict strings. Therefore `if errorlevel`
REM      alone is blind to every real test failure - this caused five separate
REM      false-green incidents in this project. Every step below is therefore
REM      verified twice: no failure marker present, AND the step's own success
REM      marker present.
REM
REM  Usage:  cd sim_modelsim  &&  run_all.bat
REM          exit code 0 = all steps passed, 1 = at least one problem
REM
REM  Step 5 (furuta_hil_tb) is the F13-b closed-loop HIL platform: j280_hw_top
REM  as DUT plus furuta_plant_model (RK4 rigid-body dynamics) as the plant,
REM  closing the loop through the real SPI angle sensor and the real quadrature
REM  encoder pins.
REM
REM  Default budget is 12000 control ticks = 12 s of physical time, about
REM  4.5 minutes wall-clock. Budget breakdown (all in control ticks = ms):
REM      setup/calib      ~65
REM      TEST A swing-up  4000  (SWING_MAX; actually captured at 2816)
REM      TEST B balance   1000  (BAL_HOLD_PLAN)
REM      TEST C disturb   1100  (DIST_PLAN)
REM      TEST D setpoint  2800  (TRAJ_PLAN = 904 ramp + 1500 settle + 200 tail)
REM      ----------------------------------------------------------
REM      total           ~8965  -> 12000 leaves 34% headroom
REM  Do NOT lower this below ~9500 or TEST D is skipped (its guard requires
REM  n_ticks >= 2700). Override for the full-length acceptance run with e.g.
REM      set HIL_ARGS=+HIL_CTRL_CYCLES=65000
REM  (see src/furuta_hil_tb.v header).
REM ===========================================================================

set PATH=D:\modelsim\win64;%PATH%
if "%HIL_ARGS%"=="" set HIL_ARGS=+HIL_CTRL_CYCLES=20000 +HIL_TRACE_DIV=50
set PROBLEMS=0

del /q _s1.log _s2.log _s3.log _s4.log _s5.log 2>nul

echo.
echo ================================================================
echo   Step 1/5  Compile all RTL and testbenches
echo ================================================================
if exist work rmdir /s /q work
call vsim -c -do "do compile.do" > _s1.log 2>&1
if errorlevel 1 goto fatal
type _s1.log
findstr /C:"** Error" _s1.log >nul && (echo *** STEP 1: compile errors detected *** & set /a PROBLEMS+=1)
findstr /C:"** Fatal" _s1.log >nul && (echo *** STEP 1: fatal errors detected *** & set /a PROBLEMS+=1)

echo.
echo ================================================================
echo   Step 2/5  swing_up_ctrl_tb   (unit: energy scaling, 35 matrix)
echo ================================================================
call vsim -c -voptargs=+acc work.swing_up_ctrl_tb -do "run -all; quit -f" > _s2.log 2>&1
if errorlevel 1 goto fatal
type _s2.log
findstr /C:"[FAIL]"  _s2.log >nul && (echo *** STEP 2 VERDICT: FAIL *** & set /a PROBLEMS+=1)
findstr /C:"[ERROR]" _s2.log >nul && (echo *** STEP 2 VERDICT: ERROR *** & set /a PROBLEMS+=1)
findstr /C:"[ALL PASS]" _s2.log >nul || (echo *** STEP 2: success marker missing *** & set /a PROBLEMS+=1)

echo.
echo ================================================================
echo   Step 3/5  j280_hw_top_tb     (integration: FSM and peripherals)
echo ================================================================
call vsim -c -voptargs=+acc work.j280_hw_top_tb -do "run -all; quit -f" > _s3.log 2>&1
if errorlevel 1 goto fatal
type _s3.log
findstr /C:"[FAIL]"  _s3.log >nul && (echo *** STEP 3 VERDICT: FAIL *** & set /a PROBLEMS+=1)
findstr /C:"[ERROR]" _s3.log >nul && (echo *** STEP 3 VERDICT: ERROR *** & set /a PROBLEMS+=1)
findstr /C:"ALL PASSED" _s3.log >nul || (echo *** STEP 3: success marker missing *** & set /a PROBLEMS+=1)

echo.
echo ================================================================
echo   Step 4/5  furuta_lqr_ctrl_tb (unit: LQI core pipeline)
echo ================================================================
call vsim -c -voptargs=+acc work.furuta_lqr_ctrl_tb -do "run -all; quit -f" > _s4.log 2>&1
if errorlevel 1 goto fatal
type _s4.log
findstr /C:"[FAIL]"  _s4.log >nul && (echo *** STEP 4 VERDICT: FAIL *** & set /a PROBLEMS+=1)
findstr /C:"[ERROR]" _s4.log >nul && (echo *** STEP 4 VERDICT: ERROR *** & set /a PROBLEMS+=1)
findstr /C:"100%%" _s4.log >nul || (echo *** STEP 4: success marker missing *** & set /a PROBLEMS+=1)

echo.
echo ================================================================
echo   Step 5/5  furuta_hil_tb      (CLOSED-LOOP HIL: DUT + plant model)
echo   args: %HIL_ARGS%
echo   NOTE: approx. 4.5 minutes wall-clock for the default 12 s budget.
echo ================================================================
call vsim -c -voptargs=+acc work.furuta_hil_tb %HIL_ARGS% -do "run -all; quit -f" > _s5.log 2>&1
if errorlevel 1 goto fatal
type _s5.log
findstr /C:"[FAIL]" _s5.log >nul && (echo *** STEP 5 VERDICT: FAIL *** & set /a PROBLEMS+=1)
findstr /C:"[ERROR]" _s5.log >nul && (echo *** STEP 5 VERDICT: ERROR *** & set /a PROBLEMS+=1)
findstr /C:"HIL RESULT: FAIL" _s5.log >nul && (echo *** STEP 5: HIL self-verdict is FAIL *** & set /a PROBLEMS+=1)
findstr /C:"(10/10" _s5.log >nul || (echo *** STEP 5: HIL did not achieve 10/10 PASS *** & set /a PROBLEMS+=1)
findstr /C:"DIVERGED" _s5.log >nul && (echo *** STEP 5: plant model diverged *** & set /a PROBLEMS+=1)

echo.
echo ================================================================
echo   VERDICT SUMMARY
echo ================================================================
echo   Step 1 compile          : see _s1.log
echo   Step 2 swing_up_ctrl_tb : see _s2.log
echo   Step 3 j280_hw_top_tb   : see _s3.log
echo   Step 4 furuta_lqr_ctrl  : see _s4.log
echo   Step 5 furuta_hil_tb    : see _s5.log
echo   Problems detected       : !PROBLEMS!
echo ================================================================

if !PROBLEMS! GTR 0 (
    echo.
    echo *** REGRESSION FAILED - !PROBLEMS! problem^(s^). Do NOT trust this build. ***
    endlocal
    exit /b 1
)

echo.
echo *** REGRESSION PASSED - all 5 steps executed and self-reported success. ***
endlocal
exit /b 0

:fatal
echo.
echo *** REGRESSION ABORTED: a vsim process itself failed ^(exit code nonzero^). ***
echo *** This is distinct from a test verdict failure - see the log above.     ***
endlocal
exit /b 2
