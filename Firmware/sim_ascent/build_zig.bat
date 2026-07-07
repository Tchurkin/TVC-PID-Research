@echo off
REM Ascent SIL -- compiles the EXACT Ascent_TVC.ino against the 2-axis attitude + vertical physics harness.
setlocal
set ZDIR=%TEMP%\sysiphus_zig
set ZIG_GLOBAL_CACHE_DIR=%ZDIR%\cache
set ZIG_LOCAL_CACHE_DIR=%ZDIR%\cache
"%ZDIR%\ziglang\zig.exe" c++ -std=c++17 -O0 -w -I shims -I ..\sim\shims -include ..\sim\shims\Arduino.h ascent_sim.cpp -x c++ ..\Ascent_TVC\Ascent_TVC.ino -o ascent_sim.exe
if errorlevel 1 ( echo BUILD FAILED & exit /b 1 )
echo Built ascent_sim.exe   ^(nominal: ascent_sim.exe   Monte Carlo: python ascent_mc.py 300^)
endlocal
