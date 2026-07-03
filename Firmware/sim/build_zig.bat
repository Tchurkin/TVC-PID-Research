@echo off
REM ============================================================================
REM Turnkey SIL build using ziglang (a pip-installable, self-contained C++ compiler).
REM Needs only Python (which you have) -- no separate MinGW/MSVC/WSL install.
REM Installs zig to a LOCAL temp dir (not the cloud drive, which drops the binary),
REM then compiles the EXACT firmware + harness. First run builds zig's libc++ (slow,
REM one-time, a few minutes); later runs are fast.
REM ============================================================================
setlocal
set ZDIR=%TEMP%\sysiphus_zig
if not exist "%ZDIR%\ziglang\zig.exe" (
  echo Installing ziglang compiler to %ZDIR% ...
  python -m pip install --target "%ZDIR%" ziglang || (echo pip install failed & exit /b 1)
)
set ZIG_GLOBAL_CACHE_DIR=%ZDIR%\cache
set ZIG_LOCAL_CACHE_DIR=%ZDIR%\cache
echo Building sim.exe (first build compiles libc++, be patient) ...
"%ZDIR%\ziglang\zig.exe" c++ -std=c++17 -O0 -w -I shims -include shims/Arduino.h sim_main.cpp ..\Sysiphus_Landing.cpp -o sim.exe
if errorlevel 1 ( echo BUILD FAILED & exit /b 1 )
echo.
echo Built sim.exe
echo   Run:   sim.exe ^> flight.csv
echo   Visual: python make_flight_html.py flight.csv flight.html   (then open flight.html)
echo   Plot:   python plot_flight.py flight.csv
endlocal
