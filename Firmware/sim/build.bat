@echo off
REM ============================================================================
REM Build the SIL: compiles the EXACT flight firmware (..\Sysiphus_Landing.ino, -x c++)
REM together with the host harness, using g++ (MinGW-w64). See README.md to get a
REM compiler if "g++ is not recognized".
REM ============================================================================
g++ -std=c++17 -O2 -I shims -include shims/Arduino.h sim_main.cpp -x c++ ..\Sysiphus_Landing.ino -o sim.exe -lm
if errorlevel 1 (
  echo.
  echo BUILD FAILED. If "g++ is not recognized", install a compiler (see README.md).
  exit /b 1
)
echo.
echo Built sim.exe
echo Run:   sim.exe ^> flight.csv     (CSV trace to flight.csv, SUMMARY to the console)
echo Plot:  python plot_flight.py flight.csv
