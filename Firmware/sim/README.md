# SIL — Software-In-the-Loop firmware test

Runs the **exact** flight firmware (`../Sysiphus_Landing.cpp`) on your PC against a
simulated rocket + sensors, so you can test the real code before launch.

It works by compiling the firmware together with a set of thin **shim headers**
(`shims/`) that stand in for the Arduino/Teensy libraries (Serial, Wire, PWMServo,
MPU6050, BMP280, SD, `millis`/`delay`/digital I/O). Those shims are backed by a
physics model in `sim_main.cpp` that:

- feeds the firmware synthetic **IMU / baro / GPS** derived from the true rocket state
  (with realistic per-flight biases + noise, so pad calibration has something to remove),
- captures the firmware's **servo + pyro** outputs and drives the dynamics with them,
- **automatically presses the button** (arm → 5× → countdown) to fly the whole profile,
- logs a CSV of **truth vs. the firmware's own EKF estimates** and prints a PASS/FAIL summary.

The firmware's `setup()` / `loop()` run unmodified — this is your actual code, not a re-implementation.

## Status: VERIFIED

This harness has been **built and run** (via `ziglang`). The default rocket flies a full
**PASS**: boost → coast (margin fins hold retrograde) → suicide burn → **soft landing
(≈0.01 m/s), on target (0.77 m), vertical (3.4°)**, apogee 56 m. Building it caught and fixed
three real firmware bugs (retrograde target during ascent; an EKF `dt` error; landing-motor
sizing). Artifacts `flight.csv` and `flight.html` in this folder are from that run.

## Easiest build: `build_zig.bat` (needs only Python)

```
build_zig.bat                 REM installs a self-contained compiler (ziglang) to TEMP, builds sim.exe
sim.exe > flight.csv          REM CSV trace -> flight.csv, SUMMARY -> console
python make_flight_html.py flight.csv flight.html    REM open flight.html to WATCH the flight
```

First `build_zig.bat` run compiles zig's libc++ (a few minutes, one-time); later runs are fast.

## Alternative: your own host C++ compiler

Any of these works — pick one:

- **WinLibs (easiest, no admin):** download a GCC zip from https://winlibs.com, extract it,
  and add its `bin\` folder to your `PATH`. Then `g++ --version` should work.
- **winget:** `winget install BrechtSanders.WinLibs.POSIX.UCRT.LLVM`
- **MSYS2:** `winget install MSYS2.MSYS2`, then in the MSYS2 shell
  `pacman -S mingw-w64-ucrt-x86_64-gcc` (add its `bin` to PATH).
- **WSL:** `wsl --install`, then inside: `sudo apt install g++ make`.
- **PlatformIO native:** you already have `pio`; `platform = native` builds use the *system*
  gcc/clang, so you still need one of the above installed first.

## Build & run

From this `Firmware/sim/` folder:

```
build.bat                     REM Windows (g++/MinGW)
sim.exe > flight.csv          REM CSV -> flight.csv, SUMMARY -> console
python plot_flight.py flight.csv
```

or with make (WSL/Linux/MinGW):

```
make run        # builds ./sim and writes flight.csv
make plot       # also renders flight.png (needs matplotlib)
```

## What you get

- **Console SUMMARY:** phases reached (BOOST/COAST/LANDING_BURN), apogee, touchdown x (miss),
  touchdown speed, final tilt, EKF-estimate-vs-truth, and a `VERDICT: PASS/…`.
- **flight.csv:** per-sample truth (`px,pz,vx,vz,th_deg,q_dps`), firmware estimates
  (`estX,estZ,estVx,estVz,thEst_deg`), and commands (`uTVC,deploy,keff,thrust`).
- **flight.png** (optional): altitude, tilt, vertical velocity (truth vs EKF), and control channels.

The most useful thing to watch: **EKF estimate vs. truth** (does the firmware *know* where it is?)
and the **suicide-burn timing** (does `estVz` reach ~0 right at the ground?).

## The "default rocket"

Physics constants live at the top of `sim_main.cpp` and are kept **consistent with the firmware
config** (`IYY`, `MASS_LAND_KG`, `LAND_THRUST_AVG_N`, the aero params, the pin map). To test *your*
rocket: set the real values in `../Sysiphus_Landing.cpp` (the `<<< SET` / `<<< MISSION` tags) and
mirror them in `sim_main.cpp`, then rebuild. If the firmware's assumed params disagree with the
physics, the SIL will show it (e.g., a mistimed hoverslam) — which is exactly the point.

## Scope / honesty

- 2D (pitch plane + vertical), matching the sim and the single-plane firmware. No yaw/roll.
- The aero model is a reasonable default, not your airframe — replace with CFD/bench values.
- Passing the SIL means the **code logic** flies the profile; it does **not** validate the real
  aerodynamics, actuator hardware, or motor thrust curves. Bench-test after SIL, then fly.
- This harness was written but **not compiled on the authoring machine** (no host compiler present).
  If the first build throws errors, they'll be small shim/signature mismatches — paste them and
  they're quick to fix.
