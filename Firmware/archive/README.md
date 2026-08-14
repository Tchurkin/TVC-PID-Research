# Archived firmware

Superseded sketches, kept because they carry flight heritage and tuning numbers that
predate Impulse 2.2. **None of these run correctly on Impulse 2.2** — they assume the
2.0/2.1 pin map, the MPU-6050 + BMP280 pair on I²C, or both. Read them for history,
don't flash them.

Archived 2026-08-13, during the Impulse 2.2 sensor bring-up.

| Archived | Superseded by | Why |
|---|---|---|
| `SisyphusCode.ino` | `../Sysiphus_Landing.ino` | Original landing firmware. Old pin map, `MPU6050_tockn`. |
| `Ascent_Test.ino` | `../Ascent_TVC/` | Predates the quaternion estimator, the fault ladder, keff identification and every Impulse 2.2 gate. |
| `Sensor_Test/` | `../Impulse22_SensorCheck/`, `../Impulse22_Bringup/` | Reads only the I²C backups, which are not fitted. No ICM-42688-P, no DPS310, no GPS. |
| `Preflight_Test/` | `../Impulse22_Bringup/` + bench TVC mode in `../Ascent_TVC/` | Old servo pin map; its "Live TVC" mode is now a button-hold inside the real flight firmware, so the signs it verifies are the ones that actually fly. |
| `Pyro_Test/` | `../Impulse22_PyroTest/` | Knows only P1/P3/P4 — Impulse 2.2 has four channels. No continuity sensing, and a 1000 ms pulse that will destroy Q21 on a low-resistance load. |
| `Bench_Tau_BNO055/` | `../Bench_Tau_ICM/` | BNO055 is not fitted on this vehicle. |

## Still live, deliberately

- `../Ascent_TVC/` — the ascent flight firmware
- `../Sysiphus_Landing.ino` — landing firmware. **Not yet ported to Impulse 2.2**: it still
  uses `Serial1` for GPS (pins 0/1 are CS_IMU and MISO), servo pin 2 (that's INT_IMU),
  900 ms on all four pyro channels, and has no ICM-42688-P or DPS310 driver at all.
- `../Impulse22_*` — bring-up, sensor check, pyro test, LED test
- `../Bench_Latency/`, `../Bench_Tau_ICM/`, `../Bench_ThrowCheck/`, `../Bench_FindLimit/`,
  `../StaticFire/` — bench instruments
- `../feedback_servo_calibration.ino`, `../gimbal_characterization.ino` — utilities, and they
  already use the current `SERVO_X_PIN=4 / SERVO_Y_PIN=3` convention
- `../sim/`, `../sim_ascent/` — SIL harnesses and the regression gate
