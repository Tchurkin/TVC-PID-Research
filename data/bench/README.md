# Bench Data

Place real gimbal bench-test CSV files in this folder.

Required columns:
- `time_s`
- `cmd_deg`
- `meas_deg`

Default runner input path:
- `gimbal_bench_test.csv`

Suggested experiment types:
- neutral-to-step commands for delay and rise-time estimation
- positive and negative large steps for slew-rate estimation
- small-amplitude commands around zero for deadband estimation
- slow triangle-wave sweep for hysteresis estimation
