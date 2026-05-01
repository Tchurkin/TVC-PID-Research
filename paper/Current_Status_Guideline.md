# Current Status Guideline

## Where we are now
- Sim framework is operational and reproducible via `matlab -batch "SimToReal_TVC_Research"`.
- Outputs are organized into:
  - `outputs/sim_to_real/graphs`
  - `outputs/sim_to_real/sheets`
- Placeholder flight-data folder is created and populated:
  - `real_flight_data_hypothetical`

## Current evidence snapshot
- Simulation side-by-side (realistic condition):
  - PID RMSE: 61.076 deg
  - ADRC RMSE: 4.321 deg
  - PID stable: false
  - ADRC stable: true
- LESO settle time after disturbance step: 0.670 s
- Robustness sweep points stable:
  - PID: 0 points
  - ADRC: 13 points
- Observer bandwidth tradeoff currently suggests a practical "golden" region near omega_o ~ 25 (based on RMSE + THD score).

## Hypothetical dataset snapshot (not real validation)
- 15 synthetic launches total:
  - PID flights: 8
  - ADRC flights: 7
- Synthetic manifest means:
  - PID theta RMSE mean: 47.300 deg
  - ADRC theta RMSE mean: 7.042 deg
  - PID max abs theta mean: 62.223 deg
  - ADRC max abs theta mean: 10.783 deg
  - PID touchdown success: 37.5%
  - ADRC touchdown success: 100%

## What this means
- The simulation story is strong and internally consistent.
- The paper can be drafted now as a provisional report.
- Final sim-to-real validation remains blocked on replacing synthetic placeholder logs with measured launch data.

## Immediate next steps
1. Replace `real_flight_data_hypothetical/*.csv` with measured logs in same column format.
2. Add calibration overlay figures (sim vs measured theta(t), q(t), delta(t)) for PID flights first.
3. Recompute comparison tables and update draft paper sections marked "provisional".
4. Freeze final paper figures and methods once real-data overlays are stable.
