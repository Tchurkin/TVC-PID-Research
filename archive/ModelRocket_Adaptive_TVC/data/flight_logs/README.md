# Flight Logs

Drop raw firmware CSV logs here after each flight.

Expected source:
- Teensy SD log files produced by Firmware/SisyphusCode.cpp
- Typical names: RES000.CSV, RES001.CSV, ...

Required columns (exact firmware header):
- TimeMs, State, AbortReason
- ThetaRad, QRad_s, UAct, UCmd
- AltM, VertVelMps, HighAltM
- KeffEst, SlewEst, GainScale, KeffTheta, KeffQ
- DemandRate, DemandRateDecoupled, AbsDuObs, Confidence
- Saturating, SatStreak, ActFeedback
- ShieldSlew, ShieldAttitude, ComputeUs

Run pipeline:
```matlab
cd ModelRocket_Adaptive_TVC/tools
run_launch_validation_pipeline
```

Generated outputs:
- outputs/flight_validation/launch_validation_summary.csv
- outputs/flight_validation/launch_validation_report.md
- outputs/flight_validation/graphs/*_overview.png
