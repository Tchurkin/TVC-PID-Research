# Hypothetical Real Flight Data (Placeholder)

Purpose:
- This folder contains synthetic placeholder flight data for workflow development while real launch logs are pending.
- Values are intentionally realistic in range but are not measured experimental truth.

Files:
- launch_manifest.csv: Per-flight summary for 15 hypothetical launches.
- imu_theta_q_timeseries.csv: Time-series IMU/control traces in long format.
- flight_event_markers.csv: Key event times (burnout, disturbance, apogee) per launch.

Important notes:
- Do not use these files as final validation evidence.
- Replace with measured logs once available and keep the same schema to avoid pipeline changes.

Schema highlights:
- launch_manifest.csv columns:
  launch_id, date_utc, controller, wind_mps, payload_kg, theta_rmse_deg, max_abs_theta_deg, max_abs_q_deg_s, touchdown_ok, note
- imu_theta_q_timeseries.csv columns:
  launch_id, controller, time_s, theta_deg, q_deg_s, delta_cmd_deg
- flight_event_markers.csv columns:
  launch_id, burnout_s, disturbance_s, apogee_s
