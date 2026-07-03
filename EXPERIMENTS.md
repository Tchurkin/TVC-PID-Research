# EXPERIMENTS.md — Experiment Records

> One entry per experiment (research, controls, tuning, algorithm work). Append new entries at the
> top. Keep the link to the script and output CSV so results are reproducible. The exhaustive
> historical record lives in `CLAUDE.md`; this file is the going-forward, structured log.

Last updated: 2026-06-30

---

## TEMPLATE (copy this to the top for each new experiment)
```
## YYYY-MM-DD — <title>  [script: tools/xxx.py → experiments/results/xxx_py.csv]
- Hypothesis:
- Experiment:           (design, n, fidelity, controllers)
- Parameters:           (seed ranges, gain grids, sweep ranges — must be disjoint from prior passes)
- Expected outcome:
- Actual outcome:       (with uncertainty: CI / CV / n)
- Remaining questions:
- Recommended next experiment:
```

---

## 2026-07-01 — SIL envelope sweep under the honest fold-out fin plant + EKF lead-comp  [Firmware/sim/sweep_html.py → Firmware/sim/sweep.html]
- **Hypothesis:** with (a) EKF measurement lead-compensation, (b) a stop-margin hoverslam, (c) the honest
  CP-shift (fold-out, migrating-CG) fin plant and trim-ride coast controller, the full envelope lands
  soft + on-target + upright — including the cells that failed under the flattering plant.
- **Experiment:** exact firmware in SIL; targets {0,8,16,24} m × wind {−4,−2,0,2,4} m/s × gust seeds {1..5}
  (100 flights) + off-grid probes W=±5,±6 (T16, 2 seeds).
- **Parameters:** STOP_MARGIN_M=0.50, BARO_LAG_S=0.040, GPS_LAT_S=0.20, LEG_H=0.08, WC_COAST/W0_COAST=2/10,
  SERVO_SLEW_DPS=150; fin plant S_FIN=0.0009/CN 2.0/l_f0 0.30 vs body CN 2.0/l_b0 0.033, CG_PROP_M=0.009,
  CG_LEGS_M=0.030 (dep* ≈0.37 launch → 0.48 coast).
- **Expected outcome:** ≥ the prior 94/100; elimination of the W+4 far-target coast divergence.
- **Actual outcome:** **100/100 PASS** (+ all 8 off-grid probes). vz worst 4.9 m/s (typ ~3.5), |miss| ≤2.3 m
  (sweep medians ≤1.6), tilt ≤3.1°. Single-flight forensics in DESIGN_LOG pm-4 (headwind capture-divergence,
  fin rail chatter, hop asymmetry).
- **Remaining questions:** the vz floor is now set by the deliberate hot-arrival margin (~3.5 typ) — trading
  STOP_MARGIN_M down needs tighter burn-scatter knowledge (thrust tolerance, gust spectrum). Real leg geometry
  (LEG_H, crush) is a guess pending hardware.
- **Recommended next experiment:** hardware bench — measure the real margin-servo slew + fin fold-out aero,
  then re-run the sweep with measured constants.

## 2026-06-30 — Pad calibration + INS gravity-projection coupling (what actually limits a real flight)  [tools/landing_interactive.py]
- **Hypothesis:** Modeling the real INS drift mechanism (attitude error → gravity/thrust leaks into horizontal accel)
  and pre-launch pad calibration will show calibration — not GPS — is the dominant precision lever for a cheap IMU.
- **Experiment:** Strapdown EKF reads the accelerometer in BODY frame, rotates by the ESTIMATED attitude (so th_err
  leaks gravity/thrust into horizontal); add pad calibration (estimate biases from 600 stationary samples → subtract)
  and accel misalignment (axial→lateral at high g; pad cal removes only the g-part). 2×2 {calib on/off}×{GPS on/off},
  20 realizations × {tg 0,20,40}, wind=4.
- **Parameters:** GYRO_BIAS 0.005 rad/s, ACCEL_BIAS 0.10 m/s², ACCEL_MISALIGN 0.015 rad; cal residual = noise/√600;
  KF P0=diag(0.04,0.04,0.04), Qh bias-walk 1e-6 (track the leak). GPS 8 Hz, bias 0.8 m, noise 0.6 m.
- **Expected:** Calibration cuts attitude tilt and (via the coupling) position drift; GPS payoff depends on IMU quality.
- **Actual:** Calibration is **dominant** — no-GPS miss 2.92→0.26 m, tilt 4.1°→1.0°, est-error 3.46→0.06 m. **GPS
  helps iff IMU-drift > GPS-bias:** uncalibrated GPS helps (2.92→0.81 m); **calibrated GPS HURTS** (0.26→0.81 m) — the
  loosely-coupled KF is pulled into the ~0.8 m correlated GPS bias it can't reject. Misalignment **cancels** over a
  return-to-rest hop (∫(SF−g)=Δv=0) → not a short-hop driver. wind=0 regression perfect.
- **Remaining questions:** Add scale-factor/vibration/temperature errors (calibrated-no-GPS 0.26 m is a best case);
  does the GPS-hurts result flip for longer flights (unbounded IMU drift) or worse IMUs? Model the GPS bias as a state?
- **Recommended next:** For the real flight — **pad-calibrate the IMU** (biggest single win); treat GPS as robustness
  insurance for long/uncalibrated cases, not a short-hop precision source. Build a fuller accel error budget before
  trusting sub-meter no-GPS.

## 2026-06-30 — EKF replaces ad-hoc fusion; vertical "lands downwind" diagnosed  [tools/landing_interactive.py]
- **Hypothesis:** The vertical-launch downwind landing is an *estimator* error (not an aim/tilt error), and a proper
  covariance-weighted EKF (estimating the accel bias) will tighten the miss and remove the bias vs the ad-hoc filter.
- **Experiment:** (1) Aim check — ideal deterministic ballistic + divert signed landing for tg0 across wind, to
  isolate aim bias from estimator error. (2) Replace per-channel hack with two per-axis 3-state KFs `[pos,vel,accel
  -bias]` (strapdown accel predict; baro every step + GPS 8 Hz pos+vel updates). Compare EKF vs ad-hoc: |miss|,
  signed miss, estimate error, gimbal reversals; GPS on/off; 25 gust realizations.
- **Parameters:** P0=diag(0.04,0.04,σ_accelbias²); Qh=diag(1e-7,(accel_noise·dt)²,1e-8); R=sensor σ². GPS 8 Hz,
  bias σ=0.8 m, noise σ=0.6 m. wind ∈ {0,±5}; tg ∈ {0,20,40}; T/W=2. Seeds = flown-pass randoms (search stays exact).
- **Expected:** Aim lands at x=0 exactly (findKick is exact); EKF est-error < ad-hoc ~1 m; downwind bias → ~0.
- **Actual:** **Aim is exact** — ballistic AND divert land x=**0.00** at every wind (kick −13.7°@3, −19°@5). So the
  rocket *was* tilting precisely enough; the downwind landing was the ad-hoc estimator (~1 m position error) flying
  the rocket to a wrong estimate. **EKF result (GPS on):** miss 0.6–1.1 m (ad-hoc ~1.0), est-error **0.57 m** (ad-hoc
  ~1.0), reversals 11–12 (smooth); vertical signed miss **−0.5 m (upwind, unbiased)**, residual ≈ irreducible GPS
  bias (~1 m CEP). GPS off: horizontal unobservable → drifts 1.6–1.8 m miss, 2.1–2.3 m est-error. wind=0 regression
  perfect (miss 0.00, vz −0.9…−1.6). The accel-bias state is the key: it estimates & removes the drift source.
- **Remaining questions:** Attitude is still gyro-only (no in-flight reference) — does a tilt/mag reference help? Can
  RTK (0.05 m) + velocity tightening reach the ideal 0.0 m? Is the per-axis decoupling (folding attitude-rotation
  error into accel-bias) adequate at higher tilt / longer flights?
- **Recommended next:** If chasing sub-meter, add a tilt reference for attitude + RTK; otherwise the ~1 m EKF floor
  is the honest consumer-GPS limit and is fine for a first flight.

## 2026-06-30 — Does a GPS module help the landing? (sensor BOM)  [tools/landing_interactive.py]
- **Hypothesis:** Adding GPS (absolute position) recovers the precision lost to IMU drift.
- **Experiment:** Loosely-coupled GPS/IMU fusion (5 Hz, per-flight correlated bias + per-fix noise, good Doppler
  velocity). Compare miss: ideal / IMU+baro / +consumer GPS / +RTK GPS. 25 realizations × {wind 0,5} × {tg 0,20,40}.
- **Parameters:** consumer: bias σ=1.1 m, noise σ=0.8 m (~1.4 m CEP), vel noise 0.08 m/s, KP=0.35, KV=0.5. RTK: bias
  0.08, noise 0.05, 10 Hz.
- **Expected (going in):** consumer GPS might not beat IMU on a short flight; RTK would.
- **Actual:** ideal 0.0–0.3 m; IMU+baro 0.8–1.2 m; **+consumer GPS 1.0–1.3 m (NO improvement, slightly worse at
  wind 0)**; +RTK 0.5–0.8 m. Consumer GPS (~1.4 m) ≈ the IMU drift over ~8 s, and its correlated bias doesn't average
  out → fusing it adds noise without helping. RTK fixes position → ~0.5 m, but does NOT reach ideal because the
  residual is now velocity (0.08 m/s), attitude (~1°) and baro (0.3 m), not position.
- **Remaining questions:** Would tight velocity (RTK Doppler) + a better attitude filter close the RTK→ideal gap?
  Does GPS help once the flight is longer (IMU drift unbounded) or the IMU is cheaper (bigger drift)?
- **Recommended next:** For precision landing, spec **RTK GPS** (a standard module won't tighten a 10 s flight) and
  then attack the velocity/attitude residual. For a first flight, IMU+baro (~1 m) is adequate; GPS adds robustness.
- **UPDATE (MPU6050-specific):** with REAL MPU6050 specs (accel bias ~10 mg → ~2.5 m drift, gyro-only attitude),
  the verdict **flips**: MPU6050-only 1.6–2.7 m (max ~9 m) → MPU6050+reasonable-GPS **~1.0 m** — GPS halves it and is
  worth it. The "consumer GPS doesn't help" result above held only for an *optimistically good* IMU. Lesson: the GPS
  payoff is entirely a function of IMU quality. Residual with MPU6050+GPS: ~1 m (GPS, → RTK) + ~3–6° tilt (gyro, →
  better/calibrated gyro). Also raised the near-ground divert threshold to 1.5 m so it stops chasing GPS position
  noise into a tip-over (tilt 7–9° → 3–6°). [memory ROUND 25]

## 2026-06-30 — Ideal vs realistic IMU+baro sensing (precision floor)  [tools/landing_interactive.py]
- **Hypothesis:** With realistic hobby sensing (gyro+baro+accel, NO GPS) the propulsive-landing controller still
  works, but horizontal-position-estimate drift caps precision well above the sub-meter ideal-state result.
- **Experiment:** Run the flown landing on fused estimates vs true state; 25 gust realizations per (wind, target).
- **Parameters:** gyro bias σ=0.0015 rad/s + noise 0.002; accel bias σ=0.035 m/s² + noise 0.025; baro noise 0.30 m;
  vert-vel noise 0.30 m/s. wind ∈ {0,±5}; target ∈ {0,20,40}; T/W=2.
- **Expected:** Realistic miss ≈ the position drift (~1 m), roughly target/wind-independent.
- **Actual:** IDEAL sensors miss 0.0–0.3 m. REALISTIC miss **0.8–1.5 m** (max 2–4 m), pos-estimate drift **0.8–1.4 m**
  — miss ≈ drift, dominated by IMU integration error, NOT wind. Touchdown still near-vertical (2–8°). Bug found &
  fixed: apogee detection on instantaneous noisy vz false-triggered on the pad → use baro-trend (true-vz threshold).
- **Remaining questions:** How much does an absolute-position sensor (GPS/vision/beacon) recover? Does an attitude-
  error→accel cross-coupling term make the drift worse (cubic) over longer flights?
- **Recommended next:** Add a GPS/beacon toggle that bounds the position estimate and re-run — quantify the precision
  you buy with absolute position (expected: back toward sub-meter). That directly informs the real-flight sensor BOM.

## 2026-06-30 — Landing wind + ADRC + nominal-tracking divert  [tools/landing_interactive.py]
- **Hypothesis:** A translational ADRC correcting only the deviation from a gust-free nominal will land
  on the pad under stochastic wind without overshoot or off-angle touchdown.
- **Experiment:** Headless Node runs of the generated JS sim; 15 stochastic-gust realizations per
  (wind, target). Attitude ADRC + translational ADRC (ESO on x-deviation); decelerating coast-flip slew.
- **Parameters:** wind ∈ {0, ±4, ±7} m/s; target ∈ {20, 40, 55} m; T/W=2; wcx=2.5, w0x=10, STILT2=24°.
- **Expected:** On-target, vertical, soft across the range.
- **Actual:** wind=0 all targets miss=0.00, td_angle=0°, vz −0.4…−1.6 (perfect). wind ±4…±7, tg40/55:
  miss <1.5 m, td_angle 4–8°, vz −2.3…−4.7, flip overshoot 0–5°. **Edge cases fail:** tg20 in strong
  wind (overshoot 52–68°, miss 3–4 m) and far+headwind (miss ~7 m) — control-authority limits.
- **Remaining questions:** Does boostback-to-above-pad + vertical descent remove the edge-case failures?
  Is the firmer wind touchdown (vz −2…−5) acceptable, or worth a throttle/2-burn model?
- **Recommended next:** Prototype a boostback descent (fly above the pad, hold x=target on vertical
  descent) and compare edge-case miss/angle to the nominal-tracking divert.

## 2026-06-24 — Saturation-transition replication (n=142)  [tools/saturation_transition_large.py]
- **Hypothesis:** ρ(log Π, f_sat) ≈ 0.80 from the n=29 regime map holds at larger n.
- **Experiment:** Same probe protocol (Kp=190/lat) at ~5× sample; binned dose-response + causal
  no-saturation test.
- **Parameters:** n=142, probe seeds as in the n=29 map; latency 1–6 main population.
- **Expected:** Replicate ρ≈0.80.
- **Actual:** ρ dropped to **0.55** (honest revision down). Dose-response stayed clean & monotonic;
  causal SR_nosat≈0.99 across all 142 (Finding 8 extended). On latency 1–6, Π **ties** keff (τ² adds
  no marginal ranking power — latency range too narrow).
- **Remaining questions:** Does τ² out-rank τ¹ on the latency 1–12 stress population?
- **Recommended next:** Re-fit the τ-exponent on the lat 1–12 pool with the selection-artifact control.

<!-- Add new experiments above this line. -->
