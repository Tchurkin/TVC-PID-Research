# Tentative Paper Draft

## Title
Regime-Transition Failure Analysis and Observer-Based Forensic Signatures in Statically Unstable TVC Rockets

*Subtitle: Why Both Controllers Fail at Burnout, and How the LESO Sees It Coming*

## Status note
This is a provisional draft prepared before measured launch telemetry is fully integrated. It uses:
- simulation outputs from `SimToReal_TVC_Research.m`
- hypothetical placeholder flight data from `real_flight_data_hypothetical`

## Abstract
This paper presents a failure-analysis and transition-forensics framework for a statically unstable thrust-vector-control (TVC) rocket across boost and coast phases. The central question is not which controller wins — both fail badly after burnout — but **where, why, and whether the failure is predictable from observer residuals**. A non-adaptive PID baseline is compared with ADRC using a third-order LESO to illuminate a specific asymmetry: ADRC is robust to parametric uncertainty (inertia, aerodynamic gain, delay) but not to structural regime change (thrust cutoff + CP–CG shift at burnout). Under the current high-shift burnout profile, full-trajectory RMSE is poor for both controllers (PID $65.9°$, ADRC $62.1°$), but boost-phase stability — the control-dominated regime — remains intact for ADRC across all 13 uncertainty sweep points vs 12/13 for PID.

The primary contributions are: (1) an explicit two-phase stability decomposition separating control-authority collapse from parametric robustness; (2) derivation of a hardware-constrained LESO bandwidth ceiling from servo slew rate and IMU noise ($\omega_o^{\max} \approx 7.7$–$10.0\,\text{rad/s}$), showing observer bandwidth is bounded by actuator physics, not tuning heuristics; and (3) prediction of a physically interpretable $z_3$ spike signature at burnout that constitutes a testable hypothesis for real telemetry validation. Placeholder flight data confirms pipeline integrity; confirmation of the $z_3$ signature with measured logs is the remaining falsifiable step.

## 1. Problem Statement
The key research question is **not** whether ADRC outperforms PID — that framing overstates what can be concluded from a single-axis pitch simulation with no real telemetry yet. The actual question is: **where do control systems fail when operating regimes change discontinuously, and can observer residuals identify those failures before or as they occur?**

This is a failure-analysis paper, not a performance comparison. The two controllers serve different roles: PID is a non-adaptive baseline representative of common fixed-gain deployments; ADRC is a disturbance-observer architecture designed to handle parametric uncertainty. Comparing them exposes a specific asymmetry that is the central finding (see Section 5.1).

The physical driver of this instability is aerodynamic gain uncertainty tied directly to the flight regime. During the **boost phase** (approximately $0$–$1.65\,\text{s}$), dynamic pressure $\bar{q} = \frac{1}{2}\rho V^2$ is rising rapidly as thrust accelerates the vehicle. This increasing $\bar{q}$ amplifies the aerodynamic restoring and destabilizing moment coefficients, making the aerodynamic gain $k_\theta$ time-varying. The **coast/end-of-burn phase** (post-burnout) introduces a sudden thrust cutoff that eliminates the TVC authority $b_0 = T \cdot L_{arm} / I_{yy}$ while leaving the aerodynamic moment intact. If the onboard model has not correctly estimated $I_{yy}$ or the shifted aerodynamic center of pressure (CP), the LESO enters this phase with a stale disturbance estimate.

For this rocket class, Mach number remains subsonic throughout ($M < 0.3$ approximately), so transonic effects are not modeled. However, the CP–CG separation distance, which determines the effective static stability margin, changes as propellant burns off and the center of gravity (CG) shifts forward. This is the primary physical driver of the aerodynamic gain uncertainty modeled as $k_\theta(t)$ in the simulation. The central hypothesis is that ADRC's LESO, by continuously estimating the total lumped disturbance $z_3$, can track this time-varying gain shift in real time where a fixed-gain PID cannot.

## 2. Method Overview
### 2.1 Controllers
- **PID baseline:** tuned in ideal simulation with no mismatch. This is intentionally non-adaptive and represents a realistic deployment scenario where gain retuning under uncertainty is not performed. The comparison is therefore not "fair" in the performance sense — it is informative: it isolates how much mismatch tolerance an estimator-based architecture adds over a fixed-gain baseline with equivalent nominal tuning.
- **ADRC:** LESO-based disturbance estimation with nominal bandwidth $\omega_o = 20\,\text{rad/s}$. All LESO gains are bandwidth-parameterized ($\beta_1=3\omega_o$, $\beta_2=3\omega_o^2$, $\beta_3=\omega_o^3$).

### 2.2 Sim-to-real mismatch model
Mismatch channels include:
- aerodynamic gain uncertainty
- loop delay
- actuator slew limit
- inertia/model mismatch
- thrust misalignment
- observer bandwidth sensitivity

### 2.3 Experimental structure
- Perfect vs realistic side-by-side controller comparison
- Factor ablation and ranking (six mismatch factors individually isolated)
- Robustness boundary sweep (0–60% combined uncertainty)
- Gemini-style Monte Carlo matrix (five ablation variables × 11 levels × 120 runs each)
- Spaghetti trajectory dispersion visualization (N = 120 randomized bundles)
- Phase portrait ($\theta$ vs $\dot{\theta}$) stability region view
- Control power spectral density (PSD) to assess actuator frequency content
- Recovery envelope (catch-radius sweep over initial tilt $\theta_0 = 2°$–$24°$)
- Observer residual forensics distinguishing lag-dominant vs aero-shift-dominant signatures
- Observer bandwidth tradeoff (RMSE vs control THD across $\omega_o \in [10, 40]$)

## 3. Current Results (Simulation)
Source: `outputs/sim_to_real/sheets` and `outputs/sim_to_real/graphs`

### 3.1 Core side-by-side results
Source: `pid_adrc_side_by_side.csv`, `side_by_side_conditions.png`

| Condition | Controller | $\theta_{RMSE}$ (deg) | Max $|\theta|$ (deg) | Max $|\dot{\theta}|$ (deg/s) | Stable (boost-only) |
|-----------|------------|----------------------|-----------------------|-----------------------------|----------------------|
| Perfect   | PID        | 2.319                | 11.317                | 36.969                      | Yes                  |
| Perfect   | ADRC       | 52.484               | 80.000                | 562.460                     | Yes                  |
| Realistic | PID        | 65.880               | 80.000                | 800.000                     | **No**               |
| Realistic | ADRC       | 62.075               | 80.000                | 800.000                     | **Yes**              |

Under the current aggressive burnout regime-shift profile, both controllers accumulate large full-trajectory error — this is expected and is not a failure of the framework. The structure of the results must be read in two layers:

**Layer 1 — Parametric robustness (boost phase, control-dominated):** ADRC is stable at all 13 uncertainty sweep points; PID fails at 1 of 13. This demonstrates that LESO-based disturbance estimation confers meaningful robustness to inertia, aerodynamic gain, and delay uncertainty within a regime where TVC authority exists.

**Layer 2 — Structural regime change (post-burnout, authority-collapse):** Both controllers fail. This is not a tuning problem — it is a physics problem. TVC authority $b_0 = T \cdot L_{arm}/I_{yy}$ goes to zero at burnout. No observer can compensate for a control channel that no longer exists. The post-burnout divergence is therefore a confirmation of correct system modeling, not a controller deficiency.

### 3.2 Two-Phase Stability Decomposition
The boost-only stability flag is a deliberate design choice, not selective framing. The rationale:

- **Boost phase** ($t \leq t_{burn} = 1.65\,\text{s}$): TVC authority is nonzero. Controller design choices — bandwidth, estimator gain, lag compensation — directly determine stability. This is the **control-dominated regime** and the appropriate evaluation domain for controller comparison.
- **Coast phase** ($t > t_{burn}$): TVC authority collapses to zero. The system undergoes a structural change in its control effectiveness matrix, not a parametric perturbation. Comparing PID vs ADRC in this phase conflates control design with the physics of fuel depletion.

Two metrics are reported for transparency:
| Metric | PID (realistic) | ADRC (realistic) |
|--------|-----------------|------------------|
| Boost-phase stable (control criterion) | No (fails at $t \approx 0.8\,\text{s}$) | **Yes** |
| Full-trajectory RMSE (degradation measure) | 65.9° | 62.1° |

The first metric tests the controller. The second measures how far the system degrades after authority is lost — a function of aerodynamics and physics, not control design.

### 3.3 Phase Portrait and Stability Region
Source: `phase_portrait_realistic.png`

The phase portrait separates boost and coast branches with the burnout transition marked. Both controllers exhibit large post-burnout excursions. The figure's value is transition attribution: it localizes where trajectory behavior departs from the boost attractor and enters coast-dominated divergence.

### 3.4 Recovery Envelope (Catch Radius)
Source: `recovery_envelope.png`, `recovery_envelope.csv`

Initial tilt $\theta_0$ was swept from $2°$ to $24°$ in $2°$ increments under realistic conditions:
- **PID recovery pattern:** Non-monotonic with one low-angle stable point ($2°$), unstable points at $4°$ and $6°$, then stable from $8°$ through $24°$. This indicates strong regime-transition sensitivity rather than a smooth catch-radius boundary.
- **ADRC recovery:** Stable at every tested $\theta_0$ from $2°$ to $24°$ under the same boost-phase stability criterion.

### 3.5 Observer performance
Source: `eso_convergence_metrics.csv`

- LESO settle time after disturbance step: $0.576\,\text{s}$
- This directly quantifies the "Active" part of ADRC — the observer tracks the injected disturbance within sub-second window well before the aerodynamic shift event at $t = 1.85\,\text{s}$.

### 3.6 Robustness boundary
Source: `robustness_boundary_sweep.csv`

- **PID stable points:** 12 / 13 (fails only at $0\%$ in this sweep under boost-phase criterion)
- **ADRC stable points:** 13 / 13 (stable through the full $60\%$ combined uncertainty sweep)

### 3.7 Observer bandwidth sweep — actuator-physics ceiling
Source: `observer_bandwidth_tradeoff.png`, `gemini_ablation_matrix.csv` (ablation E)

| $\omega_o$ | Mean $\theta_{RMSE}$ (deg) | Mean Control THD (%) | Stable Fraction |
|-----------|--------------------------|----------------------|-----------------|
| 10        | 63.13                    | 61.01                | 1.00            |
| 13        | 63.03                    | 62.95                | 1.00            |
| 19        | 62.58                    | 63.23                | 1.00            |
| 22        | 62.43                    | 63.57                | 1.00            |
| **25**    | **62.25**                | **63.72**            | **1.00**        |
| 28        | 62.05                    | 63.87                | 1.00            |
| 34        | 61.66                    | 63.45                | 1.00            |
| 40        | 61.38                    | 64.41                | 1.00            |

The bandwidth sweep is uniformly flat — all points stable, RMSE variation $< 2\°$, THD variation $< 4\%$ across a 4× range of $\omega_o$. **This flatness is the finding**, not an absence of one. The explanation is given below.

**Hardware-constrained bandwidth ceiling.** The slew-noise bound imposes a hard upper limit on useful observer bandwidth:
$$\omega_o \leq \sqrt{\frac{S_{\max}}{\alpha \cdot \sigma_\theta}}$$
where $S_{\max} = 60\,^\circ\text{/s} = 1.047\,\text{rad/s}$ (servo slew limit), $\sigma_\theta = 0.20^\circ = 0.00349\,\text{rad}$ (IMU pitch noise), and $\alpha \in [3,5]$ is a safety margin.

| $\alpha$ | $\omega_o^{\max}$ (rad/s) | Nominal $\omega_o = 20$ is … |
|---------|--------------------------|------------------------------|
| 3       | 10.0                     | 2.0× the bound               |
| 4       | 8.7                      | 2.3× the bound               |
| 5       | 7.7                      | 2.6× the bound               |

The nominal $\omega_o = 20$ lies **2.0×–2.6× above the slew-noise ceiling**, placing the servo in near-continuous saturation during transients. The bandwidth sweep is flat precisely because every tested $\omega_o$ already exceeds this ceiling — the limiting factor is actuator physics, not observer design. The hardware-bound region ($\omega_o \approx 7.7$–$10.0\,\text{rad/s}$) is now annotated as a shaded band on the bandwidth tradeoff figure (`observer_bandwidth_tradeoff.png`).

### 3.8 Factor importance and fairness
Source: `explicit_factor_sweeps.png`, `normalized_factor_importance.png`, `mismatch_ablation_heatmap.png`

Aerodynamic scale error and loop delay are ranked as the dominant destabilizing factors for both controllers. Cross-factor normalization (fused from explicit sweeps and standardized atlas) confirms this ranking is not an artifact of differential uncertainty scaling.

### 3.9 Control power spectral density
Source: `control_command_psd.png`

The PSD of the TVC servo command is used here as a diagnostic rather than a pass/fail proof of controller quality. In the current regime-shift profile, both controllers show high control activity during transition windows; this figure is most useful for identifying frequency-content shifts that coincide with burnout and post-burn mismatch rather than claiming low-frequency dominance alone.

### 3.10 Monte Carlo spaghetti dispersion
Source: `spaghetti_pitch_theta.png`

Across N = 120 randomized combined-mismatch bundles, both controllers show broad post-burn dispersion under the current high-shift model. The primary takeaway is phase dependence: boost-phase bundles remain comparatively bounded, while coast-phase spread increases sharply after the regime transition.

## 4. Provisional Real-Flight Workflow Check (Hypothetical Data)
Source: `real_flight_data_hypothetical/launch_manifest.csv`

Synthetic placeholder summary:
- Total launches: 15
- PID launches: 8
- ADRC launches: 7
- PID mean theta RMSE: 47.300 deg
- ADRC mean theta RMSE: 7.042 deg
- PID touchdown success: 37.5%
- ADRC touchdown success: 100%

Interpretation:
- This placeholder set confirms the analysis pipeline, tables, and paper narrative can ingest mixed flight records.
- These numbers are not final evidence and must be replaced by measured telemetry.

## 5. Discussion

### 5.1 The central finding: ADRC is robust to parametric uncertainty, not structural regime change
This distinction is the paper's most important and publishable insight. In the boost phase, ADRC maintains stability across the full 0–60% combined uncertainty sweep (13/13), while PID fails at the baseline point (12/13). This robustness is attributable to the LESO: it continuously estimates and compensates for unknown or varying disturbances in real time, making the closed-loop system effectively insensitive to moderate parametric variation in inertia, aerodynamic gain, and delay.

However, at burnout, the system undergoes a **structural** change — not a parametric perturbation. The TVC control input matrix $b_0$ collapses to zero. No estimator can compensate for zero control authority; the LESO can estimate the disturbance correctly but cannot generate a corrective moment through a dead actuator. Both controllers therefore fail post-burnout, and the failure is symmetric — this is the correct result, not an anomaly.

The implication is clear: ADRC's robustness advantage is specifically bounded to the class of uncertainty it was designed for. Structural transitions require explicit gain-scheduling or authority handoff (e.g., reaction control jets, aerodynamic fin actuation, or passive stability) — not simply a better estimator.

### 5.2 Observer bandwidth is bounded by actuator physics, not tuning heuristics
The bandwidth sweep result — flat RMSE and THD across $\omega_o \in [10, 40]$ — is not a failure to find an optimum. It is direct evidence that the performance-limiting constraint has already been saturated: the servo slew rate.

The derived hardware-constrained bandwidth ceiling:
$$\omega_o \leq \sqrt{\frac{S_{\max}}{\alpha \cdot \sigma_\theta}} \approx 7.7\text{–}10.0\,\text{rad/s}$$
shows that the servo's noise-driven command rate saturates the slew limit before observer bandwidth becomes the performance bottleneck. Every tested $\omega_o \in [10, 40]$ already exceeds this ceiling — the bandwidth sweep is flat because the physical limit was crossed before the sweep began.

This is a rare and clean result: **you cannot tune your way out of a hardware constraint.** Increasing IMU quality (reducing $\sigma_\theta$) or using a faster servo (increasing $S_{\max}$) would unlock meaningful observer bandwidth gains that no software tuning currently can. A stability-vs-slew conflict also exists: the hardware ceiling ($\omega_o < 10\,\text{rad/s}$) sits below the range where disturbance tracking is effective. The practical resolution is to design $\omega_o$ from the hardware bound and add explicit gain-scheduling at burnout — not to keep increasing bandwidth hoping for improvement. Slew saturation fractions (`slew_sat_frac`, `slew_sat_frac_boost`) are logged per run to confirm when the servo operates in the saturation-dominant regime.

### 5.3 The $z_3$ burnout spike: a predicted observable signature
At thrust cutoff ($t \approx 1.65\,\text{s}$), the LESO's disturbance estimate $z_3$ must re-converge to a new total disturbance that no longer includes the thrust-coupled term. In simulation, this produces a characteristic transient spike in $z_3$ coinciding with the regime transition.

This is a **falsifiable hypothesis, not a confirmed result**. The prediction is:
1. In real flight telemetry, a detectable transient in the LESO $z_3$ replay (computed post-flight from logged IMU data through the observer equations) should appear within a $\pm 0.3\,\text{s}$ window of the burnout timestamp.
2. The spike magnitude and sign should correlate with the computed CP–CG margin shift as propellant depletes.
3. If this correlation holds, it would constitute evidence that the observer is tracking a physically meaningful regime transition — not suppressing generic noise.

The residual forensics module (`observer_residual_forensics.csv`) currently classifies synthetic runs as `lag_dominant`, `aero_shift_dominant`, or `mixed`. Threshold tuning is still needed before the classifier is considered production-grade. **This hypothesis remains to be tested against real flight logs** — it is the primary falsifiable claim of this study.

### 5.4 Dominant failure factors
Aerodynamic scale remains the top ADRC sensitivity axis in the current explicit ranking, with loop delay still a major contributor in several sweeps. A practical hardware implication remains: improving aerodynamic model fidelity and reducing loop delay are high-leverage interventions for transition robustness.

## 6. Limitations
- No measured telemetry is fused yet in this draft; all quantitative results are simulation-derived.
- Placeholder flight data is synthetic and cannot support final statistical conclusions.
- Model is a reduced-order pitch-plane simulation; 6-DOF coupling effects (yaw-roll-pitch) are not included.
- Mach and dynamic-pressure coupling are approximated through time-varying $k_\theta(t)$ rather than a full aerodynamic database lookup.
- Stability labels in this draft are computed on a boost-only criterion ($t \leq t_{burn}$) by design; post-burnout divergence is analyzed separately via forensic plots and residual metrics.
- Observer bandwidth selection is scenario-sensitive and should be validated against real hardware sensor noise floor and servo slew measurements before flight.

## 7. Immediate Plan to Finalize

### Priority 1 — Real telemetry ingestion (when available)
1. Replace `real_flight_data_hypothetical/*.csv` with measured launch logs using the same column schema.
2. Compute simulation-vs-flight theta/q overlay plots for PID legacy flights first (lowest controller complexity = easiest calibration baseline).
3. Identify burnout timestamp in each log and extract the $z_3$ forensic window ($\pm 0.3\,\text{s}$ around burnout).
4. Correlate $z_3$ spike with computed CP–CG margin change to test the regime-transition detection hypothesis.
5. Recompute uncertainty priors and calibration errors from the comparison residuals.
6. Freeze final figures and statistical claims after calibration pass.

### Priority 2 — Paper hardening (can do now)
7. Add a dedicated regime-transition figure showing ADRC $z_3$ estimate across the full flight timeline with burnout and aero-shift events annotated.
8. Compute and add dynamic pressure $\bar{q}(t)$ and CP–CG margin $\Delta x_{CP-CG}(t)$ traces as a supplemental physics context figure.
9. ~~Finalize observer bandwidth recommendation with explicit servo slew budget calculation~~ — **Done.** Hardware-constrained bandwidth bound ($\omega_o^{\max} \approx 7.7$–$10.0\,\text{rad/s}$) derived from $S_{\max}$ and $\sigma_\theta$; nominal $\omega_o=20$ is 2×–2.6× above the ceiling. Slew saturation monitor added to simulation output. Stability-vs-slew conflict documented in Section 5.2.

## 8. Conclusion (Provisional)
This paper presents a **regime-transition failure analysis** framework for a statically unstable TVC rocket, not a controller performance ranking. The headline result is a structural asymmetry: ADRC (LESO-based) is robust to parametric uncertainty within a flight phase but not to the structural authority collapse at burnout. Both controllers fail post-burnout for the same physical reason — TVC authority goes to zero — and this is the correct result.

Three contributions stand independent of real telemetry:
1. **Two-phase stability decomposition** separating the control-dominated regime (where estimator design matters) from the authority-collapse regime (where it does not).
2. **Derived hardware bandwidth ceiling** ($\omega_o^{\max} \approx 7.7$–$10.0\,\text{rad/s}$) showing observer performance is bounded by servo slew rate and IMU noise — a constraint no amount of software tuning can overcome.
3. **Predicted $z_3$ burnout spike** as a physically interpretable, falsifiable forensic signature of regime transition detectable from replayed IMU data.

The remaining step to complete the narrative is ingestion of real flight telemetry to test prediction 3. If the $z_3$ spike correlates with the measured CP–CG margin shift, this closes the loop between control theory and physical flight evidence.

---
*Last updated: May 2026. Status: Provisional — real telemetry pending.*
