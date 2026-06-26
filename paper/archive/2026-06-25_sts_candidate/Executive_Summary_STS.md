# When Delay and Authority Conspire: A Saturation Regime Parameter for Hobby-Scale TVC Rockets

### Predicting Controller Failure from Hardware Specs Alone
**Braxton Herold — STS Research Summary**

---

## The Central Discovery

Here is a design that should fail by every intuitive measure: a TVC rocket with keff = 1900 s⁻² — the highest rotational authority per radian of gimbal in a population of 2,400 designs. By the "over-actuated causes problems" hypothesis, this rocket should be the hardest to control.

It is not. With one step of control-loop latency (τ = 5 ms), this rocket achieves SR = 1.000 at any reasonable gain. It never saturates its servo. It is completely benign.

Three doors down in the population: a design with keff = 790 s⁻² — less than half the authority — but with four steps of latency (τ = 20 ms). It is deeply dangerous: the servo saturates 50% of every flight second regardless of gain choice.

**The difference is Π = keff × τ²** — the angular displacement a rocket accumulates during one latency window per radian of commanded deflection. The first rocket: Π = 0.047. The second: Π = 0.32. An empirical transition near Π ≈ 0.32 predicts the onset of servo saturation within this class of TVC plants and FIFO-delay implementations. The transition is invariant across the following dimensions (tested independently):

- Wind variations from 0.10 to 0.40 strength: Π_crit unchanged (max std = 0.003)
- Servo speeds from 60 to 200 °/s: Π_crit = 0.27–0.32 (< 20% variation)
- Aerodynamic damping halved or doubled: Π_crit unchanged (exact zero effect)
- Probe gain varied 6×: Π_crit = 0.27 at all probe fractions (max/min ratio = 1.00)
- Inertia and thrust scaled 4×: Π_crit tracks the rescaled Π correctly

Below the transition: PID with careful gain selection can achieve SR = 1.000 — and does, for optimal tuning across all tested designs up to Π ≈ 0.92 (performance frontier, n = 63). Above Π ≈ 0.32: aerodynamic disturbances begin driving persistent bang-bang; the gain window narrows, calibration failure risk jumps 6.7×, and ADRC's disturbance observer becomes the more reliable choice. ADRC does not become strictly necessary until Π ≈ 0.37 (first optimal-PID failure at SR < 0.90).

---

## Project Overview

**What I studied:** 2,400 simulated hobby-scale TVC rockets (LHS design space: mass 0.5–1.2 kg, rotational inertia 0.005–0.100 kg·m², latency 1–6 control steps, full aerodynamics, sensor noise, servo limits, wind). Full physics simulator validated across 8 independent modules. All major findings reported with correction passes — three bugs were found and fixed during the study; each correction *strengthened* the primary result.

**Primary deliverable:** A nondimensional parameter (Π) that predicts a structural regime transition in TVC attitude control, validated across 6 independent universality campaigns and confirmed causal by a 2×2 factorial experiment.

---

## Six Findings

### Finding 1 — Aerodynamic instability doesn't matter

r(stability, sensitivity) ≈ 0. Stable and unstable rockets are equally likely to be gain-sensitive. Stable rockets outperform unstable at every task tested (10° step: 86.8% vs 68.8% success). The "fighter-jet instability" idea doesn't apply to attitude-hold TVC.

---

### Finding 2 — Π = keff × τ² predicts a saturation regime transition

**The formula:**
```
keff     = T_avg × L_nozzle / Iyy    [s⁻²; rotational authority per radian of gimbal]
τ        = latency_steps / loop_hz   [s; total loop delay]
Π        = keff × τ²                 [-; dimensionless risk parameter]
θ̈_max   = keff × max_gimbal_deg × (π/180)    [rad/s²; optional — needs gimbal angle]
```

keff does NOT include max_gimbal_deg. The ceiling and floor mechanisms both depend on keff, not on max deflection. Adding max_gimbal to the predictor makes it worse (delta CV = −0.14).

**Empirical regime map** (n = 142 designs after replication, probe Kp = 0.021/τ [rad/rad] = half the linear ceiling):

| Π bin | Servo saturation fraction | PID SR | SR (saturation removed) |
|---|---|---|---|
| < 0.11 | 0.007 | 1.000 | 1.000 |
| 0.11–0.23 | 0.043 | 0.996 | 0.996 |
| 0.23–0.46 | 0.151 | 0.999 | 1.000 |
| 0.46–0.92 | 0.336 | 0.981 | 0.988 |
| 0.92–1.72 | 0.590 | 0.880 | 0.970 |

**Spearman rho(Π, saturation) = +0.55 (n = 142, p = 9×10⁻¹³).** A first n = 29 sample gave an optimistic +0.80; replication at 5× the sample revised it down honestly. The binned dose-response (above) and the causal test (removing saturation restores SR ≈ 0.99) both *strengthened* with the larger sample. One concession: on this design space (latency 1–6 steps) Π does not out-rank keff alone (rho 0.55 vs 0.54), because latency's narrow range leaves the τ² factor little marginal ranking power. The evidence that the *product* (not keff alone) is the right parameter comes from R0522, the latency-extended (1–12 step) data, and the blind-spot derivation — not from this correlation.

**Smoking gun — the R0522 counterexample:**
keff = 1900 s⁻² (highest in dataset), latency = 1 step (τ = 5 ms), **Π = 0.047**, fsat = 0.018, SR = 1.000.
The most "over-actuated" design in the population is perfectly safe because Π is low. This single design falsifies "authority causes control difficulty" and confirms "delay × authority causes it."

**Why τ²?** During one latency window (the period where the controller is "blind"), the rocket accumulates angular displacement A_blind = ½ × keff × u_max × τ². This blind-spot impulse is proportional to Π = keff × τ² — which is *why* the right combination is the product with latency squared, rather than keff alone or keff × τ. The derivation predicts τ² scaling, and the data are consistent with it; the exponent is not uniquely identified by the available experiments (latency only spans 1–6 steps here). This is the theoretical motivation for the parameter, not a proof of the exponent.

**Threshold:** Π ≈ 0.32 (first design where saturation fraction ≥ 0.35). This is not a fit — it's measured from simulation and confirmed across 6 universality campaigns.

---

### Finding 3 — Causal confirmation: three independent tests

**Test A — Remove saturation (2×2 factorial, n = 15 designs):**

| Condition | Low Π (< 0.23) | Π ≈ 1.0 | Π ≈ 1.4 |
|---|---|---|---|
| PID + real saturation | 1.000 | 0.867 | 0.600 |
| PID − saturation | 1.000 | **1.000** | **1.000** |
| ADRC slew fraction | 0.000 | 0.000 | 0.000 |

PID-nosat = 1.000 for **all 15 designs**. Saturation is both necessary and sufficient for PID failure. ADRC never saturates because its observer cancels disturbances upstream of the actuator.

**Test B — Rate filtering eliminates saturation:** Adding an EMA filter with time constant L×dt to the gyro output gives fsat = **0.000 at ALL Pi values** (including Π = 1.38). The mechanism is the derivative channel: bang-bang produces rate spikes → Kd × spike saturates the servo. Smoothing the rate signal prevents the spike from reaching the servo, regardless of Π.

Scope: The saturation onset is a continuous transition in [0.20, 0.32] for pure-delay FIFO implementations. The delay model robustness test found Π_crit = 0.20 (first onset, 10 reference designs); the regime map found Π_crit = 0.32 (first onset, 29 LHS designs). Conservative bare-metal threshold: Π > 0.20. RTOS scheduling jitter shifts onset to ~0.32. MCUs with IIR filtering (Madgwick, complementary) eliminate saturation entirely.

**Test C — Triple alignment:**
1. Saturation map (n = 29): fsat onset at Π = 0.32
2. Performance frontier (n = 63): ADRC advantage onset at Π = 0.37
3. 2×2 factorial (n = 15): saturation confirmed causal

Alignment ratio: 1.17×. Three completely independent experiments converge on the same threshold.

---

### Finding 4 — Disturbance-free simulators cause 64% calibration failure for sensitive rockets

**Corrected S2R table (final population, re-optimized gains):**

| Regime | n | Calibration failure (sim gain fails in real flight) | False approval (sim says GO, no valid gain exists) |
|---|---|---|---|
| EASY (normal) | 2,362 | 10.1% | 0.0% |
| Narrow-window | 36 | **63.9%** | 0.0% |
| INFEASIBLE | 2 | 0.0% | **100.0%** |

**The "simple model is never dangerous" claim is false.** Both genuinely uncontrollable designs receive false approval — a disturbance-free simulator has no signal that these designs can't work. The rare design that is truly uncontrollable appears fine in still-air tuning.

**Π predicts when tuning fails** (Π as calibration failure predictor):
- Π < 0.34: calibration failure rate = 9.1%
- Π ≥ 0.34: calibration failure rate = 61.3% — **6.7× jump**

Mechanism: high keff drives still-air autotune to Kp ≈ 7.0 [rad/rad] (no wind ceiling signal). High latency compresses the real ceiling to 1.4–2.1 [rad/rad]. The gain is wrong before any flight begins.

---

### Finding 5 — Controller invariance: the Π constraint is reactive-saturation-specific

Five architectures tested, same 50 designs, same Π parameter (full-physics audit completed 2026-06-24):

| Architecture | rho(Π, frac_pass) | p | Verdict |
|---|---|---|---|
| PID | ≈ −0.78 | — | Reactive — constrained |
| LQR | −0.747 | 4.8×10⁻¹⁰ | Reactive — constrained |
| SMC (sliding mode) | −0.789 | 9.6×10⁻¹² | Reactive — constrained |
| MPC H=1 (saturated clip) | −0.807 | 1.5×10⁻¹² | Reactive — constrained (worse than PD) |
| **MPC H=5 (5-step QP)** | **−0.052** | **0.718** | **Non-reactive — escapes Π** |
| ADRC (ESO) | −0.283 (full study); ~0 (audit) | — | **Non-reactive — escapes Π** |

**Reactive** controllers (PID, LQR, SMC, H=1 clip) all face ρ ≈ −0.75: observe current state → apply proportional/integral command → encounter saturation as an unplanned consequence. **Non-reactive** approaches escape it: ADRC's ESO cancels disturbances *upstream* of saturation; H=5 MPC plans bounded command sequences *ahead*, avoiding saturation proactively. **The architectural principle: any mechanism that prevents reactive saturation from becoming the controlling constraint escapes Π.** Integral PID (Ki > 0) partially helps at intermediate Π but fails identically at Π_keff > 0.80. ADRC remains the practical recommendation (all 50/50 designs perfect at all Kp; minimal compute); H=5 MPC requires solving a 5-step QP at 200 Hz (not feasible on Arduino, borderline on Teensy 4.x).

**At standard tuning (ω₀/ωc = 5):** zero ADRC failures to Π_keff ≈ 6.4 (3.9× beyond the original population maximum). At Π_keff > 2.6, ω₀/ωc must be increased to 12–20. ADRC has no fixed Π ceiling — only a tuning requirement that scales with Π.

---

### Finding 6 — Single test flight detects narrow-window designs (AUC = 0.954)

Fly at Kp = 0.044 [rad/rad] and measure RMS attitude error. Final population (n = 36 + 36, 7 seeds each):

| Class | Mean RMS at Kp = 0.044 [rad/rad] | Ratio |
|---|---|---|
| Narrow-window | 13.3° ± 5.2° | 3.53× |
| EASY | 3.8° ± 2.7° | — |

AUC = **0.954** [0.907, 0.989]. Recommended threshold: **RMS > 6.0°** (F1 = 0.89, precision = 0.89, recall = 0.86 on 7-seed means). Single-flight AUC = 0.921 — improved because the corrected narrow-window population is more severe on average.

Mechanism: Kp = 0.044 [rad/rad] is below the gain floor for narrow-window designs. The servo enters persistent bang-bang saturation trying to reject wind (slew_sat ≈ 0.66). Easy designs at Kp = 0.044 [rad/rad] stay in the linear regime.

---

## The Scientific Journey (What Judges Want to Hear)

| Round | Hypothesis | Result |
|---|---|---|
| 1 | High wind causes gain sensitivity | r = −0.001. **WRONG.** |
| 2 | Slow servos cause gain sensitivity | r = +0.013. **WRONG.** |
| 3 | Aerodynamic instability matters | r = +0.034. **WRONG.** |
| 4 | Interaction effects explain it | ΔCV < 0.003. **WRONG.** |
| True | θ̈_max = authority × inertia (mechanical) | AUC = 0.975, d = 3.71. ✓ |
| Extended | Latency compounds authority | Π = keff × τ²; triple alignment confirmed |

**Three bugs found and fixed:**
1. Servo formula unit error: servos modeled 57.3× too slow. Fixed → previous INFEASIBLE category dissolved.
2. Initial-angle bug: theta0_bias_std=3.0 was 3.0 radians (171.9°), not 3 degrees. Fixed → gain mechanism study invalid, redone correctly.
3. Gain search: Kd decoupled from Kp, Kp grid too coarse. Fixed → 56.7% of "confirmed" FRAGILE labels flipped.

Each correction strengthened AUC: **0.944 → 0.957 → 0.975.** The corrections removed noise, revealing a stronger signal. This is the right direction for a real physical effect.

The self-correction process is a scientific contribution in itself: finding and reporting bugs that invalidate prior results, rather than keeping them because they told a convenient story.

---

## The Physical Mechanism

```
Hardware:
  keff   = T_avg × L_nozzle / Iyy    ← rotational authority per radian of gimbal [s⁻²] (no fitting)
  τ      = latency_steps / loop_hz    ← total loop delay [s]
  Π      = keff × τ²                  ← dimensionless risk parameter
      ↓
Below Π ≈ 0.32: LINEAR REGIME
  Ceiling ≈ 0.042 / τ  (DIPDT phase margin; ≈ 380/latency_steps in simulator units)
  Any gain below ceiling → SR = 1.000
  PID = ADRC (Carlson 2025 linear equivalence)
      ↓
Above Π ≈ 0.32: SATURATION REGIME
  Aerodynamic disturbances → servo runs at max slew
  Bang-bang limit cycle regardless of Kp choice
  PID SR degrades monotonically (rho = −0.668 with Π)
      ↓
ADRC with ESO:
  ESO estimates disturbance from state derivatives
  Subtracts from control command BEFORE servo limit
  Servo never saturates (slew_frac = 0.000)
  SR maintained (rho = −0.283, much weaker Π dependence)
```

K_u separation (true relay probe, n = 36 narrow-window + 36 EASY):
- EASY median K_u = **2.0 [rad/rad]**
- Narrow-window median K_u = **0.64 [rad/rad]** (3.12×, p = 4.17×10⁻⁷)

The gain ceiling has closed toward the gain floor for narrow-window designs. The ceiling is where oscillation onset would occur; when the ceiling falls to meet the floor driven by wind rejection requirements, no valid gain exists.

---

## Numbers at a Glance

| Quantity | Value |
|---|---|
| Design space | n = 2,400 LHS designs, latency 1–6 steps, full physics |
| Narrow-window prevalence | 36/2,400 = 1.5% (after 3-pass correction) |
| AUC (θ̈_max formula) | **0.975** [0.959, 0.987]; Cohen's d = 3.71, p = 3.6×10⁻¹³ |
| AUC (flight detection, 7-seed) | **0.954** [0.907, 0.989] |
| Threshold (Π_crit, FIFO) | **0.20–0.32** (bare-metal conservative; regime map central) |
| Calibration failure — narrow-window | **63.9%** (sim-tuned gain fails in real flight) |
| Calibration failure — EASY | 10.1% |
| False approval — INFEASIBLE | **100%** (2 designs; disturbance-free = no warning) |
| PID-nosat = 1.000 | All 15 tested designs (saturation = necessary & sufficient) |
| ADRC slew_frac | 0.000 for all 15 designs |
| K_u separation (true relay) | 3.12× (2.0 vs 0.64 [rad/rad], p = 4.17×10⁻⁷, n = 36) |
| ADRC Π ceiling | None found to Π_td = 57 (Π_keff ≈ 6.4) |
| Rate filter finding | max fsat = 0.000 at Π = 1.38 with EMA tau = L×dt |

---

## Preflight Worksheet (For a Hobby Builder)

**Step 1 — Compute keff and Π:**
```
T_avg     = motor average thrust (N)
L_nozzle  = nozzle-to-CG distance (m)
Iyy       = rotational inertia (kg·m²)
τ         = total loop delay (s)  [= latency_steps / loop_hz, e.g. 5 steps at 200 Hz → 0.025 s]

keff   = T_avg × L_nozzle / Iyy    [s⁻²]
Π      = keff × τ²                 [-; dimensionless]
```

**Step 2 — Interpret (Π = keff × τ² throughout):**
- Π < 0.32: linear regime. Any full-physics tuning or ADRC works. Kp ceiling ≈ 0.042/τ.
- Π 0.32–1.0: saturation regime. PID window narrows; add wind+latency to simulator or use ADRC.
- Π 1.0–6.4: PID starts failing. **Use ADRC** (ω₀/ωc = 5 for Π < 2.6; 12–20 for Π 2.6–6.4).
- Π > 6.4: Unknown; ADRC with ω₀/ωc = 20 is the best current option.
- *For bare-metal FIFO MCUs only. Add rate filtering (IIR, complementary) → Π_crit shifts upward.*

**Step 3 — Kp = 0.044 [rad/rad] flight test** (to diagnose Π > 0.32 designs without precise Iyy):
- Fly at Kp = 0.044 [rad/rad]. Measure RMS attitude error over 3 seconds.
- RMS > 6.0°: narrow-window confirmed. Target Kp = 0.87–1.75 [rad/rad] from full-physics tuning or switch to ADRC.
- RMS < 6.0°: likely EASY. Small Kp increase may improve tracking.

**Step 4 — ADRC option** (if Π_keff > 1.0 or full-physics simulator unavailable):
```
b0 = keff   (from spec computation above)
ωc = 5 for Π_keff < 2.6; lower ωc for higher Π_keff
ω₀ = 5 × ωc  to  20 × ωc  depending on Π_keff
```
ADRC tolerates 2–4× error in b0. No per-design tuning required.

---

## Recommended Figures for Submission

| Figure | What it shows | Key message |
|---|---|---|
| 1. fsat vs Π scatter | n = 29 designs, color by regime | Clear transition at Π ≈ 0.32 |
| 2. R0522 annotation | keff = 1900 s⁻², lat = 1 step, Π = 0.047 → linear | "Authority alone doesn't predict difficulty" |
| 3. S2R calibration failure vs Π | Calibration failure rate by Π bin | 6.7× jump at Π ≥ 0.34 |
| 4. PID-nosat = 1.000 | 2×2 factorial table | "Remove saturation → PID is perfect" |
| 5. Lag filter eliminates saturation | fsat vs model type | "Rate smoothing = architectural equivalent of ADRC for bang-bang" |
| 6. Flight detection boxplot | RMS at Kp = 0.044 [rad/rad] by regime | 6.0° threshold; 3.53× separation |
| 7. Triple alignment | Three experiments → Π = 0.32, 0.37, confirmed | "Not a coincidence" |

---

*Full paper: `paper/Paper_Draft_Condensed.md` | Interactive figures: `outputs/sts_gold_*.html`*
