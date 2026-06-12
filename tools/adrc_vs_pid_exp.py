"""
adrc_vs_pid_exp.py

Research question
─────────────────
Is the FRAGILE regime a controller architecture limitation (fixable with
ADRC's Extended State Observer) or a hard actuator-bandwidth wall that no
controller architecture can escape?

Experiment
──────────
  Population: 25 FRAGILE + 25 EASY designs from the Q-A gain-window sweep.
  Fidelity:   Full physics (wind + slew + noise + latency + nonlinear aero + ...).
  Seeds:      7 per condition (same as FINDING 5 / flight signature study).

  Conditions per design:
    1. PID  @ Kp=2,  Kd=1  (simple-model default — FINDING 5 baseline)
    2. PID  @ best_Kp from Exp1, Kd from Exp1  (optimal PID ceiling)
    3. ADRC @ omega_c × omega0 sweep, b0 = keff_full  (oracle b0)
    4. ADRC @ best (omega_c, omega0), b0 = keff_simple  (biased b0 — what a
       builder using the design-space model would compute)

  omega_c sweep: [5, 10, 20, 30, 50] rad/s
  omega0 = omega_c × ratio, ratio in [3, 5, 8]  → 15 ADRC combos
  Plus b0 sensitivity: best params × [keff_full, keff_simple, keff_full/2, keff_full*2]

Key output
──────────
  Result 1 (ADRC fixes FRAGILE): SR(ADRC_best) >> SR(PID_best) for FRAGILE
    → FRAGILE is an architecture limitation; ESO can cancel wind without bang-bang
  Result 2 (ADRC hits the wall): SR(ADRC_best) ≈ SR(PID_best), both capped
    → Slew rate is a controller-independent physical limit
  Mixed: ADRC helps some FRAGILE but not all; wall exists but is shifted
    → Threshold formula separates "ADRC-fixable" from "physics-limited"
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))

import numpy as np
import pandas as pd

from design_space import build_plant, build_actuator, build_sensor, build_disturbance
from simulator import simulate, ScenarioParams
from controller import PIDParams, ADRCParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

# ── Constants ─────────────────────────────────────────────────────────────────

CU_TO_RAD = np.pi / 180.0 * REF_MAX_GIMBAL_DEG / REF_U_MAX  # 0.02182 rad/CU (constant)
L_NOZZLE  = 0.25          # m, fixed for all designs

FIDELITY = FidelityConfig.from_flags(
    wind=True, slew=True, latency=True, sensor_noise=True,
    nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    backlash=False, deadband=False, thrust_var=False,
)

N_SEEDS    = 7
KD_PID_REF = 1            # Kd for PID@Kp=2 baseline (same as FINDING 5)
T_END      = 3.0

# ADRC sweep grid — omega0 capped at 25 rad/s to avoid ESO instability with latency
# Stability constraint: omega0 × (latency_steps × dt) < ~0.3 for reliable ESO
# Typical latency = 3 × 0.005s = 0.015s → omega0 < 20 comfortable, 25 borderline
OMEGA_C_GRID   = [5.0, 8.0, 10.0, 12.0]       # closed-loop bandwidth (rad/s)
OMEGA0_MULT    = [2.0, 2.5]                     # omega0 = mult × omega_c (capped at 25)
OMEGA0_CAP     = 25.0                           # hard cap to prevent ESO divergence
# b0 sensitivity labels (computed per design below)
B0_VARIANTS = ["oracle", "simple", "half_oracle", "double_oracle"]


def keff_full(design: dict) -> float:
    """keff_full = T_avg × motor_scale × CU_TO_RAD × l_nozzle / Iyy  [rad/s²/CU]"""
    return (F15_AVG_THRUST_N * design["motor_scale"] * CU_TO_RAD * L_NOZZLE
            / design["Iyy"])


def eval_pid(design: dict, Kp: float, Kd: float, seeds: list[int]) -> dict:
    plant = build_plant(design)
    act   = build_actuator(design)
    sen   = build_sensor(design)
    dis   = build_disturbance(design)
    sc    = ScenarioParams(t_end=T_END)
    pid   = PIDParams(Kp=Kp, Kd=Kd, Ki=0.0, u_max=act.u_max)
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    results = [simulate(pid, plant, act2, sen2, dis2, sc2, seed=s) for s in seeds]
    return {
        "sr":        np.mean([r.success         for r in results]),
        "rms":       np.mean([r.rms_error_deg   for r in results]),
        "slew_sat":  np.mean([r.slew_sat_frac   for r in results]),
        "peak":      np.mean([r.peak_error_deg  for r in results]),
    }


def eval_adrc(design: dict, omega_c: float, omega0: float, b0: float,
              seeds: list[int]) -> dict:
    plant  = build_plant(design)
    act    = build_actuator(design)
    sen    = build_sensor(design)
    dis    = build_disturbance(design)
    sc     = ScenarioParams(t_end=T_END)
    adrc_p = ADRCParams(omega_c=omega_c, omega0=omega0, b0=b0, u_max=act.u_max)
    pid_dummy = PIDParams(Kp=2.0, Kd=1.0, Ki=0.0, u_max=act.u_max)  # not used
    act2, sen2, dis2, sc2 = apply_fidelity_config(act, sen, dis, sc, FIDELITY)
    results = [simulate(pid_dummy, plant, act2, sen2, dis2, sc2, seed=s, adrc=adrc_p)
               for s in seeds]
    return {
        "sr":        np.mean([r.success         for r in results]),
        "rms":       np.mean([r.rms_error_deg   for r in results]),
        "slew_sat":  np.mean([r.slew_sat_frac   for r in results]),
        "peak":      np.mean([r.peak_error_deg  for r in results]),
    }


# ── Load designs ──────────────────────────────────────────────────────────────

print("Loading designs...")
qa   = pd.read_csv(os.path.join(os.path.dirname(__file__), "..",
                   "experiments", "results", "gain_window_sweep_py.csv"))
exp1 = pd.read_csv(os.path.join(os.path.dirname(__file__), "..",
                   "experiments", "results", "exp1_regime_index_py.csv"))

qa_designs = (qa[["rocket_id", "regime_label"]]
              .drop_duplicates()
              .rename(columns={"regime_label": "qa_regime"}))
merged = qa_designs.merge(
    exp1.drop(columns=["regime_label"], errors="ignore"),
    on="rocket_id"
)

fragile_ids = merged[merged["qa_regime"] == "FRAGILE"]["rocket_id"].unique()
easy_ids    = merged[merged["qa_regime"] == "EASY"]["rocket_id"].unique()
print(f"  FRAGILE: {len(fragile_ids)}  EASY: {len(easy_ids)}")

seeds = list(range(1, N_SEEDS + 1))

# ── Main experiment loop ───────────────────────────────────────────────────────

rows_sweep   = []   # full ADRC sweep results
rows_summary = []   # per-design summary

total = len(merged["rocket_id"].unique())
done  = 0

for _, row in merged.drop_duplicates("rocket_id").iterrows():
    done += 1
    rid    = row["rocket_id"]
    regime = row["qa_regime"]
    design = row.to_dict()

    kf     = keff_full(design)
    ks     = float(design["control_effectiveness"])   # keff_simple
    best_Kp = float(exp1[exp1["rocket_id"] == rid]["best_Kp"].values[0])
    best_Kd = float(exp1[exp1["rocket_id"] == rid]["best_Kd"].values[0])

    # ── PID baselines ─────────────────────────────────────────────────────────
    pid2   = eval_pid(design, Kp=2.0,    Kd=KD_PID_REF, seeds=seeds)
    pid_best = eval_pid(design, Kp=best_Kp, Kd=best_Kd,    seeds=seeds)

    # ── ADRC oracle sweep ─────────────────────────────────────────────────────
    best_adrc_sr  = 0.0
    best_omega_c  = None
    best_omega0   = None

    for wc in OMEGA_C_GRID:
        for mult in OMEGA0_MULT:
            w0 = min(wc * mult, OMEGA0_CAP)
            res = eval_adrc(design, omega_c=wc, omega0=w0, b0=kf, seeds=seeds)
            rows_sweep.append({
                "rocket_id": rid, "regime": regime,
                "keff_full": kf, "keff_simple": ks,
                "omega_c": wc, "omega0": w0, "b0_type": "oracle", "b0": kf,
                **res,
            })
            if res["sr"] > best_adrc_sr:
                best_adrc_sr = res["sr"]
                best_omega_c = wc
                best_omega0  = w0

    # ── b0 sensitivity at best params ─────────────────────────────────────────
    if best_omega_c is not None:
        for b0_label, b0_val in [
            ("simple",        ks),
            ("half_oracle",   kf * 0.5),
            ("double_oracle", kf * 2.0),
        ]:
            res_b0 = eval_adrc(design, omega_c=best_omega_c, omega0=best_omega0,
                                b0=b0_val, seeds=seeds)
            rows_sweep.append({
                "rocket_id": rid, "regime": regime,
                "keff_full": kf, "keff_simple": ks,
                "omega_c": best_omega_c, "omega0": best_omega0,
                "b0_type": b0_label, "b0": b0_val,
                **res_b0,
            })

    rows_summary.append({
        "rocket_id": rid, "regime": regime,
        "keff_full": kf, "keff_simple": ks,
        "authority_ratio": float(design.get("authority_inertia_ratio", 0)),
        "Iyy": float(design["Iyy"]),
        "servo_slew_deg_s": float(design["servo_slew_deg_s"]),
        "wind_strength": float(design["wind_strength"]),
        # PID results
        "sr_pid2":    pid2["sr"],
        "rms_pid2":   pid2["rms"],
        "slew_pid2":  pid2["slew_sat"],
        "sr_pidbest": pid_best["sr"],
        "rms_pidbest":pid_best["rms"],
        "slew_pidbest":pid_best["slew_sat"],
        "best_Kp":    best_Kp,
        "best_Kd":    best_Kd,
        # Best ADRC oracle results
        "sr_adrc":    best_adrc_sr,
        "best_omega_c": best_omega_c,
        "best_omega0":  best_omega0,
        "sr_improvement_vs_pid2":    best_adrc_sr - pid2["sr"],
        "sr_improvement_vs_pidbest": best_adrc_sr - pid_best["sr"],
    })

    print(f"[{done:3d}/{total}] {rid} {regime:8s}  "
          f"PID@2={pid2['sr']:.2f}  PID@best={pid_best['sr']:.2f}  "
          f"ADRC_best={best_adrc_sr:.2f}  "
          f"(wc={best_omega_c}, w0={best_omega0})  "
          f"kf={kf:.2f}  ks={ks:.2f}")

# ── Save results ──────────────────────────────────────────────────────────────

out_dir = os.path.join(os.path.dirname(__file__), "..", "experiments", "results")
df_sweep   = pd.DataFrame(rows_sweep)
df_summary = pd.DataFrame(rows_summary)

df_sweep.to_csv(os.path.join(out_dir, "adrc_sweep_py.csv"),   index=False)
df_summary.to_csv(os.path.join(out_dir, "adrc_summary_py.csv"), index=False)
print(f"\nSaved sweep ({len(df_sweep)} rows) and summary ({len(df_summary)} rows).")

# ── Analysis ──────────────────────────────────────────────────────────────────

THRESHOLD = 0.80   # SR threshold: "fixed" means ADRC achieves >= THRESHOLD

print("\n" + "="*65)
print("ADRC vs PID COMPARISON")
print("="*65)

for regime in ["FRAGILE", "EASY"]:
    sub = df_summary[df_summary["regime"] == regime]
    n   = len(sub)
    if n == 0:
        continue

    mean_pid2    = sub["sr_pid2"].mean()
    mean_pidbest = sub["sr_pidbest"].mean()
    mean_adrc    = sub["sr_adrc"].mean()

    # "Fixed" = ADRC achieves SR >= threshold where PID@best did NOT
    fragile_failed_by_pid  = sub[sub["sr_pidbest"] < THRESHOLD]
    fixed_by_adrc = (fragile_failed_by_pid["sr_adrc"] >= THRESHOLD).sum()

    # Slew saturation comparison
    mean_slew_pid2    = sub["slew_pid2"].mean()
    mean_slew_pidbest = sub["slew_pidbest"].mean()

    print(f"\n{regime} (n={n})")
    print(f"  SR: PID@Kp=2 = {mean_pid2:.3f}  |  PID@best = {mean_pidbest:.3f}  |  ADRC_best = {mean_adrc:.3f}")
    print(f"  ADRC improvement vs PID@best: {(mean_adrc - mean_pidbest):+.3f}")
    if len(fragile_failed_by_pid) > 0:
        print(f"  Designs failing PID@best (SR<{THRESHOLD}): {len(fragile_failed_by_pid)}")
        print(f"  Of those, fixed by ADRC (SR>={THRESHOLD}): {fixed_by_adrc} / {len(fragile_failed_by_pid)}")
    print(f"  Slew sat: PID@2 = {mean_slew_pid2:.3f}  |  PID@best = {mean_slew_pidbest:.3f}")

print("\n--- Per-design FRAGILE breakdown ---")
frag = df_summary[df_summary["regime"] == "FRAGILE"].sort_values("sr_adrc", ascending=False)
print(f"{'ID':8s}  {'PID@2':6s}  {'PID@best':8s}  {'ADRC':6s}  {'wc':4s}  {'w0':5s}  {'kf':5s}  {'slew_pid2':9s}  {'verdict':12s}")
for _, r in frag.iterrows():
    adrc_fix = "FIXED" if r["sr_adrc"] >= THRESHOLD else ("PARTIAL" if r["sr_adrc"] > r["sr_pid2"] + 0.15 else "NO HELP")
    print(f"{r['rocket_id']:8s}  {r['sr_pid2']:.2f}    {r['sr_pidbest']:.3f}      {r['sr_adrc']:.2f}    "
          f"{int(r['best_omega_c']) if r['best_omega_c'] else 0:4d}  "
          f"{int(r['best_omega0']) if r['best_omega0'] else 0:5d}  "
          f"{r['keff_full']:5.1f}  {r['slew_pid2']:.3f}      {adrc_fix}")

# Correlations: does keff_full or authority_ratio predict whether ADRC helps?
from scipy.stats import pearsonr
frag_df = df_summary[df_summary["regime"] == "FRAGILE"]
if len(frag_df) >= 5:
    print("\n--- What predicts ADRC improvement? (FRAGILE only) ---")
    adrc_gain = frag_df["sr_adrc"] - frag_df["sr_pidbest"]
    for col in ["keff_full", "authority_ratio", "Iyy", "servo_slew_deg_s", "wind_strength"]:
        if col in frag_df.columns and frag_df[col].std() > 0:
            r_val, p_val = pearsonr(frag_df[col], adrc_gain)
            print(f"  r({col}, ADRC_gain) = {r_val:+.3f}  p={p_val:.3f}")

# Best omega_c distribution
print("\n--- Best omega_c by regime ---")
for regime in ["FRAGILE", "EASY"]:
    sub = df_summary[df_summary["regime"] == regime]
    if len(sub) > 0 and sub["best_omega_c"].notna().any():
        counts = sub["best_omega_c"].value_counts().sort_index()
        print(f"  {regime}: {dict(counts)}")

print("\nDone.")
