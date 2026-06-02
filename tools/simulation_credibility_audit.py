from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.inspection import permutation_importance
from sklearn.metrics import balanced_accuracy_score


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "outputs" / "sim_credibility"
EXP_RESULTS = ROOT / "experiments" / "results"


@dataclass
class Thresholds:
    easy_success: float = 0.80
    easy_robustness: float = 1.00
    easy_u_sat_frac: float = 0.60
    easy_slew_sat_frac: float = 1.00
    easy_rms_error_deg: float = 8.0
    easy_settling_time_s: float = 3.0
    easy_oscillation_score: float = 3.0
    fragile_success: float = 0.35
    fragile_rms_error_deg: float = 16.0
    infeasible_robustness: float = 0.0


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def _first_present(row: pd.Series, candidates: List[str], default: float = np.nan) -> float:
    for c in candidates:
        if c in row.index and pd.notna(row[c]):
            return float(row[c])
    return float(default)


def _classify_row(row: pd.Series, q: Thresholds) -> str:
    nominal_success = _first_present(row, ["nominal_success_rate"], default=0.0)
    robustness = _first_present(row, ["robustness"], default=0.0)

    # Exp1 regime index may not carry actuator/error diagnostics. If absent, do not force-fail easy/fragile checks.
    u_sat = _first_present(row, ["u_cmd_sat_frac", "u_sat_frac"], default=np.nan)
    slew_sat = _first_present(row, ["slew_sat_frac"], default=np.nan)
    rms_err = _first_present(row, ["rms_error_deg", "base_rms_deg"], default=np.nan)
    settling = _first_present(row, ["settling_time_s"], default=np.nan)
    osc = _first_present(row, ["oscillation_score"], default=np.nan)

    easy_quality = nominal_success >= q.easy_success
    if pd.notna(u_sat):
        easy_quality = easy_quality and (u_sat <= q.easy_u_sat_frac)
    if pd.notna(slew_sat):
        easy_quality = easy_quality and (slew_sat <= q.easy_slew_sat_frac)
    if pd.notna(rms_err):
        easy_quality = easy_quality and (rms_err <= q.easy_rms_error_deg)
    if pd.notna(settling):
        easy_quality = easy_quality and (settling <= q.easy_settling_time_s)
    if pd.notna(osc):
        easy_quality = easy_quality and (osc <= q.easy_oscillation_score)

    fragile_quality = nominal_success >= q.fragile_success
    if pd.notna(rms_err):
        fragile_quality = fragile_quality and (rms_err <= q.fragile_rms_error_deg)

    if robustness >= q.easy_robustness and easy_quality:
        return "EASY"
    if robustness > q.infeasible_robustness and fragile_quality:
        return "FRAGILE"
    return "INFEASIBLE"


def _prep_exp1() -> pd.DataFrame:
    backup = OUT_DIR / "exp1_regime_index_original_backup.csv"
    path = backup if backup.exists() else (EXP_RESULTS / "exp1_regime_index.csv")
    return pd.read_csv(path)


def _rf_bacc_and_rank(df: pd.DataFrame, y: pd.Series, feature_cols: List[str]) -> Dict[str, object]:
    x = df[feature_cols].apply(pd.to_numeric, errors="coerce")
    x = x.fillna(x.median(numeric_only=True))

    idx = np.arange(len(x))
    rng = np.random.default_rng(123)
    rng.shuffle(idx)
    split = int(0.7 * len(idx))
    tr, te = idx[:split], idx[split:]

    clf = RandomForestClassifier(n_estimators=300, random_state=123, n_jobs=1)
    clf.fit(x.iloc[tr], y.iloc[tr])
    pred = clf.predict(x.iloc[te])
    bacc = balanced_accuracy_score(y.iloc[te], pred)

    perm = permutation_importance(
        clf,
        x.iloc[te],
        y.iloc[te],
        n_repeats=10,
        random_state=123,
        scoring="balanced_accuracy",
    )
    imp = pd.Series(perm.importances_mean, index=x.columns).sort_values(ascending=False)

    top = imp.index.tolist()
    backlash_rank = int(np.where(imp.index == "backlash")[0][0] + 1) if "backlash" in imp.index else np.nan
    return {
        "balanced_accuracy": float(bacc),
        "backlash_permutation_rank": backlash_rank,
        "top_feature_1": top[0] if len(top) > 0 else "",
        "top_feature_2": top[1] if len(top) > 1 else "",
        "top_feature_3": top[2] if len(top) > 2 else "",
    }


def generate_parameter_audit() -> pd.DataFrame:
    exp1_src = _read(ROOT / "experiments" / "framework" / "experiments" / "run_exp1_stability_authority_frontier.m")
    cfg_src = _read(ROOT / "experiments" / "framework" / "plant" / "build_realistic_cfg.m")
    clf_src = _read(ROOT / "experiments" / "framework" / "analysis" / "classify_regime.m")

    params = [
        "mass",
        "Iyy",
        "static_margin",
        "Cm_alpha",
        "control_effectiveness",
        "thrust",
        "servo_slew_deg_s",
        "max_gimbal_deg",
        "best_u_max_frac",
        "deadband",
        "backlash",
        "latency_steps",
        "wind_strength",
        "p_unstable",
    ]

    evidence = {
        "mass": "sampled/recorded but not directly consumed in simulate_case_realistic dynamics",
        "Iyy": "sampled/recorded but not directly consumed in simulate_case_realistic dynamics",
        "static_margin": "sampled; used only to derive p_unstable if override absent",
        "Cm_alpha": "sampled; used only to derive p_unstable if override absent",
        "control_effectiveness": "wired to cfg.plant.control_eff and enters qdot",
        "thrust": "sampled; used in derived p_unstable calculation in Exp1 sampler",
        "servo_slew_deg_s": "wired to actuator slew_max",
        "max_gimbal_deg": "used in unit/code scaling path",
        "best_u_max_frac": "wired to cfg.plant.u_max",
        "deadband": "wired to realism.servo_deadband",
        "backlash": "wired to realism.servo_backlash",
        "latency_steps": "wired to realism.sensor_latency_steps",
        "wind_strength": "wired to realism.gust_std",
        "p_unstable": "wired to destabilizing p^2*theta term",
    }

    rows = []
    for p in params:
        sampled = p in exp1_src or p == "best_u_max_frac"
        cfg_wired = p in cfg_src or p.replace("_steps", "") in cfg_src or p.replace("_deg_s", "") in cfg_src
        enters_dyn = p in {"control_effectiveness", "p_unstable"}
        enters_sensor_or_actuator = p in {
            "servo_slew_deg_s",
            "max_gimbal_deg",
            "best_u_max_frac",
            "deadband",
            "backlash",
            "latency_steps",
            "wind_strength",
        }
        rows.append(
            {
                "parameter": p,
                "sampled_in_exp1": sampled,
                "wired_in_cfg_builder": cfg_wired,
                "enters_runtime_dynamics_directly": enters_dyn,
                "enters_sensor_or_actuator_model": enters_sensor_or_actuator,
                "directly_used_in_classifier": p in clf_src,
                "audit_flag": "UNWIRED_OR_PROXY" if sampled and not (enters_dyn or enters_sensor_or_actuator) else "WIRED",
                "evidence": evidence[p],
            }
        )

    out = pd.DataFrame(rows)
    out.to_csv(OUT_DIR / "parameter_audit.csv", index=False)
    return out


def generate_backlash_artifact_audit(exp1_df: pd.DataFrame) -> pd.DataFrame:
    y = exp1_df["regime_label"].astype(str)
    features = [
        "p_unstable",
        "servo_slew_deg_s",
        "max_gimbal_deg",
        "best_u_max_frac",
        "mass",
        "Iyy",
        "static_margin",
        "Cm_alpha",
        "control_effectiveness",
        "thrust",
        "deadband",
        "backlash",
        "latency_steps",
        "wind_strength",
    ]
    features = [f for f in features if f in exp1_df.columns]

    rows: List[Dict[str, object]] = []
    base = _rf_bacc_and_rank(exp1_df, y, features)
    rows.append({"test": "baseline", **base})

    if "backlash" in features:
        x_drop = [f for f in features if f != "backlash"]
        rows.append({"test": "remove_backlash", **_rf_bacc_and_rank(exp1_df, y, x_drop)})

        rng = np.random.default_rng(7)
        lo = float(exp1_df["backlash"].min())
        hi = float(exp1_df["backlash"].max())

        rnd = exp1_df.copy()
        rnd["backlash"] = rng.uniform(lo, hi, size=len(rnd))
        rows.append({"test": "randomize_backlash", **_rf_bacc_and_rank(rnd, y, features)})

        hold = exp1_df.copy()
        hold["backlash"] = float(hold["backlash"].median())
        rows.append({"test": "hold_backlash_constant", **_rf_bacc_and_rank(hold, y, features)})

        shf = exp1_df.copy()
        shf["backlash"] = rng.permutation(shf["backlash"].to_numpy())
        rows.append({"test": "shuffle_backlash", **_rf_bacc_and_rank(shf, y, features)})

    out = pd.DataFrame(rows)
    base_bacc = float(out.loc[out["test"] == "baseline", "balanced_accuracy"].iloc[0])
    out["delta_bacc_vs_baseline"] = out["balanced_accuracy"] - base_bacc
    out.to_csv(OUT_DIR / "backlash_artifact_audit.csv", index=False)
    return out


def generate_regime_threshold_sensitivity(exp1_df: pd.DataFrame) -> pd.DataFrame:
    base = Thresholds()
    base_labels = exp1_df["regime_label"].astype(str).str.upper()

    rows: List[Dict[str, object]] = []
    for pct in [0.05, 0.10, 0.20]:
        for sign in [-1, 1]:
            q = Thresholds(**base.__dict__.copy())
            fac = 1.0 + sign * pct
            q.easy_success *= fac
            q.fragile_success *= fac
            q.easy_u_sat_frac *= (1.0 - sign * pct)
            q.easy_slew_sat_frac *= (1.0 - sign * pct)
            q.easy_rms_error_deg *= (1.0 - sign * pct)
            q.easy_settling_time_s *= (1.0 - sign * pct)
            q.easy_oscillation_score *= (1.0 - sign * pct)
            q.fragile_rms_error_deg *= (1.0 - sign * pct)

            perturbed = exp1_df.apply(lambda r: _classify_row(r, q), axis=1)
            rows.append(
                {
                    "perturbation": f"{int(sign * pct * 100)}pct",
                    "flip_rate": float((perturbed != base_labels).mean()),
                    "easy_to_other": int(((base_labels == "EASY") & (perturbed != "EASY")).sum()),
                    "fragile_to_other": int(((base_labels == "FRAGILE") & (perturbed != "FRAGILE")).sum()),
                    "infeasible_to_other": int(((base_labels == "INFEASIBLE") & (perturbed != "INFEASIBLE")).sum()),
                    "easy_count": int((perturbed == "EASY").sum()),
                    "fragile_count": int((perturbed == "FRAGILE").sum()),
                    "infeasible_count": int((perturbed == "INFEASIBLE").sum()),
                }
            )

    out = pd.DataFrame(rows)
    out.to_csv(OUT_DIR / "regime_threshold_sensitivity.csv", index=False)
    return out


def generate_repeatability_summary() -> pd.DataFrame:
    seed_files = sorted(OUT_DIR.glob("exp1_seed_*.csv"))
    rows: List[Dict[str, object]] = []
    if not seed_files:
        out = pd.DataFrame([{"note": "No exp1_seed_*.csv files found."}])
        out.to_csv(OUT_DIR / "repeatability_summary.csv", index=False)
        return out

    dist_map: Dict[str, np.ndarray] = {}
    rank_map: Dict[str, pd.Series] = {}
    feature_cols = [
        "p_unstable",
        "servo_slew_deg_s",
        "max_gimbal_deg",
        "best_u_max_frac",
        "mass",
        "Iyy",
        "static_margin",
        "Cm_alpha",
        "control_effectiveness",
        "thrust",
        "deadband",
        "backlash",
        "latency_steps",
        "wind_strength",
    ]

    for f in seed_files:
        df = pd.read_csv(f)
        key = f.stem
        c = df["regime_label"].astype(str).str.upper().value_counts(normalize=True)
        vec = np.array([c.get("EASY", 0.0), c.get("FRAGILE", 0.0), c.get("INFEASIBLE", 0.0)])
        dist_map[key] = vec

        x = df[[c for c in feature_cols if c in df.columns]].apply(pd.to_numeric, errors="coerce").fillna(0.0)
        y = df["regime_label"].astype(str)
        clf = RandomForestClassifier(n_estimators=200, random_state=42, n_jobs=1)
        clf.fit(x, y)
        rank_map[key] = pd.Series(clf.feature_importances_, index=x.columns)

        rows.append(
            {
                "seed_file": key,
                "n_rows": int(len(df)),
                "easy_frac": float(vec[0]),
                "fragile_frac": float(vec[1]),
                "infeasible_frac": float(vec[2]),
                "boundary_abs_mean": float(np.abs(df["boundary_distance"]).mean()),
                "boundary_abs_p90": float(np.abs(df["boundary_distance"]).quantile(0.90)),
            }
        )

    keys = list(dist_map.keys())
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            a, b = keys[i], keys[j]
            p = np.clip(dist_map[a], 1e-9, 1.0)
            q = np.clip(dist_map[b], 1e-9, 1.0)
            m = 0.5 * (p + q)
            jsd = 0.5 * np.sum(p * np.log(p / m)) + 0.5 * np.sum(q * np.log(q / m))
            rho = rank_map[a].rank(ascending=False).corr(rank_map[b].rank(ascending=False), method="spearman")
            rows.append(
                {
                    "seed_file": f"pair:{a}__{b}",
                    "regime_distribution_jsd": float(jsd),
                    "feature_rank_spearman": float(rho) if pd.notna(rho) else np.nan,
                }
            )

    out = pd.DataFrame(rows)
    out.to_csv(OUT_DIR / "repeatability_summary.csv", index=False)
    return out


def generate_exp4_decision_validity() -> pd.DataFrame:
    s = pd.read_csv(EXP_RESULTS / "exp4_first_correct_fidelity.csv")
    t = pd.read_csv(EXP_RESULTS / "exp4_fidelity_decision_trajectories.csv")

    s["disagreement"] = s["first_correct_fidelity_level"] > 1
    rows: List[Dict[str, object]] = []
    rows.append({"test": "overall_disagreement_rate", "value": float(s["disagreement"].mean()), "n": int(len(s))})

    for reg, g in s.groupby("regime_label"):
        rows.append({"test": "disagreement_rate_by_regime", "group": reg, "value": float(g["disagreement"].mean()), "n": int(len(g))})

    agree = s.loc[~s["disagreement"], "boundary_distance"].abs()
    dis = s.loc[s["disagreement"], "boundary_distance"].abs()
    rows.append({"test": "boundary_abs_distance_shift", "group": "disagree_minus_agree", "value": float(dis.mean() - agree.mean()) if len(dis) and len(agree) else np.nan, "n": int(len(s))})

    lvl = t.groupby("fidelity_level").apply(lambda g: (g["decision"] != g["reference_decision"]).mean()).reset_index(name="mismatch_rate")
    for _, r in lvl.iterrows():
        rows.append({"test": "mismatch_rate_by_fidelity_level", "group": str(r["fidelity_level"]), "value": float(r["mismatch_rate"]), "n": int((t["fidelity_level"] == r["fidelity_level"]).sum())})

    out = pd.DataFrame(rows)
    out.to_csv(OUT_DIR / "exp4_decision_validity.csv", index=False)
    return out


def generate_lever_stability_audit() -> pd.DataFrame:
    cells = pd.read_csv(EXP_RESULTS / "exp5_design_lever_cells.csv")
    rows: List[Dict[str, object]] = []

    reg_base = cells.groupby(["regime_label", "design_lever"])["tradeoff_score"].mean().reset_index()
    for reg in reg_base["regime_label"].unique():
        sub = reg_base[reg_base["regime_label"] == reg].sort_values("tradeoff_score", ascending=False)
        top = sub.iloc[0]
        rows.append({"audit": "base_mean_top_lever", "regime_label": reg, "design_lever": top["design_lever"], "score": float(top["tradeoff_score"])})

    rng = np.random.default_rng(123)
    B = 120
    for reg in cells["regime_label"].unique():
        sub = cells[cells["regime_label"] == reg]
        ids = sub["rocket_id"].unique()
        wins: Dict[str, int] = {}
        for _ in range(B):
            sampled = rng.choice(ids, size=len(ids), replace=True)
            boot = sub[sub["rocket_id"].isin(sampled)]
            win = boot.groupby("design_lever")["tradeoff_score"].mean().sort_values(ascending=False).index[0]
            wins[win] = wins.get(win, 0) + 1
        for lever, c in wins.items():
            rows.append({"audit": "bootstrap_win_rate", "regime_label": reg, "design_lever": lever, "score": float(c / B)})

    q_lo = cells["tradeoff_score"].quantile(0.10)
    q_hi = cells["tradeoff_score"].quantile(0.90)
    tr = cells[(cells["tradeoff_score"] >= q_lo) & (cells["tradeoff_score"] <= q_hi)]
    trm = tr.groupby(["regime_label", "design_lever"])["tradeoff_score"].mean().reset_index()
    for reg in trm["regime_label"].unique():
        sub = trm[trm["regime_label"] == reg].sort_values("tradeoff_score", ascending=False)
        rows.append({"audit": "trimmed_top_lever", "regime_label": reg, "design_lever": sub.iloc[0]["design_lever"], "score": float(sub.iloc[0]["tradeoff_score"])})

    for reg in cells["regime_label"].unique():
        sub = cells[cells["regime_label"] == reg].copy()
        base_win = sub.groupby("design_lever")["tradeoff_score"].mean().sort_values(ascending=False).index[0]
        sigma = max(1e-6, float(sub["tradeoff_score"].std())) * 0.15
        keep = 0
        runs = 80
        for _ in range(runs):
            noisy = sub.copy()
            noisy["tradeoff_score"] = noisy["tradeoff_score"] + rng.normal(0.0, sigma, size=len(noisy))
            win = noisy.groupby("design_lever")["tradeoff_score"].mean().sort_values(ascending=False).index[0]
            keep += int(win == base_win)
        rows.append({"audit": "perturbation_winner_retention", "regime_label": reg, "design_lever": base_win, "score": float(keep / runs)})

    out = pd.DataFrame(rows)
    out.to_csv(OUT_DIR / "lever_stability_audit.csv", index=False)
    return out


def write_report(parameter_df: pd.DataFrame, backlash_df: pd.DataFrame, threshold_df: pd.DataFrame, repeat_df: pd.DataFrame, exp4_df: pd.DataFrame, lever_df: pd.DataFrame) -> None:
    unconnected = parameter_df[parameter_df["audit_flag"] == "UNWIRED_OR_PROXY"]["parameter"].tolist()
    base = backlash_df.loc[backlash_df["test"] == "baseline"].iloc[0]
    worst = backlash_df.sort_values("delta_bacc_vs_baseline").iloc[0]
    max_flip = threshold_df.loc[threshold_df["flip_rate"].idxmax()]

    pairs = repeat_df[repeat_df["seed_file"].astype(str).str.startswith("pair:")]
    jsd = float(pairs["regime_distribution_jsd"].mean()) if "regime_distribution_jsd" in pairs else np.nan
    rho = float(pairs["feature_rank_spearman"].mean()) if "feature_rank_spearman" in pairs else np.nan

    exp4_dis = exp4_df[exp4_df["test"] == "overall_disagreement_rate"]["value"].iloc[0]
    lever_ret = float(lever_df[lever_df["audit"] == "perturbation_winner_retention"]["score"].mean())

    text = [
        "# SIMULATION_CREDIBILITY_REPORT",
        "",
        "## Adversarial Verdict",
        "- Status: CONDITIONAL CREDIBILITY",
        "- Interpretation: core trends are present, but multiple claims are vulnerable to artifacts.",
        "",
        "## Key Risks",
        f"- Sampled but not runtime-wired parameters: {', '.join(unconnected) if unconnected else 'none'}.",
        f"- Backlash baseline balanced accuracy: {base['balanced_accuracy']:.4f}; worst perturbation: {worst['test']} ({worst['delta_bacc_vs_baseline']:.4f}).",
        f"- Max regime flip rate under threshold perturbation: {max_flip['flip_rate']:.3f} at {max_flip['perturbation']}.",
        f"- Repeatability mean pairwise JSD: {jsd:.4f}; mean feature-rank Spearman: {rho:.4f}.",
        f"- Exp4 disagreement rate vs highest-fidelity reference: {exp4_dis:.3f}.",
        f"- Exp5 winner retention under perturbation: {lever_ret:.3f}.",
        "",
        "## Claim Tightening",
        "- Treat unwired sampled parameters as metadata unless connected into runtime state update.",
        "- Always publish threshold-sensitivity and seed-repeatability with regime claims.",
        "- Present Exp4/Exp5 decisions as confidence-banded, not deterministic.",
    ]
    (OUT_DIR / "SIMULATION_CREDIBILITY_REPORT.md").write_text("\n".join(text), encoding="utf-8")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    exp1_df = _prep_exp1()
    p = generate_parameter_audit()
    b = generate_backlash_artifact_audit(exp1_df)
    t = generate_regime_threshold_sensitivity(exp1_df)
    r = generate_repeatability_summary()
    e4 = generate_exp4_decision_validity()
    l = generate_lever_stability_audit()
    write_report(p, b, t, r, e4, l)

    manifest = {
        "outputs": [
            "parameter_audit.csv",
            "backlash_artifact_audit.csv",
            "regime_threshold_sensitivity.csv",
            "repeatability_summary.csv",
            "exp4_decision_validity.csv",
            "lever_stability_audit.csv",
            "SIMULATION_CREDIBILITY_REPORT.md",
        ]
    }
    (OUT_DIR / "audit_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"Audit artifacts written to: {OUT_DIR}")


if __name__ == "__main__":
    main()
