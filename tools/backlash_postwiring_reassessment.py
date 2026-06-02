from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd
import shap
from sklearn.ensemble import RandomForestClassifier
from sklearn.inspection import partial_dependence, permutation_importance
from sklearn.model_selection import train_test_split


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "outputs" / "sim_credibility"
OUT_DIR.mkdir(parents=True, exist_ok=True)

FEATURES = [
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


def _prep(df: pd.DataFrame) -> Tuple[pd.DataFrame, pd.Series]:
    cols = [c for c in FEATURES if c in df.columns]
    x = df[cols].apply(pd.to_numeric, errors="coerce").astype(float)
    x = x.fillna(x.median(numeric_only=True))
    y = df["regime_label"].astype(str)
    return x, y


def _normalize(s: pd.Series) -> pd.Series:
    m = float(s.max()) if len(s) else 0.0
    if m <= 0:
        return pd.Series(np.zeros(len(s)), index=s.index)
    return s / m


def compute_feature_metrics(df: pd.DataFrame, random_state: int = 42) -> pd.DataFrame:
    x, y = _prep(df)

    x_tr, x_te, y_tr, y_te = train_test_split(
        x, y, test_size=0.35, random_state=random_state, stratify=y
    )

    clf = RandomForestClassifier(n_estimators=120, random_state=random_state, n_jobs=-1)
    clf.fit(x_tr, y_tr)

    perm = permutation_importance(
        clf,
        x_te,
        y_te,
        n_repeats=12,
        random_state=random_state,
        scoring="balanced_accuracy",
    )
    perm_s = pd.Series(perm.importances_mean, index=x.columns, name="perm_importance")

    sample_n = min(60, len(x_te))
    x_shap = x_te.sample(n=sample_n, random_state=random_state)

    explainer = shap.TreeExplainer(clf)
    shap_vals = explainer.shap_values(x_shap)
    if isinstance(shap_vals, list):
        arr = np.stack(shap_vals, axis=0)
    else:
        arr = np.asarray(shap_vals)

    arr_abs = np.abs(arr)
    feat_n = x.shape[1]
    feat_axis = None
    for ax, size in enumerate(arr_abs.shape):
        if size == feat_n:
            feat_axis = ax
            break
    if feat_axis is None:
        raise RuntimeError(f"Could not identify feature axis in SHAP array shape {arr_abs.shape}")

    arr_feat_first = np.moveaxis(arr_abs, feat_axis, 0)
    shap_mean_abs = arr_feat_first.reshape(feat_n, -1).mean(axis=1)
    shap_s = pd.Series(shap_mean_abs, index=x.columns, name="shap_importance")

    pdp_strength = {}
    for feat in x.columns:
        try:
            pdp = partial_dependence(clf, x_tr, [feat], kind="average", grid_resolution=10)
            avg = pdp["average"]
            pdp_strength[feat] = float(np.mean(np.std(avg, axis=1)))
        except Exception:
            pdp_strength[feat] = np.nan
    pdp_s = pd.Series(pdp_strength, name="pdp_effect_strength")

    # Interaction proxy: correlation-weighted permutation coupling.
    inter_s = pd.Series(0.0, index=x.columns, name="interaction_strength")
    c = x_tr.corr().abs().fillna(0.0)
    for f in x.columns:
        inter_s[f] = float((c[f] * perm_s).sum() - c.loc[f, f] * perm_s[f])

    out = pd.concat([shap_s, perm_s, pdp_s, inter_s], axis=1)
    out = out.fillna(0.0)

    out["composite_score"] = (
        _normalize(out["shap_importance"])
        + _normalize(out["perm_importance"])
        + _normalize(out["pdp_effect_strength"])
        + _normalize(out["interaction_strength"])
    ) / 4.0

    out = out.sort_values("composite_score", ascending=False)
    out["composite_rank"] = np.arange(1, len(out) + 1)
    out.index.name = "feature"
    return out.reset_index()


def measured_backlash_range_deg() -> Tuple[float, float, str]:
    measured_file = ROOT / "data" / "bench" / "gimbal_bench_test.csv"
    if measured_file.exists():
        d = pd.read_csv(measured_file)
        if {"cmd_deg", "meas_deg"}.issubset(d.columns):
            err = (d["cmd_deg"].astype(float) - d["meas_deg"].astype(float)).abs()
            q = err.quantile([0.25, 0.75]).to_numpy()
            return float(q[0]), float(q[1]), "data/bench/gimbal_bench_test.csv"

    assumed = ROOT / "data" / "bench" / "assumed_gimbal_profile.csv"
    if assumed.exists():
        a = pd.read_csv(assumed)
        if "hysteresis_deg" in a.columns:
            h = float(a["hysteresis_deg"].iloc[0])
            return max(0.0, 0.9 * h), 1.1 * h, "data/bench/assumed_gimbal_profile.csv"

    return np.nan, np.nan, "not_found"


def compute_backlash_rank_fast(df: pd.DataFrame, random_state: int = 42) -> Tuple[float, float]:
    x, y = _prep(df)
    x_tr, x_te, y_tr, y_te = train_test_split(
        x, y, test_size=0.35, random_state=random_state, stratify=y
    )
    clf = RandomForestClassifier(n_estimators=100, random_state=random_state, n_jobs=-1)
    clf.fit(x_tr, y_tr)
    perm = permutation_importance(
        clf,
        x_te,
        y_te,
        n_repeats=8,
        random_state=random_state,
        scoring="balanced_accuracy",
    )
    s = pd.Series(perm.importances_mean, index=x.columns).sort_values(ascending=False)
    if "backlash" not in s.index:
        return np.nan, np.nan
    rank = float(np.where(s.index == "backlash")[0][0] + 1)
    score = float(s.loc["backlash"])
    return rank, score


def sensitivity_table(df_new: pd.DataFrame, baseline_metrics: pd.DataFrame) -> pd.DataFrame:
    work = df_new.copy()
    work["eff_backlash_deg"] = work["backlash"] * work["max_gimbal_deg"] / 12.0

    m_lo, m_hi, src = measured_backlash_range_deg()

    ranges = [
        ("current_sweep", -np.inf, np.inf),
        ("realistic_hobby_tv_range", 0.05, 0.30),
        ("measured_value_range", m_lo, m_hi),
    ]

    base_rank, base_score = compute_backlash_rank_fast(work, random_state=42)

    rows: List[Dict[str, object]] = []
    for name, lo, hi in ranges:
        if np.isfinite(lo) and np.isfinite(hi):
            sub = work[(work["eff_backlash_deg"] >= lo) & (work["eff_backlash_deg"] <= hi)].copy()
        else:
            sub = work.copy()

        if len(sub) < 120:
            rows.append(
                {
                    "range_name": name,
                    "range_deg_min": lo,
                    "range_deg_max": hi,
                    "source": src if name == "measured_value_range" else "n/a",
                    "n_rows": int(len(sub)),
                    "backlash_rank": np.nan,
                    "backlash_score": np.nan,
                    "score_percent_change_vs_current": np.nan,
                    "rank_shift_vs_current": np.nan,
                }
            )
            continue

        rank, score = compute_backlash_rank_fast(sub, random_state=42)
        rows.append(
            {
                "range_name": name,
                "range_deg_min": lo,
                "range_deg_max": hi,
                "source": src if name == "measured_value_range" else "n/a",
                "n_rows": int(len(sub)),
                "backlash_rank": int(rank) if pd.notna(rank) else np.nan,
                "backlash_score": score,
                "score_percent_change_vs_current": ((score - base_score) / max(1e-12, abs(base_score))) * 100.0,
                "rank_shift_vs_current": rank - base_rank,
            }
        )

    return pd.DataFrame(rows)


def main() -> None:
    old_df = pd.read_csv(OUT_DIR / "exp1_regime_index_original_backup.csv")
    new_df = pd.read_csv(ROOT / "experiments" / "results" / "exp1_regime_index.csv")

    old_m = compute_feature_metrics(old_df, random_state=42)
    new_m = compute_feature_metrics(new_df, random_state=42)

    old_map = old_m.set_index("feature")
    new_map = new_m.set_index("feature")

    features = sorted(set(old_map.index).intersection(set(new_map.index)))
    rows = []
    for f in features:
        old_rank = int(old_map.loc[f, "composite_rank"])
        new_rank = int(new_map.loc[f, "composite_rank"])
        old_score = float(old_map.loc[f, "composite_score"])
        new_score = float(new_map.loc[f, "composite_score"])
        pct = ((new_score - old_score) / max(1e-12, abs(old_score))) * 100.0
        rows.append(
            {
                "OLD_RANK": old_rank,
                "NEW_RANK": new_rank,
                "FEATURE": f,
                "OLD_SCORE": old_score,
                "NEW_SCORE": new_score,
                "PERCENT_CHANGE": pct,
            }
        )

    compare = pd.DataFrame(rows).sort_values(["OLD_RANK", "NEW_RANK", "FEATURE"]).reset_index(drop=True)

    b = compare.loc[compare["FEATURE"] == "backlash"].iloc[0]
    status = pd.DataFrame(
        [
            {
                "FEATURE": "backlash",
                "NEW_RANK": int(b["NEW_RANK"]),
                "rank_1_overall": bool(int(b["NEW_RANK"]) == 1),
                "top_3_overall": bool(int(b["NEW_RANK"]) <= 3),
                "no_longer_dominant": bool(int(b["NEW_RANK"]) > 3),
            }
        ]
    )

    sens = sensitivity_table(new_df, new_m)

    ranks = sens["backlash_rank"].dropna()
    scores = sens["backlash_score"].dropna()
    if len(ranks) == 0:
        verdict = "D"
    else:
        new_rank = int(b["NEW_RANK"])
        rank_span = float(ranks.max() - ranks.min()) if len(ranks) else np.inf
        score_cv = float(scores.std(ddof=1) / max(1e-12, scores.mean())) if len(scores) > 1 else 0.0
        if rank_span >= 3 or score_cv > 0.35:
            verdict = "D"
        elif new_rank == 1:
            verdict = "A"
        elif new_rank <= 3:
            verdict = "B"
        else:
            verdict = "C"

    verdict_df = pd.DataFrame([{"FINAL_VERDICT": verdict}])

    old_m.to_csv(OUT_DIR / "importance_metrics_old.csv", index=False)
    new_m.to_csv(OUT_DIR / "importance_metrics_new.csv", index=False)
    compare.to_csv(OUT_DIR / "backlash_rank_comparison.csv", index=False)
    status.to_csv(OUT_DIR / "backlash_dominance_status.csv", index=False)
    sens.to_csv(OUT_DIR / "backlash_range_sensitivity.csv", index=False)
    verdict_df.to_csv(OUT_DIR / "backlash_final_verdict.csv", index=False)

    print("Saved:", OUT_DIR / "importance_metrics_old.csv")
    print("Saved:", OUT_DIR / "importance_metrics_new.csv")
    print("Saved:", OUT_DIR / "backlash_rank_comparison.csv")
    print("Saved:", OUT_DIR / "backlash_dominance_status.csv")
    print("Saved:", OUT_DIR / "backlash_range_sensitivity.csv")
    print("Saved:", OUT_DIR / "backlash_final_verdict.csv")


if __name__ == "__main__":
    main()
