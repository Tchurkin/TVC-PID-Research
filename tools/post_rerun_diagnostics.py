#!/usr/bin/env python3
"""Generate post-rerun diagnostics proving parameter variation and dataset readiness."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = ROOT / "experiments" / "results"
OUT_DIR = ROOT / "outputs" / "diagnostics_postrerun"


def _read_csv(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"Missing required file: {path}")
    return pd.read_csv(path)


def _coerce_numeric(df: pd.DataFrame, cols: Iterable[str]) -> None:
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")


def _param_columns(df: pd.DataFrame) -> list[str]:
    aliases = {
        "nominal_servo_slew_deg_s": "servo_slew",
        "nominal_max_gimbal_deg": "max_gimbal",
    }
    cols = [
        "mass",
        "Iyy",
        "static_margin",
        "Cm_alpha",
        "control_effectiveness",
        "thrust",
        "deadband",
        "backlash",
        "latency",
        "wind_strength",
        "servo_slew",
        "max_gimbal",
    ]
    out = []
    for c in cols:
        if c in df.columns:
            out.append(c)
            continue
        for src, dst in aliases.items():
            if dst == c and src in df.columns:
                df[c] = df[src]
                out.append(c)
                break
    return out


def variation_report(exp4: pd.DataFrame) -> tuple[pd.DataFrame, list[str]]:
    params = _param_columns(exp4)
    _coerce_numeric(exp4, params)
    rows = []
    for c in params:
        s = exp4[c]
        rows.append(
            {
                "parameter": c,
                "min": float(s.min()) if s.notna().any() else np.nan,
                "max": float(s.max()) if s.notna().any() else np.nan,
                "mean": float(s.mean()) if s.notna().any() else np.nan,
                "std": float(s.std(ddof=1)) if s.notna().sum() > 1 else 0.0,
                "unique_values": int(s.nunique(dropna=True)),
                "missing": int(s.isna().sum()),
            }
        )
    return pd.DataFrame(rows), params


def design_integrity_report(exp4: pd.DataFrame, params: list[str]) -> pd.DataFrame:
    if params:
        _ = _param_columns(exp4)
        _coerce_numeric(exp4, params)
    total_rows = len(exp4)
    design_vectors = exp4[params] if params else pd.DataFrame(index=exp4.index)
    unique_vectors = design_vectors.drop_duplicates().shape[0] if params else 0
    duplicates = total_rows - unique_vectors
    duplicate_ratio = duplicates / total_rows if total_rows else np.nan
    missing_any = int(design_vectors.isna().any(axis=1).sum()) if params else 0
    missing_all = int(design_vectors.isna().all(axis=1).sum()) if params else 0

    out = pd.DataFrame(
        [
            {
                "rows": total_rows,
                "rocket_ids_unique": int(exp4["rocket_id"].nunique()) if "rocket_id" in exp4.columns else np.nan,
                "parameter_columns_used": ",".join(params),
                "unique_parameter_vectors": int(unique_vectors),
                "duplicate_parameter_vectors": int(duplicates),
                "duplicate_parameter_vector_ratio": float(duplicate_ratio),
                "rows_missing_any_parameter": missing_any,
                "rows_missing_all_parameters": missing_all,
            }
        ]
    )
    return out


def exp1_diagnostics(exp1: pd.DataFrame, exp4: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    regime_col = "regime_label"
    cnt = exp1[regime_col].value_counts(dropna=False)
    pct = exp1[regime_col].value_counts(normalize=True, dropna=False) * 100.0
    regime_df = pd.DataFrame(
        {
            "regime_label": cnt.index.astype(str),
            "count": cnt.values,
            "percentage": [float(pct[k]) for k in cnt.index],
        }
    )

    bd_source = exp1 if "boundary_distance" in exp1.columns else exp4
    bd = pd.to_numeric(bd_source.get("boundary_distance", pd.Series(dtype=float)), errors="coerce")
    summary = pd.DataFrame(
        [
            {
                "count": int(bd.notna().sum()),
                "min": float(bd.min()) if bd.notna().any() else np.nan,
                "p25": float(bd.quantile(0.25)) if bd.notna().any() else np.nan,
                "median": float(bd.median()) if bd.notna().any() else np.nan,
                "p75": float(bd.quantile(0.75)) if bd.notna().any() else np.nan,
                "max": float(bd.max()) if bd.notna().any() else np.nan,
                "mean": float(bd.mean()) if bd.notna().any() else np.nan,
                "std": float(bd.std(ddof=1)) if bd.notna().sum() > 1 else 0.0,
            }
        ]
    )
    return regime_df, summary


def exp4_diagnostics(exp4: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    phy = exp4["dominant_missing_physics"].fillna("MISSING")
    cnt = phy.value_counts(dropna=False)
    pct = (cnt / len(exp4)) * 100.0 if len(exp4) else cnt
    counts = pd.DataFrame(
        {
            "dominant_missing_physics": cnt.index.astype(str),
            "count": cnt.values,
            "percentage": [float(pct[k]) for k in cnt.index],
        }
    )

    ctab = pd.crosstab(exp4["regime_label"], exp4["dominant_missing_physics"], dropna=False)
    ctab = ctab.reset_index()
    return counts, ctab


def exp5_readiness(exp5_summary: pd.DataFrame, exp5_cells: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    regime_cov = (
        exp5_summary.groupby("regime_label", dropna=False)
        .agg(
            rockets=("rocket_id", "nunique"),
            best_levers=("best_design_lever", "nunique"),
            mean_expected_improvement=("expected_improvement", "mean"),
            mean_tradeoff_score=("tradeoff_score", "mean"),
        )
        .reset_index()
    )

    fidelity_region = pd.crosstab(exp5_summary["regime_label"], exp5_summary["best_design_lever"], dropna=False)
    fidelity_region = fidelity_region.reset_index()

    perf_landscape = (
        exp5_cells.groupby("design_lever", dropna=False)
        .agg(
            rows=("rocket_id", "count"),
            mean_success_improvement=("expected_success_improvement", "mean"),
            median_success_improvement=("expected_success_improvement", "median"),
            mean_stability_gain=("expected_stability_gain", "mean"),
            mean_maneuverability_gain=("expected_maneuverability_gain", "mean"),
            mean_tradeoff=("tradeoff_score", "mean"),
        )
        .reset_index()
        .sort_values("mean_success_improvement", ascending=False)
    )

    bottlenecks = (
        exp5_summary.groupby("worst_bottleneck", dropna=False)
        .agg(
            count=("rocket_id", "count"),
            mean_expected_improvement=("expected_improvement", "mean"),
            mean_worst_tradeoff_score=("worst_tradeoff_score", "mean"),
        )
        .reset_index()
        .sort_values("count", ascending=False)
    )

    return regime_cov, fidelity_region, perf_landscape, bottlenecks


def build_verdict(variation: pd.DataFrame, integrity: pd.DataFrame) -> pd.DataFrame:
    unique_gt1 = int((variation["unique_values"] > 1).sum()) if not variation.empty else 0
    total = int(len(variation))
    unique_vectors = int(integrity.loc[0, "unique_parameter_vectors"])
    rows = int(integrity.loc[0, "rows"])

    substantial_variation = unique_gt1 >= max(4, total // 2)
    vector_diversity = unique_vectors > max(1, rows // 20)
    verdict = "PASS" if substantial_variation and vector_diversity else "FAIL"

    return pd.DataFrame(
        [
            {
                "verdict": verdict,
                "reason": (
                    "Substantial multi-parameter variation detected and diverse parameter vectors present"
                    if verdict == "PASS"
                    else "Insufficient multi-parameter variation and/or weak parameter-vector diversity"
                ),
                "parameters_with_gt1_unique": unique_gt1,
                "total_parameters_checked": total,
                "unique_parameter_vectors": unique_vectors,
                "rows": rows,
            }
        ]
    )


def find_exp5_cells_file() -> Path:
    candidates = [
        RESULTS_DIR / "exp5_design_lever_cell.csv",
        RESULTS_DIR / "exp5_design_lever_cells.csv",
    ]
    for c in candidates:
        if c.exists():
            return c
    raise FileNotFoundError("Missing Exp5 cell file (exp5_design_lever_cell.csv or exp5_design_lever_cells.csv)")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    exp1 = _read_csv(RESULTS_DIR / "exp1_regime_index.csv")
    exp4 = _read_csv(RESULTS_DIR / "exp4_first_correct_fidelity.csv")
    exp5_summary = _read_csv(RESULTS_DIR / "exp5_design_lever_summary.csv")
    exp5_cells = _read_csv(find_exp5_cells_file())

    variation, params = variation_report(exp4.copy())
    integrity = design_integrity_report(exp4.copy(), params)
    exp1_counts, exp1_boundary = exp1_diagnostics(exp1.copy(), exp4.copy())
    exp4_counts, exp4_ctab = exp4_diagnostics(exp4.copy())
    reg_map, fidelity_map, perf_map, bottleneck = exp5_readiness(exp5_summary.copy(), exp5_cells.copy())
    verdict = build_verdict(variation, integrity)

    variation.to_csv(OUT_DIR / "parameter_variation_report.csv", index=False)
    integrity.to_csv(OUT_DIR / "design_space_integrity_report.csv", index=False)
    exp1_counts.to_csv(OUT_DIR / "exp1_regime_counts_percentages.csv", index=False)
    exp1_boundary.to_csv(OUT_DIR / "exp1_boundary_distance_distribution.csv", index=False)
    exp4_counts.to_csv(OUT_DIR / "exp4_dominant_missing_physics_counts.csv", index=False)
    exp4_ctab.to_csv(OUT_DIR / "exp4_regime_x_physics_crosstab.csv", index=False)
    reg_map.to_csv(OUT_DIR / "exp5_readiness_regime_mapping.csv", index=False)
    fidelity_map.to_csv(OUT_DIR / "exp5_readiness_fidelity_region_mapping.csv", index=False)
    perf_map.to_csv(OUT_DIR / "exp5_readiness_performance_landscape_mapping.csv", index=False)
    bottleneck.to_csv(OUT_DIR / "exp5_readiness_bottleneck_analysis.csv", index=False)
    verdict.to_csv(OUT_DIR / "post_rerun_verdict.csv", index=False)

    print(f"Saved diagnostics to: {OUT_DIR}")
    print(verdict.to_string(index=False))


if __name__ == "__main__":
    main()
