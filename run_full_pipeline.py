"""
run_full_pipeline.py — Single entry point for the full TVC research pipeline.

Stages
──────
  1. Load (or re-run) Exp1 design space regime mapping
  2. Fit probabilistic regime boundary classifiers (LR + RF)
  3. Augment raw data with per-design probabilistic predictions
  4. Build stability frontier manifold
  5. Compute permutation importance for three outcome targets
  6. Save all artifacts to results/

Usage
─────
  python run_full_pipeline.py                 # use cached Exp1 data
  python run_full_pipeline.py --rerun-exp1    # re-run Exp1 (slow)
  python run_full_pipeline.py --n 400         # smaller run for testing
  python run_full_pipeline.py --seed 99       # different random seed

Outputs (all reproducible from the same seed)
──────────────────────────────────────────────
  results/raw_runs.parquet        Exp1 data + model-predicted probabilities
  results/regime_model.pkl        fitted LR + RF models + bootstrap CIs
  results/frontier_map.parquet    P(regime | params) over 2-D design slices
  results/sensitivity.json        permutation importance ranked by target

Reproducibility
───────────────
  All randomness is seeded via --seed (default 42 for Exp1 LHS, 100 for bootstrap).
  The simulator uses SEED_SIMULATION=1 for all evaluations (see experiment_protocol.py).
  Given identical seeds and N, the full pipeline is deterministic.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import pandas as pd

# ── Paths ──────────────────────────────────────────────────────────────────────
ROOT       = Path(__file__).resolve().parent
SIM_DIR    = ROOT / "sim"
RESULT_DIR = ROOT / "results"
EXP_DIR    = ROOT / "experiments" / "results"
RESULT_DIR.mkdir(parents=True, exist_ok=True)

# Add sim/ to path so analysis/ can see experiment_runner, etc.
sys.path.insert(0, str(SIM_DIR))
sys.path.insert(0, str(ROOT))

# ── Imports (after path setup) ────────────────────────────────────────────────
from sim.experiment_protocol import (
    N_DESIGNS_EXP1, SEED_PRIMARY, N_BOOTSTRAP, DESIGN_FEATURES, ARTIFACT_PATHS
)
from analysis.regime_boundary import (
    fit_regime_boundary, predict_probabilities, save_model, print_summary as rb_summary,
)
from analysis.frontier import build_frontier_grid
from analysis.sensitivity import (
    permutation_importance_analysis, save_sensitivity, print_summary as sens_summary,
)


def _section(title: str) -> None:
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def load_or_run_exp1(
    rerun: bool,
    n: int,
    seed: int,
    n_jobs: int,
) -> pd.DataFrame:
    """Load Exp1 from cache or re-run if requested."""
    cached_py  = EXP_DIR / "exp1_regime_index_py.csv"
    cached_mat = EXP_DIR / "exp1_regime_index.csv"

    if not rerun and cached_py.exists():
        _section("Stage 1: Load Exp1 (cached Python results)")
        df = pd.read_csv(cached_py)
        print(f"  Loaded {len(df)} designs from {cached_py}")
        # Warn if cached data has fewer designs than requested
        if len(df) < n:
            print(f"  [WARN] Cached N={len(df)} < requested N={n}. "
                  f"Pass --rerun-exp1 to regenerate.")
        return df

    if not rerun and cached_mat.exists():
        _section("Stage 1: Load Exp1 (cached MATLAB results)")
        df = pd.read_csv(cached_mat)
        print(f"  Loaded {len(df)} designs from {cached_mat}")
        print("  [NOTE] Using MATLAB Exp1 results. Run with --rerun-exp1 for Python baseline.")
        return df

    _section("Stage 1: Run Exp1 (design space regime mapping)")
    # Import here to avoid loading joblib/scipy unless needed
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "experiment_runner", SIM_DIR / "experiment_runner.py"
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    t0 = time.time()
    df = mod.run_exp1(n_designs=n, seed=seed, n_jobs=n_jobs)
    elapsed = time.time() - t0
    print(f"  Exp1 complete in {elapsed:.1f}s")
    return df


def main(
    rerun_exp1: bool = False,
    n: int = N_DESIGNS_EXP1,
    seed: int = SEED_PRIMARY,
    n_bootstrap: int = N_BOOTSTRAP,
    n_jobs: int = -1,
) -> None:
    t_start = time.time()
    print(f"\nTVC Research Pipeline")
    print(f"  N={n}  seed={seed}  bootstrap={n_bootstrap}  n_jobs={n_jobs}")

    # ── Stage 1: Exp1 data ────────────────────────────────────────────────────
    df = load_or_run_exp1(rerun_exp1, n, seed, n_jobs)

    # Verify required columns are present
    missing = [f for f in DESIGN_FEATURES if f not in df.columns]
    if missing:
        print(f"[ERROR] Exp1 data missing features: {missing}")
        sys.exit(1)
    if "regime_code" not in df.columns:
        print("[ERROR] Exp1 data missing 'regime_code' column")
        sys.exit(1)

    counts = df["regime_label"].value_counts()
    print(f"  Regime distribution: EASY={counts.get('EASY',0)}  "
          f"FRAGILE={counts.get('FRAGILE',0)}  "
          f"INFEASIBLE={counts.get('INFEASIBLE',0)}")

    if len(df) < 100:
        print("[WARN] Very small dataset — results will be unreliable.")

    # ── Stage 2: Regime boundary model ───────────────────────────────────────
    _section("Stage 2: Regime boundary estimation (LR + RF + bootstrap)")
    t2 = time.time()
    model_bundle = fit_regime_boundary(df, n_bootstrap=n_bootstrap, seed=seed + 100)
    print(f"  Fit complete in {time.time() - t2:.1f}s")
    rb_summary(model_bundle)

    regime_model_path = ROOT / ARTIFACT_PATHS["regime_model"]
    save_model(model_bundle, regime_model_path)

    # ── Stage 3: Augment raw data with probabilistic predictions ─────────────
    _section("Stage 3: Probabilistic prediction on raw runs")
    df_aug = predict_probabilities(model_bundle, df)

    # Gate decomposition: identify which success gates fail
    if all(c in df_aug.columns for c in ["max_theta_deg", "rms_error_deg",
                                           "end_error_deg", "peak_error_deg"]):
        df_aug["gate_fail_stability"] = (df_aug["max_theta_deg"]  >= 70.0).astype(int)
        df_aug["gate_fail_tracking"]  = (df_aug["rms_error_deg"]  >= 15.0).astype(int)
        df_aug["gate_fail_terminal"]  = (df_aug["end_error_deg"]  >= 15.0).astype(int)
        df_aug["gate_fail_peak"]      = (df_aug["peak_error_deg"] >= 50.0).astype(int)
        df_aug["n_gates_failed"]      = (
            df_aug["gate_fail_stability"]
            + df_aug["gate_fail_tracking"]
            + df_aug["gate_fail_terminal"]
            + df_aug["gate_fail_peak"]
        )

    raw_path = ROOT / ARTIFACT_PATHS["raw_runs"]
    df_aug.to_parquet(raw_path, index=False)
    print(f"  Saved: {raw_path}  ({len(df_aug)} rows, {len(df_aug.columns)} columns)")

    # Print probabilistic summary
    print(f"\n  Population P(success)    : {df_aug['p_success'].mean():.3f} "
          f"+/- {df_aug['p_success'].std():.3f}")
    print(f"  Population P(EASY)       : {df_aug['p_easy'].mean():.3f} "
          f"+/- {df_aug['p_easy'].std():.3f}")
    print(f"  Population P(INFEASIBLE) : {df_aug['p_infeasible'].mean():.3f} "
          f"+/- {df_aug['p_infeasible'].std():.3f}")
    print(f"  Population CI (EASY)     : "
          f"[{model_bundle['population_easy_ci95'][0]:.3f}, "
          f"{model_bundle['population_easy_ci95'][1]:.3f}] 95%")

    # ── Stage 4: Stability frontier manifold ──────────────────────────────────
    _section("Stage 4: Stability frontier manifold")
    t4 = time.time()
    frontier_df = build_frontier_grid(model_bundle, df)
    frontier_path = ROOT / ARTIFACT_PATHS["frontier_map"]
    frontier_df.to_parquet(frontier_path, index=False)
    print(f"  Grid complete in {time.time() - t4:.1f}s")
    print(f"  Saved: {frontier_path}  ({len(frontier_df)} grid points, "
          f"{frontier_df['slice_name'].nunique()} slices)")

    # Print frontier extremes
    for slice_name in frontier_df["slice_name"].unique():
        s = frontier_df[frontier_df["slice_name"] == slice_name]
        print(f"    {slice_name:<35s} "
              f"P(success) range [{s['p_success'].min():.2f}, {s['p_success'].max():.2f}]")

    # ── Stage 5: Permutation importance ───────────────────────────────────────
    _section("Stage 5: Permutation importance")
    t5 = time.time()
    sens_results = permutation_importance_analysis(
        df_aug, model_bundle, n_permutations=30, seed=seed
    )
    print(f"  Permutation analysis in {time.time() - t5:.1f}s")
    sens_path = ROOT / ARTIFACT_PATHS["sensitivity"]
    save_sensitivity(sens_results, sens_path)
    sens_summary(sens_results)

    # ── Done ──────────────────────────────────────────────────────────────────
    _section("Pipeline complete")
    elapsed = time.time() - t_start
    print(f"  Total time: {elapsed:.1f}s")
    print(f"\n  Artifacts:")
    for key, rel_path in ARTIFACT_PATHS.items():
        p = ROOT / rel_path
        size = f"{p.stat().st_size / 1024:.1f} KB" if p.exists() else "missing"
        print(f"    {key:<20s} {rel_path}  ({size})")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="TVC stability research pipeline")
    parser.add_argument("--rerun-exp1",  action="store_true",
                        help="Re-run Exp1 instead of using cached results")
    parser.add_argument("--n",           type=int, default=N_DESIGNS_EXP1,
                        help=f"Number of LHS designs (default {N_DESIGNS_EXP1})")
    parser.add_argument("--seed",        type=int, default=SEED_PRIMARY,
                        help=f"Primary random seed (default {SEED_PRIMARY})")
    parser.add_argument("--bootstrap",   type=int, default=N_BOOTSTRAP,
                        help=f"Bootstrap iterations (default {N_BOOTSTRAP})")
    parser.add_argument("--n-jobs",      type=int, default=-1,
                        help="Parallel jobs (-1 = all cores, default -1)")
    args = parser.parse_args()

    main(
        rerun_exp1  = args.rerun_exp1,
        n           = args.n,
        seed        = args.seed,
        n_bootstrap = args.bootstrap,
        n_jobs      = args.n_jobs,
    )
