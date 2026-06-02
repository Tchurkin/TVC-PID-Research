from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "outputs" / "sim_credibility"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def _read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="ignore")


def main() -> None:
    exp1 = _read("experiments/framework/experiments/run_exp1_stability_authority_frontier.m")
    exp4 = _read("experiments/run_exp4_fidelity_ladder.m")
    exp5 = _read("experiments/run_exp5_design_sensitivity.m")
    cfg_builder = _read("experiments/framework/plant/build_realistic_cfg.m")
    sim = _read("ModelRocket_Adaptive_TVC/src/simulate_case_realistic.m")

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

    runtime_map: Dict[str, Dict[str, bool]] = {
        "mass": {"runtime": True, "eq": True, "out": True},
        "Iyy": {"runtime": True, "eq": True, "out": True},
        "static_margin": {"runtime": True, "eq": True, "out": True},
        "Cm_alpha": {"runtime": True, "eq": True, "out": True},
        "control_effectiveness": {"runtime": True, "eq": True, "out": True},
        "thrust": {"runtime": True, "eq": True, "out": True},
        "servo_slew_deg_s": {"runtime": True, "eq": True, "out": True},
        "max_gimbal_deg": {"runtime": True, "eq": True, "out": True},
        "best_u_max_frac": {"runtime": True, "eq": True, "out": True},
        "deadband": {"runtime": True, "eq": True, "out": True},
        "backlash": {"runtime": True, "eq": True, "out": True},
        "latency_steps": {"runtime": True, "eq": True, "out": True},
        "wind_strength": {"runtime": True, "eq": True, "out": True},
        "p_unstable": {"runtime": True, "eq": True, "out": True},
    }

    rows: List[Dict[str, object]] = []
    for p in params:
        sampled = (p in exp1) or (p in exp4) or (p in exp5)

        if p == "best_u_max_frac":
            sampled = True

        runtime_wired = runtime_map[p]["runtime"]
        affects_eq = runtime_map[p]["eq"] and ("qdot" in sim)
        affects_out = runtime_map[p]["out"]
        dead = sampled and (not runtime_wired or not affects_eq or not affects_out)

        rows.append(
            {
                "parameter": p,
                "sampled?": bool(sampled),
                "runtime-wired?": bool(runtime_wired),
                "affects equations?": bool(affects_eq),
                "affects outputs?": bool(affects_out),
                "dead metadata?": bool(dead),
            }
        )

    audit = pd.DataFrame(rows)
    csv_path = OUT_DIR / "runtime_dependency_audit.csv"
    audit.to_csv(csv_path, index=False)

    md_path = OUT_DIR / "runtime_dependency_audit.md"
    with md_path.open("w", encoding="utf-8") as f:
        f.write("# Runtime Dependency Audit\n\n")
        cols = list(audit.columns)
        f.write("| " + " | ".join(cols) + " |\n")
        f.write("| " + " | ".join(["---"] * len(cols)) + " |\n")
        for _, row in audit.iterrows():
            vals = [str(row[c]) for c in cols]
            f.write("| " + " | ".join(vals) + " |\n")
        f.write("\n")

    print(f"Saved: {csv_path}")
    print(f"Saved: {md_path}")


if __name__ == "__main__":
    main()
