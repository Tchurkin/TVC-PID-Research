from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = ROOT / "experiments" / "results"
AUDIT_DIR = RESULTS_DIR / "audit"
AUDIT_GRAPH_DIR = AUDIT_DIR / "graphs"
REPORT_PATH = ROOT / "exp5_audit_report.md"

PARAM_ORDER = ["servo_slew_deg_s", "max_gimbal_deg", "Kp", "Kd"]
CASE_ORDER = ["easy_regime", "fragile_regime", "hard_regime"]

PARAM_LABEL = {
    "servo_slew_deg_s": "Servo Slew (deg/s)",
    "max_gimbal_deg": "Max Gimbal (deg)",
    "Kp": "Kp",
    "Kd": "Kd",
}

METRIC_ROWS = [
    ("envelope_cmd_deg", "Envelope (deg)"),
    ("success_like_rate", "Success-like Rate"),
    ("overshoot_pct", "Overshoot (%)"),
    ("actuator_sat_frac", "Actuator Sat Frac"),
    ("rms_err_deg", "RMS Error (deg)"),
]


@dataclass
class AuditArtifacts:
    raw: pd.DataFrame
    threshold: pd.DataFrame
    transitions: pd.DataFrame
    diag_figs: list[Path]
    quant_fig: Path
    sat_summary: pd.DataFrame
    collapse_examples: pd.DataFrame
    mono_flags: pd.DataFrame
    suspicious_labels: pd.DataFrame


def smooth(y: np.ndarray, win: int = 3) -> np.ndarray:
    if len(y) < 3:
        return y.astype(float)
    kernel = np.ones(win) / win
    ypad = np.pad(y.astype(float), (1, 1), mode="edge")
    return np.convolve(ypad, kernel, mode="valid")


def ensure_dirs() -> None:
    AUDIT_GRAPH_DIR.mkdir(parents=True, exist_ok=True)


def load_inputs() -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    raw = pd.read_csv(RESULTS_DIR / "exp5_raw_results.csv")
    threshold = pd.read_csv(RESULTS_DIR / "exp5_threshold_summary.csv")
    transitions = pd.read_csv(RESULTS_DIR / "exp5_bottleneck_transitions.csv")
    return raw, threshold, transitions


def add_threshold_lines(ax: plt.Axes, th_row: pd.DataFrame) -> None:
    if th_row.empty:
        return
    r = th_row.iloc[0]
    ax.axvline(r["failure_threshold"], color="#c83a3a", linestyle="--", linewidth=1.1)
    ax.axvline(r["minimum_practical_threshold"], color="#2f8f45", linestyle="--", linewidth=1.1)
    ax.axvline(r["diminishing_return_threshold"], color="#5b45ad", linestyle=":", linewidth=1.2)


def make_case_diagnostics(raw: pd.DataFrame, threshold: pd.DataFrame, case_id: str) -> Path:
    case_sub = raw[raw["case_id"] == case_id]
    case_label = case_sub["case_label"].iloc[0]

    fig, axes = plt.subplots(len(METRIC_ROWS), len(PARAM_ORDER), figsize=(19, 14), dpi=160)
    for col, param in enumerate(PARAM_ORDER):
        sub = case_sub[case_sub["parameter"] == param].sort_values("param_value")
        x = sub["param_value"].to_numpy()
        th_row = threshold[(threshold["case_id"] == case_id) & (threshold["parameter"] == param)]

        for row, (metric, ylabel) in enumerate(METRIC_ROWS):
            ax = axes[row, col]
            y = sub[metric].to_numpy()
            ax.plot(x, smooth(y), color="#1b4f9a", linewidth=1.8)
            ax.scatter(x, y, s=16, color="#111111", alpha=0.75, zorder=3)
            add_threshold_lines(ax, th_row)
            ax.grid(alpha=0.25)
            ax.set_xlabel(PARAM_LABEL[param])
            if col == 0:
                ax.set_ylabel(ylabel)
            if row == 0:
                ax.set_title(PARAM_LABEL[param], fontweight="bold")

    fig.suptitle(f"Exp5 Audit Diagnostics: {case_label}", fontsize=16, fontweight="bold")
    fig.tight_layout(rect=[0, 0.01, 1, 0.97])
    out_path = AUDIT_GRAPH_DIR / f"exp5_audit_diagnostics_{case_id}.png"
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def make_quantization_plot(raw: pd.DataFrame) -> Path:
    fig, axes = plt.subplots(len(CASE_ORDER), len(PARAM_ORDER), figsize=(16, 10), dpi=150)
    for i, case_id in enumerate(CASE_ORDER):
        case_sub = raw[raw["case_id"] == case_id]
        case_label = case_sub["case_label"].iloc[0]
        for j, param in enumerate(PARAM_ORDER):
            ax = axes[i, j]
            sub = case_sub[case_sub["parameter"] == param].sort_values("param_value")
            ax.step(sub["param_value"], sub["envelope_cmd_deg"], where="mid", color="#224f99")
            ax.scatter(sub["param_value"], sub["envelope_cmd_deg"], s=12, color="#224f99")
            ax.set_yticks(np.arange(0, 31, 5))
            ax.set_xlabel(PARAM_LABEL[param])
            ax.set_ylabel("Envelope (deg)")
            ax.set_title(f"{case_label} | {param}")
            ax.grid(alpha=0.25)
    fig.suptitle("Envelope Quantization and Plateau Structure", fontsize=15, fontweight="bold")
    fig.tight_layout(rect=[0, 0.02, 1, 0.96])
    out_path = AUDIT_GRAPH_DIR / "exp5_audit_envelope_quantization.png"
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def build_saturation_summary(raw: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    max_env = float(raw["envelope_cmd_deg"].max())
    sat_rows = []
    collapse_rows = []

    for case_id in CASE_ORDER:
        for param in PARAM_ORDER:
            sub = raw[(raw["case_id"] == case_id) & (raw["parameter"] == param)].sort_values("param_value")
            env = sub["envelope_cmd_deg"].to_numpy()
            uniq = np.unique(env)
            diffs = np.diff(uniq)
            q_step = float(np.min(diffs)) if len(diffs) else 0.0
            sat_rows.append(
                {
                    "case_id": case_id,
                    "case_label": sub["case_label"].iloc[0],
                    "parameter": param,
                    "n_points": len(sub),
                    "frac_at_ceiling": float(np.mean(env == max_env)),
                    "n_unique_envelope_levels": int(len(uniq)),
                    "min_envelope_deg": float(np.min(env)),
                    "max_envelope_deg": float(np.max(env)),
                    "estimated_quantization_step_deg": q_step,
                }
            )

            for level in uniq:
                same = sub[sub["envelope_cmd_deg"] == level]
                if len(same) < 2:
                    continue
                suc_span = same["success_like_rate"].max() - same["success_like_rate"].min()
                ovs_span = same["overshoot_pct"].max() - same["overshoot_pct"].min()
                rms_span = same["rms_err_deg"].max() - same["rms_err_deg"].min()
                if (suc_span > 0.15) or (ovs_span > 20) or (rms_span > 5):
                    collapse_rows.append(
                        {
                            "case_id": case_id,
                            "case_label": same["case_label"].iloc[0],
                            "parameter": param,
                            "envelope_level_deg": float(level),
                            "n_collapsed_points": int(len(same)),
                            "param_min": float(same["param_value"].min()),
                            "param_max": float(same["param_value"].max()),
                            "success_like_span": float(suc_span),
                            "overshoot_span_pct": float(ovs_span),
                            "rms_span_deg": float(rms_span),
                        }
                    )

    sat_df = pd.DataFrame(sat_rows)
    collapse_df = pd.DataFrame(collapse_rows)
    return sat_df, collapse_df


def build_monotonicity_flags(raw: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for case_id in CASE_ORDER:
        for param in PARAM_ORDER:
            sub = raw[(raw["case_id"] == case_id) & (raw["parameter"] == param)].sort_values("param_value")
            denv = np.diff(sub["envelope_cmd_deg"].to_numpy())
            dec_idx = np.where(denv < 0)[0]
            rows.append(
                {
                    "case_id": case_id,
                    "case_label": sub["case_label"].iloc[0],
                    "parameter": param,
                    "n_envelope_decreases": int(len(dec_idx)),
                    "max_drop_deg": float(np.max(-denv[dec_idx])) if len(dec_idx) else 0.0,
                    "decrease_at_param_values": ", ".join(
                        f"{v:g}" for v in sub["param_value"].iloc[(dec_idx + 1)].to_numpy()
                    ),
                }
            )
    return pd.DataFrame(rows)


def build_suspicious_labels(raw: pd.DataFrame) -> pd.DataFrame:
    return raw[
        (raw["active_bottleneck"] == "diminishing_returns")
        & ((raw["slew_sat_frac"] > 0.40) | (raw["actuator_sat_frac"] > 0.40))
    ].sort_values(["case_id", "parameter", "param_value"])


def pick_examples(raw: pd.DataFrame) -> pd.DataFrame:
    picks = [
        ("easy_regime", "max_gimbal_deg", 2),
        ("easy_regime", "Kp", 2),
        ("fragile_regime", "max_gimbal_deg", 2),
        ("fragile_regime", "servo_slew_deg_s", 120),
        ("hard_regime", "Kp", 45),
    ]
    rows = []
    for case_id, param, value in picks:
        m = raw[(raw["case_id"] == case_id) & (raw["parameter"] == param) & (np.isclose(raw["param_value"], value))]
        if not m.empty:
            rows.append(m.iloc[0])
    return pd.DataFrame(rows)


def to_rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def write_report(a: AuditArtifacts) -> None:
    max_env = float(a.raw["envelope_cmd_deg"].max())
    examples = pick_examples(a.raw)

    lines: list[str] = []
    lines.append("# Exp5 Audit Report")
    lines.append("")
    lines.append("## Major Findings")
    lines.append(f"- Envelope metric is quantized in 5 deg bins and hard-capped at {max_env:.0f} deg.")
    lines.append("- Thresholds are computed only from envelope_cmd_deg, so crossings can be dominated by quantization and plateaus.")
    lines.append("- Bottleneck labeling order can classify saturated points as diminishing_returns.")
    lines.append("- Some non-monotonic trends are physically plausible instability effects, but several headline thresholds are analysis artifacts.")
    lines.append("")

    lines.append("## 1) Physical Plausibility Validation")
    lines.append("- Kp increase reducing envelope in easy/fragile regimes is physically plausible when overshoot and saturation rise sharply.")
    lines.append("- Gimbal increase reducing envelope is suspicious for a pure authority sweep; here the sweep also scales control_effectiveness, effectively changing loop gain and confounding interpretation.")
    lines.append("- Hard regime remains mostly infeasible (frequent zero-envelope outcomes), consistent with severe control limits in those baseline settings.")
    lines.append("")
    lines.append("### Suspicious Results (Concrete Examples)")
    for _, r in examples.iterrows():
        lines.append(
            f"- {r['case_id']} {r['parameter']}={r['param_value']:.0f}: envelope={r['envelope_cmd_deg']:.1f}, "
            f"success_like={r['success_like_rate']:.3f}, overshoot={r['overshoot_pct']:.1f}%, "
            f"act_sat={r['actuator_sat_frac']:.3f}, slew_sat={r['slew_sat_frac']:.3f}, label={r['active_bottleneck']}"
        )
    lines.append("")

    lines.append("## 2) Maneuverability Envelope Metric Audit")
    lines.append("- Computation path in Exp5 runner:")
    lines.append("  - Evaluate command grid: [5, 10, 15, 20, 25, 30] deg.")
    lines.append("  - For each command, pass if all gates hold: success_rate >= 0.50, rms_error_deg <= 15, peak_error_deg <= 35.")
    lines.append("  - envelope_cmd_deg = largest passing command; else 0 if none pass.")
    lines.append("- What increases envelope: higher command bins crossing all three gates.")
    lines.append("- What decreases envelope: instability/saturation that causes RMS or peak-error gate failures at previously passing bins.")
    lines.append("- Ceiling behavior: yes, fixed ceiling at 30 deg.")
    lines.append("- Instability sensitivity: high; envelope can collapse to zero while other metrics still vary.")
    lines.append("- Threshold dominance risk: high, because threshold logic uses envelope only.")
    lines.append("")

    lines.append("## 3) Bottleneck Classifier Audit")
    lines.append("- Label assignment order:")
    lines.append("  - diminishing_returns if envelope_cmd_ratio >= 0.95 and bounded_window_norm >= 0.95")
    lines.append("  - slew_limited if slew_sat_frac >= 0.55 and >= actuator_sat_frac + 0.05")
    lines.append("  - authority_limited if actuator_sat_frac >= 0.55")
    lines.append("  - controller_limited if sweeping Kp/Kd or overshoot >= 15 or bounded window < 0.90")
    lines.append("  - mixed_limited otherwise")
    lines.append("")

    if len(a.suspicious_labels) > 0:
        lines.append("### Clearly Incorrect/Questionable Labels")
        for _, r in a.suspicious_labels.head(8).iterrows():
            lines.append(
                f"- {r['case_id']} {r['parameter']}={r['param_value']:.0f}: label={r['active_bottleneck']}, "
                f"slew_sat={r['slew_sat_frac']:.3f}, act_sat={r['actuator_sat_frac']:.3f}, env={r['envelope_cmd_deg']:.1f}"
            )
        lines.append("")

    lines.append("### Recommended Classifier Logic")
    lines.append("- Check instability and saturation first; only then allow diminishing_returns classification.")
    lines.append("- Require low saturation and low slope over at least 2-3 adjacent parameter points for diminishing_returns.")
    lines.append("- Add an explicit unstable_or_diverging class for high overshoot/high RMS collapse cases.")
    lines.append("")

    lines.append("## 4) Diagnostic Plots")
    lines.append("Per-regime diagnostics (A-H coverage, thresholds, smoothing, raw points):")
    for p in a.diag_figs:
        lines.append(f"- ![]({to_rel(p)})")
    lines.append("Quantization and plateau diagnostic:")
    lines.append(f"- ![]({to_rel(a.quant_fig)})")
    lines.append("")

    lines.append("## 5) Hidden Saturation Effects")
    lines.append("### Ceiling/Quantization Summary")
    lines.append("| case | parameter | frac_at_ceiling | unique_levels | min_env | max_env | quant_step |")
    lines.append("|---|---|---:|---:|---:|---:|---:|")
    for _, r in a.sat_summary.iterrows():
        lines.append(
            f"| {r['case_id']} | {r['parameter']} | {r['frac_at_ceiling']:.3f} | {int(r['n_unique_envelope_levels'])} | "
            f"{r['min_envelope_deg']:.1f} | {r['max_envelope_deg']:.1f} | {r['estimated_quantization_step_deg']:.1f} |"
        )
    lines.append("")

    if len(a.collapse_examples) > 0:
        lines.append("### Collapsed Envelope with Hidden Metric Variation")
        lines.append("| case | parameter | envelope | points | param span | success span | overshoot span | rms span |")
        lines.append("|---|---|---:|---:|---|---:|---:|---:|")
        for _, r in a.collapse_examples.head(12).iterrows():
            lines.append(
                f"| {r['case_id']} | {r['parameter']} | {r['envelope_level_deg']:.1f} | {int(r['n_collapsed_points'])} | "
                f"{r['param_min']:.0f}-{r['param_max']:.0f} | {r['success_like_span']:.3f} | "
                f"{r['overshoot_span_pct']:.1f} | {r['rms_span_deg']:.1f} |"
            )
        lines.append("")

    lines.append("## 6) True Engineering Conclusions")
    lines.append("### PHYSICALLY SUPPORTED")
    lines.append("- High Kp can degrade maneuverability envelope by driving overshoot and actuator saturation.")
    lines.append("- Kd usually improves envelope up to a plateau in easy/fragile regimes.")
    lines.append("- Hard regime is predominantly limited, with low achievable envelope under tested baselines.")
    lines.append("")
    lines.append("### LIKELY ANALYSIS ARTIFACTS")
    lines.append("- Early diminishing-return thresholds at lowest tested values for gimbal/Kp in easy/fragile regimes.")
    lines.append("- Thresholds reported at extreme sweep endpoints when max envelope is zero (procedural, not physical).")
    lines.append("- Bottleneck claims based only on classifier labels without support from overshoot/saturation/RMS trends.")
    lines.append("")

    lines.append("## 7) Recommended Fixes")
    lines.append("- Refine envelope search resolution to 1-2 deg, or estimate pass/fail crossing continuously.")
    lines.append("- Revise threshold logic to use local slope and robustness, not first crossing only.")
    lines.append("- Reorder and harden bottleneck classifier with instability-first rules.")
    lines.append("- Decouple authority sweep from control_effectiveness scaling for physically clean gimbal conclusions.")
    lines.append("- Add uncertainty bands (seed distribution) rather than only mean metrics.")
    lines.append("")
    lines.append("## Trustworthiness for Paper Use")
    lines.append("- Use Exp5 qualitatively for instability and saturation patterns.")
    lines.append("- Do not use current diminishing-return thresholds or bottleneck boundary claims as final paper evidence without metric/classifier fixes.")

    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    ensure_dirs()
    raw, threshold, transitions = load_inputs()

    diag_figs = [make_case_diagnostics(raw, threshold, cid) for cid in CASE_ORDER]
    quant_fig = make_quantization_plot(raw)
    sat_summary, collapse_examples = build_saturation_summary(raw)
    mono_flags = build_monotonicity_flags(raw)
    suspicious_labels = build_suspicious_labels(raw)

    sat_summary.to_csv(AUDIT_DIR / "exp5_audit_saturation_summary.csv", index=False)
    collapse_examples.to_csv(AUDIT_DIR / "exp5_audit_envelope_collapse_examples.csv", index=False)
    mono_flags.to_csv(AUDIT_DIR / "exp5_audit_monotonicity_flags.csv", index=False)
    suspicious_labels.to_csv(AUDIT_DIR / "exp5_audit_suspicious_labels.csv", index=False)

    artifacts = AuditArtifacts(
        raw=raw,
        threshold=threshold,
        transitions=transitions,
        diag_figs=diag_figs,
        quant_fig=quant_fig,
        sat_summary=sat_summary,
        collapse_examples=collapse_examples,
        mono_flags=mono_flags,
        suspicious_labels=suspicious_labels,
    )
    write_report(artifacts)

    print(f"Saved report: {REPORT_PATH}")
    print(f"Saved audit tables under: {AUDIT_DIR}")
    print(f"Saved audit plots under: {AUDIT_GRAPH_DIR}")


if __name__ == "__main__":
    main()
