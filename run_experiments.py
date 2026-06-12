"""
run_experiments.py -- top-level driver for the TVC stability study.

Usage:
    python run_experiments.py exp1              # regime mapping (~60-90 min)
    python run_experiments.py exp4_s2r          # S2R gain transfer study (~30 min)
    python run_experiments.py exp4_ablation     # full fidelity ablation (~60 min)
    python run_experiments.py exp4_mechanism    # gain mechanism study, all modules (~3-4 h)
    python run_experiments.py exp4_mech_priority # gain mechanism, priority modules only (~1 h)
    python run_experiments.py all               # exp1 then exp4_s2r sequentially
"""

import sys
import os

# Add sim/ to path so all imports work
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "sim"))

from experiment_runner import (
    run_exp1,
    run_exp4_s2r_gains,
    run_exp4_ablation,
    run_exp4_gain_mechanism,
)


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "help"

    if cmd == "exp1":
        run_exp1(n_designs=1200, seed=42)

    elif cmd == "exp4_s2r":
        run_exp4_s2r_gains()

    elif cmd == "exp4_ablation":
        run_exp4_ablation()

    elif cmd == "exp4_mechanism":
        run_exp4_gain_mechanism()

    elif cmd == "exp4_mech_priority":
        # Quick run: only the modules predicted to drive the Kp gap.
        # wind and sensor_noise are the environment modules; slew and latency
        # are the actuator bandwidth modules.  Pair confirms the combined effect.
        run_exp4_gain_mechanism(
            modules=["wind", "sensor_noise", "slew", "latency"],
            pairs=[("wind", "sensor_noise")],
        )

    elif cmd == "all":
        print("=== Running Exp1 then Exp4-S2R ===")
        df = run_exp1(n_designs=1200, seed=42)
        run_exp4_s2r_gains(designs_df=df)

    else:
        print(__doc__)


if __name__ == "__main__":
    main()
