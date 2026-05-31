Project Summary: A Bench-Calibrated Preflight Tool for Amateur TVC Rockets

What this project is
This project builds and validates a free, open-source preflight workflow for amateur thrust-vector-control rockets. The workflow replaces the current hobbyist practice of "copy a published PID, fly, crash, retune, repeat" with a quantitative pipeline: bench-measure the real actuator, propagate that measurement through a realistic flight model, classify the rocket as GO / MARGINAL / NOGO, and recommend starting PD gains.

Why it matters
The amateur TVC community is growing fast, driven by propulsive-landing demonstrations (BPS.space and similar) and educational kits. These rockets are intentionally statically unstable and live close to actuator limits, which is exactly where the gap between the assumed servo and the real one decides whether the rocket stabilizes or falls over. A low-cost preflight method for measuring that gap and using it before flight is directly useful, safety-relevant, and currently missing from the community's open-source toolchain.

Core contribution
The contribution is not a new control law. It is a measurement-to-decision pipeline that does not currently exist in the amateur TVC space:
1. Bench-measures the actuator and linkage using analog-feedback servo data.
2. Carries the measurements into a realistic flight model with documented sensor and actuator nonidealities.
3. Classifies the rocket as GO, MARGINAL, or NOGO using an empirically calibrated phase diagram.
4. Recommends starting PD gains tuned on the realistic plant rather than on an over-clean basic sim.
5. Reports which parts of the sim-to-real reality gap actually matter in each operating regime, so the model does not have to be everything-to-everyone.

What is already built (simulation side)
1. Realistic MATLAB TVC simulator with sensor noise/bias/latency, servo deadband/backlash, gusts, and effectiveness drift.
2. Calibrated regime classifier (GO precision 91%, NOGO precision 100% on held-out random configs).
3. Empirical scaling law: required loaded slew rate grows approximately as the square of the airframe instability, matching first-principles inverted-pendulum theory in this regime.
4. `tools/bench_to_autotune.m` bridge that turns a bench CSV into a GO/MARGINAL/NOGO verdict plus tuned PD gains.
5. Leave-one-out sim-to-real ablation showing which factor dominates in which regime (sensor noise and effectiveness drift at the boundary; actuator nonlinearity in the actuator-limited regime).
6. "Basic sim vs better sim" comparison showing that tuning a controller on a clean sim and deploying it on the realistic plant loses meaningful success rate (LOW_DEMAND 0.75 vs 1.00; BOUNDARY 0.58 vs 0.83).
7. Audit of three publicly deployed open-source TVC firmwares showing their shipped PID gains fail on statically unstable plants.

Framing
This is best understood as an engineering tool for an underserved community, not as a discovery paper. The novelty is the pipeline and the empirical calibration, not new control theory.

Current simulation takeaways
1. The regime classifier itself is strong; the original LQR gain recipe needed to be replaced by measured-plant PD tuning, and that replacement is in the workflow.
2. The required actuator slew rises roughly quadratically with airframe instability, which is the headline structural finding.
3. Actuator nonlinearity is the main isolated realism factor that hurts performance in actuator-limited regimes; sensor noise and effectiveness drift dominate at the boundary.
4. Open-source hobby firmware gains fail predictably on statically unstable airframes; the validator's value in those cases is gain replacement, not GO/NOGO triage.
5. Additional MATLAB-only novelty hunting is unlikely to be the strongest remaining contribution. Real hardware measurements and a held-out launch comparison are the important next evidence sources.

Who this helps
1. Amateur and educational TVC builders.
2. University rocketry and controls teams that fly small TVC vehicles.
3. Open-source flight-control communities that currently lack a reproducible preflight workflow.

Mentorship requested
Most useful mentor backgrounds:
1. Guidance, navigation, and control.
2. Sim-to-real validation and experimental design.
3. Embedded aerospace systems or actuator characterization.

Most useful mentor help:
1. Pressure-testing whether the evidence really supports the proposed workflow.
2. Reviewing fairness of the gain-recommendation comparisons.
3. Advising on how to present the work as a narrow, honest, builder-facing engineering tool rather than as a discovery claim.

Summer plan
1. June: lock the bench protocol; collect repeatable actuator datasets across realistic loaded and thermal conditions.
2. July: use the bench-to-autotune path on measured data; build held-out comparison cases against copied firmware gains and against the basic-sim baseline.
3. August: run repeated ground and flight tests with the measured-data recommendations; check verdict accuracy and gain quality on the real rocket.
4. September: freeze the main claim and convert the strongest results into final figures and tables.

One-line thesis
This project builds and validates a free, open-source preflight workflow for amateur TVC rockets that replaces "tune until it flies" with bench-measured stabilizability prediction and starting-gain recommendation.
