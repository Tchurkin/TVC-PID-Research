Project Summary: Failure-Aware TVC Control for Small-Scale Rockets

Student Project Overview
This project investigates how to make small thrust-vector-controlled (TVC) rockets safer and more robust by translating modern fail-aware control methods from large aerospace systems to low-cost model-rocket hardware.

The central idea is to adapt to common degradation modes in real time, such as:
Loss of effective control authority.
Actuator rate/position limitations under load, voltage sag, heating, or wear.
Measurement artifacts from vibration, flex, backlash, or sensor noise that the controller’s safety and adaptation logic must tolerate.

Instead of treating a small TVC rocket as a simple PID-tuning problem, the project aims to build a reproducible control framework that can recognize degraded behavior and adapt safely within measured operating limits.

The current research direction is to measure real actuator-slew degradation and control-effectiveness variation, determine whether those uncertainties materially shrink the stable or acceptable operating region of fixed controllers, and compare those results against a safety-bounded adaptive controller.

Core Research Question
Can a small-rocket TVC controller detect degradation mode online and adapt safely before instability occurs, while remaining practical enough for real amateur and educational use?

Why This Project Matters
Large launch vehicles already use sophisticated robustness, saturation handling, and safety logic, but publicly documented operational systems appear to rely much more on conservative, certifiable control architectures than on openly described online adaptive primary flight control. At the same time, those ideas are rarely translated into small-scale rocketry in a way that is:
1. Hardware-validated.
2. Reproducible.
3. Accessible to non-experts.

Most small-scale TVC projects still rely on lightly tested fixed-gain PID-style control with limited simulation, limited fault analysis, and little understanding of safe operating envelopes.

This project aims to bridge that gap.

Why small rockets are the right niche for this work:
1. They have larger relative uncertainty and weaker actuator margins than large launch systems.
2. They are practical to instrument and test repeatedly.
3. They allow rigorous study of uncertainty-aware control without claiming to replace certified large-launch vehicle flight software.
4. Improvements here could materially improve safety and mission success for amateur and educational TVC systems.

Main Contribution
The project does not claim to invent adaptive control theory from scratch.
Its contribution is a small-scale, hardware-grounded translation of modern fail-aware control ideas, with emphasis on:
1. Online identification of degradation modes.
2. Safety-bounded adaptation rather than unconstrained adaptation.
3. Boundary-level validation showing where the method works and where it fails.
4. Documentation and tooling that could allow others to implement the framework more safely.
5. Measured uncertainty propagation from hardware tests into controller comparison and flight evidence.

Current Technical Direction
The current controller architecture includes:
1. A nominal state-feedback controller for rocket attitude stabilization.
2. Online estimation of effective control authority.
3. Online monitoring of actuator envelope degradation.
4. Arbitration logic when one degradation type can contaminate estimation of another.
5. Safety guardrails such as command limiting, confidence gating, and abort logic.

The hardware path currently uses a model rocket TVC platform with Teensy-based avionics and analog-feedback servos for direct actuator-state measurement.

The experimental path is:
1. Measure actuator slew degradation under unloaded, installed, static-fire, and reduced-voltage conditions.
2. Bound plausible control-effectiveness variation from motor and vehicle uncertainty.
3. Inject those measured uncertainties into simulation.
4. Compare fixed baselines against uncertainty-aware controllers.
5. Validate conclusions on real rocket hardware and flight tests within safe envelopes.

Current Status
Completed
1. Simulation framework with baseline controllers and disturbance/fault sweeps.
2. Initial firmware implementation of the adaptive and safety logic.
3. Early evidence that degradation-aware adaptation can substantially reduce post-fault error in relevant regimes.
4. Bench diagnostics for direct actuator feedback, slew characterization, and endpoint-utilization measurement.

In Progress
1. Calibration and integration of feedback servos.
2. Hardware validation of the estimator and safety logic.
3. Failure-boundary mapping and repeatability analysis.

Potential Real-World Impact
If successful, the project could enable:
1. Safer educational and amateur TVC rocketry.
2. Better preflight controller validation through measured operating-boundary maps.
3. A reusable testbed for evaluating advanced control methods on constrained hardware.
4. Open-source tools and documentation that make modern control methods more accessible.

Who Could Use This Research
1. Amateur and educational TVC builders who currently rely on fixed-gain tuning.
2. University rocket teams and controls labs.
3. Open-source flight-control communities.
4. Other constrained embedded-actuation systems with similar uncertainty patterns.

The deliverable is both a controller approach and a practical methodology for uncertainty-aware validation.

Why Joint-Aware (Slew + keff) Instead of One-Sided Adaptation
1. keff-only adaptation can misread rate-limited actuator behavior as authority loss and adapt in the wrong direction.
2. Slew-only adaptation can miss genuine control-effectiveness drift from thrust or geometry variation.
3. The strongest hypothesis is that joint-aware arbitration is only justified if it measurably outperforms single-channel methods in combined uncertainty conditions.
4. This prevents adding complexity without evidence.

What Kind of Mentorship Would Be Most Helpful
This project would benefit most from mentorship in one or more of the following:
1. Guidance, navigation, and control (GNC).
2. Adaptive or robust control theory.
3. Embedded systems / avionics validation.
4. Experimental design for hardware validation.
5. Aerospace systems engineering or launch-vehicle controls.

Ideal mentor support would include:
1. Reviewing research framing and helping sharpen the central claim.
2. Stress-testing the methodology and validation plan.
3. Advising on how to present the work at a level competitive for Regeneron STS.
4. Helping distinguish what is genuinely novel from what is primarily translation and implementation.

Timeline
The current plan is to:
1. Complete hardware calibration and integration in June.
2. Run controlled baseline and ablation campaigns in July.
3. Build uncertainty and failure-boundary maps in August.
4. Lock hardware-backed claims in September.
5. Finalize the paper and application materials in October for STS submission in early November.

One-Sentence Summary
This project seeks to translate fail-aware control ideas from high-end launch systems and adaptive-control research into small TVC rockets by measuring real uncertainty, testing how much it harms fixed controllers, and proving with hardware where safety-bounded adaptation helps and where it does not.

