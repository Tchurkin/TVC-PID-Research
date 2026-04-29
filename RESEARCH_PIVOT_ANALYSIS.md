# Research Pivot Analysis: STS-Finalist ADRC Program

**Date:** April 27, 2026  
**Status:** Actionable blueprint with mathematical core, stress tests, and HIL metrics

---

## Finalist-Level Objective

The target is not "controller works". The target is:

1. A mathematically grounded controller/observer design for nonlinear uncertain rocket dynamics.
2. A quantifiable robustness gain over PID/LQR baselines.
3. A computational-efficiency argument for embedded hardware.

Primary claim to prove:

**A discrete-time ADRC with tuned observer bandwidth achieves materially higher stability under aerodynamic uncertainty and gust disturbances, while remaining feasible on resource-constrained flight computers.**

---

## Core Gap and Novelty

### Gap
Existing small-rocket TVC work commonly does one of two things:

1. Assumes sufficiently accurate aerodynamic models.
2. Uses PID with limited robustness to model error and nonlinearity.

### Proposed Novelty

1. Treat unknown aero/structural/wind terms as one total disturbance state and estimate it online via DLESO.
2. Co-design observer bandwidth and servo-delay constraints in discrete time.
3. Quantify the Pareto frontier between robustness and compute cost (loop rate, utilization, cycle time).

---

## Mathematical Core: DLESO-Based ADRC

Use a second-order pitch channel with augmented disturbance:

$$
\begin{aligned}
\dot{x}_1 &= x_2 \\
\dot{x}_2 &= b_0 u + f(t) \\
\dot{x}_3 &= \dot{f}(t)
\end{aligned}
\qquad x_3 = f(t)
$$

In compact form:

$$
\begin{bmatrix}
\dot{x}_1 \\
\dot{x}_2 \\
\dot{x}_3
\end{bmatrix}
=
\begin{bmatrix}
0 & 1 & 0 \\
0 & 0 & 1 \\
0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
x_1 \\
x_2 \\
x_3
\end{bmatrix}
+
\begin{bmatrix}
0 \\
b_0 \\
0
\end{bmatrix}u
+
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}\dot{f}
$$

Observer update (discrete-time Euler implementation in simulation):

$$
\begin{aligned}
e_k &= z_{1,k} - y_k \\
z_{1,k+1} &= z_{1,k} + T_s( z_{2,k} - \beta_1 e_k ) \\
z_{2,k+1} &= z_{2,k} + T_s( z_{3,k} + b_0 u_k - \beta_2 e_k ) \\
z_{3,k+1} &= z_{3,k} + T_s( -\beta_3 e_k )
\end{aligned}
$$

Bandwidth parameterization:

$$
\beta_1=3\omega_o,\quad \beta_2=3\omega_o^2,\quad \beta_3=\omega_o^3
$$

Control law:

$$
u_k = \frac{v_k - z_{3,k}}{b_0},
\qquad
v_k = -k_p(z_{1,k}-r_k) - k_d z_{2,k} - k_i\int(z_{1}-r)dt
$$

---

## Key Research Questions (Testable)

1. **Observer convergence vs sensor noise floor:**
How does DLESO disturbance-estimation error scale with MPU-6050/BNO055-equivalent noise?

2. **Bandwidth-delay coupling law:**
For a servo delay range of 40-50 ms, what observer bandwidth range preserves stability margin and avoids noise amplification?

3. **Nonlinearity robustness:**
How do rate limit, saturation, and backlash shift the stability boundary for PID, LQR-like baseline, and ADRC?

4. **Embedded feasibility:**
What is the compute-cost trade between loop frequency and robustness (Pareto frontier)?

---

## High-Fidelity Disturbance and Actuator Modeling

### Disturbance Model

1. Baseline gusts: deterministic step/ramp for controlled stress tests.
2. Stochastic turbulence: Dryden/Von Karman-style filtered-noise process.
3. Parametric aero uncertainty: CoP/CP shift, damping variation, dynamic-pressure scaling.

### Actuator Nonlinearity

1. Hard saturation ($\pm\delta_{max}$).
2. Rate limiting ($|\dot{\delta}|\leq\dot{\delta}_{max}$).
3. Backlash/deadband.

This is required to claim practical robustness, not just ideal-model performance.

---

## Comparative Stress-Test Matrix

Controllers to compare:

1. PID baseline.
2. LQR-like linear baseline (or robust linear baseline).
3. ADRC with DLESO (proposed).

Required stress dimensions:

1. CoP/CP uncertainty: 0% to 30%.
2. Wind intensity: 0% to 25% (normalized).
3. Sensor noise floor: low/medium/high.
4. Servo delay: 30 ms to 60 ms.

Primary outcomes:

1. Stability success rate.
2. RMS attitude and rate error.
3. Peak command and saturation time fraction.
4. Disturbance-estimation RMS error.
5. Control compute time per loop and feasible loop frequency.

---

## HIL Validation Program

Simulation-only is acceptable only with unusually deep theory. HIL substantially increases project strength.

HIL architecture:

1. Plant and disturbances run in MATLAB.
2. Flight computer (Teensy/ESP32) computes control in real time.
3. MATLAB and MCU exchange state/command at fixed loop rate.

Record:

1. Loop jitter (us).
2. CPU utilization proxy and cycle budget.
3. Control deadline misses.
4. Robustness metrics under same stress-test matrix.

Finalist-grade output: complexity-robustness Pareto frontier with explicit operating envelope.

---

## Success Criteria (Quantifiable)

The project should aim to show a table of this form with confidence intervals:

1. PID loses stability near 10% CoP uncertainty.
2. Linear baseline loses stability near 15% gust-equivalent disturbance.
3. ADRC maintains stability substantially farther into uncertainty/disturbance space.

Exact percentages must come from your data, not assumptions.

---

## Immediate Implementation Plan

### Week 1: ADRC Core and Benchmark Harness

1. Implement DLESO in discrete time.
2. Add disturbance process and actuator nonlinearity block.
3. Build PID vs ADRC Monte Carlo benchmark scripts.

### Week 2: Bandwidth and Delay Study

1. Sweep observer bandwidth.
2. Sweep servo delay/noise floors.
3. Derive practical tuning law region for your hardware class.

### Week 3: Stress-Test Campaign

1. Run full uncertainty/gust matrix.
2. Generate robustness boundary plots.
3. Quantify effect sizes and confidence intervals.

### Week 4+: HIL Integration

1. Connect MATLAB plant loop with MCU control loop.
2. Measure timing and jitter.
3. Add compute-constrained tuning recommendations.

---

## References to Prioritize

1. Jingqing Han: Active Disturbance Rejection Control foundational papers/books.
2. ADRC stability analyses with Lyapunov framing for nonlinear systems.
3. 2025-2026 NASA technical memoranda on adaptive launch-vehicle control.
4. Recent model-rocket TVC experimental papers for actuator realism.

---

## Decision

Proceed with ADRC+DLESO as primary research thread, and treat previous delay-vs-aero study as supporting context for actuator/latency constraints.
