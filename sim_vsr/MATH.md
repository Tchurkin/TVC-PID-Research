# Variable-Stability Rocket — Control System Derivation

Full math behind `sim_vsr` and the control-mechanism comparison (`tools/vsr_lab.py`,
`vsr_full_comparison.py`, `vsr_animate.py`). Everything here matches the code 1:1.

---

## 1. Frame, state, and kinematics

Planar (pitch-plane) flight. Angles measured **from the horizon**; a straight-up launch is
`θ = γ = 90°`.

State vector: `[x, z, V, γ, θ, q]`
- `x, z` — downrange / altitude [m]
- `V` — airspeed [m/s]
- `γ` — flight-path angle (direction of the velocity vector) [rad]
- `θ` — body pitch attitude [rad]
- `q = θ̇` — pitch rate [rad/s]

Angle of attack:  **α = θ − γ**  (body pointing minus velocity direction).

Position kinematics: `ẋ = V cos γ`, `ż = V sin γ`.

---

## 2. Forces and the equations of motion

Dynamic pressure `q̄ = ½ ρ V²`. Reference area `S = π d²/4`.

**Normal force** (perpendicular to the body, lifting):  `N = q̄ S C_N(α)`
**Drag**:  `D = q̄ S (C_{D0} + K · C_{Nα} α²)`

Translational (point mass with thrust `T` along the body axis):

```
m V̇      =  T cos α − D − m g sin γ
m V γ̇    =  T sin α + N + N_can − m g cos γ
```

Rotational about the CG (`I_yy` = pitch inertia):

```
I_yy q̇  =  M_aero + M_tvc + M_can
θ̇       =  q
```

Mass and inertia vary linearly with remaining propellant fraction `f ∈ [0,1]`:
`m = m_dry + f·m_prop`, `I_yy = I_dry + f·(I_wet − I_dry)`.

---

## 3. Aerodynamics with stall

Linear lift slope below stall, saturating plateau above (`cn_eff` in `vehicle.py`):

```
C_N(angle) =  C_Nα · angle                 , |angle| ≤ α_stall
           =  C_Nα · α_stall · sign(angle)  , |angle| > α_stall   (gentle plateau)
```

`α_stall = None` ⇒ pure-linear legacy aero. The plateau is conservative; a real post-stall
**drop** would be harsher (open caveat in all results).

**Aft fins** (fixed, provide baseline stability). With static margin `sm_aft` [calibers]:

```
M_aero = − N · (sm_aft · d) = − q̄ S C_Nα α · (sm_aft d)        (restoring for sm_aft>0)
```

**Forward canard** (deployable, fraction `δ_dep ∈ [0,1]`, deflection `δ`, arm `ℓ_c` ahead of CG):

```
N_can = q̄ (S_c δ_dep) · C_N,c(α + δ)        (stall acts on the LOCAL angle α+δ)
M_can = N_can · ℓ_c                          (forward ⇒ nose-up for positive force)
```

The canard force splits into an **α-part** (∝ α, a stability/stiffness effect) and a
**δ-part** (∝ δ, a direct control force):  `C_N,c(α+δ) ≈ C_Nα,c α + C_Nα,c δ` (linear regime).

---

## 4. Static margin and the two-sided neutral-centered geometry

The **neutral point** is where the *α-derivative* of the pitching moment acts. Its location is
set by the lift-slope-weighted geometry, and it is **independent of deflection δ** (δ adds a
constant moment, which differentiates to zero). Total α-moment slope:

```
dM/dα = − q̄ S C_Nα (sm_aft d)  +  q̄ (S_c δ_dep) C_N,c ℓ_c
        └─── aft fins (stabilizing) ──┘   └── canard (destabilizing, ∝ deploy) ──┘
```

Define the **full-deploy margin swing** (margin removed by fully deploying the canard):

```
ΔΠ_m  =  (S_c C_N,c ℓ_c) / (S C_Nα d)         ( = CanardParams.dmargin_full )
```

The **net static margin** in calibers:

```
SM(δ_dep) = sm_aft − ΔΠ_m · δ_dep
```

**Two-sided neutral-centered design** (the build: static aft fins + forward canards that
*cancel* at half-deploy). Choose `sm_aft = ½ ΔΠ_m`. Then:

```
δ_dep = 0    → SM = +½ΔΠ_m   (stable)
δ_dep = 0.5  → SM = 0        (NEUTRAL)
δ_dep = 1    → SM = −½ΔΠ_m   (unstable, equal & opposite)
```

This symmetry is essential: it gives the controller **two-sided authority** about neutral —
it can actively *over-stabilize* (retract, `δ_dep<0.5`) to damp, and *destabilize* (extend,
`δ_dep>0.5`) to fling. A one-sided canard (retracted = max-stable) cannot damp the catch and
**fails** (verified — that was an early setup error).

---

## 5. The three control mechanisms

All use the **same matched control law** — a feedback-linearizing desired angular acceleration

```
a_des = K_p (θ_ref − θ) − K_d q ,   with K_p = ω_n², K_d = 2ζω_n   (ω_n=6, ζ≈0.67)
M_des = I_yy · a_des
```

Each mechanism then inverts its own actuator to deliver `M_des`:

| mechanism | moment produced | inversion (controller output) | works at α=0? | needs thrust? |
|---|---|---|---|---|
| **TVC** | `M = T sin δ_g · L` | `δ_g = arcsin(M_des / (T L))` | yes | **yes** |
| **Canard deflection** | `M = q̄ S_c δ_dep C_N,c δ · ℓ_c` | `δ = M_des /(q̄ S_c δ_dep C_N,c ℓ_c)` | yes | no |
| **Margin modulation** | `M = K_net(δ_dep)·α` (see below) | `δ_dep = 0.5 + M_des /(q̄ S_c C_N,c ℓ_c · α)` | **no (singular)** | no |

where the margin-modulation net α-moment is

```
M = K_net(δ_dep) · α ,   K_net(δ_dep) = q̄ S_c C_N,c ℓ_c (δ_dep − 0.5)      (with sm_aft=½ΔΠ_m)
```

### Why margin modulation is fundamentally different (bilinear / parametric)

TVC and canard-deflection are **additive**: the control input enters the dynamics as an
*added moment*, independent of the state. Margin modulation is **parametric / bilinear**: the
control input `δ_dep` *multiplies the state* `α`:

```
I_yy θ̈ = q̄ S_c C_N,c ℓ_c (δ_dep − 0.5) · α  + (additive terms)
                       └──── control × state ────┘
```

You are not pushing on the rocket; you are **changing the sign of its aerodynamic stiffness**
(a negative-spring "fling", then a positive-spring/neutral "catch"). Two consequences:
- **Singular at α = 0** — zero authority when there is no angle of attack (start of the
  maneuver and final fine-hold). A small TVC backstop covers this regime.
- The maneuver completes at **high α** (the body rotates ~5× faster than the velocity vector
  turns: `τ_γ/τ_rot ≈ 5`), so margin keeps authority through the bulk of the slew; α→0 only
  afterward as γ catches up.

A pure spring cannot dissipate energy, but **bang-bang stiffness switching is still rest-to-rest**:
fling (negative SM, hyperbolic growth) → catch (positive SM, decelerate to the zero-rate
turnaround) → freeze (neutral SM). Damping is *synthesized* by modulating SM out of phase with
rate (`δ_dep − 0.5 ∝ −q/α`), valid while α is bounded away from 0.

---

## 6. Moment decomposition (additive vs parametric)

At every step the pitching moment splits into four measurable pieces (`vsr_full_comparison`,
the ChatGPT-proposed experiment):

```
M_total = M_aft,α + M_can,α + M_can,δ + M_tvc
          │         │          │         └ thrust vectoring (additive)
          │         │          └ canard DEFLECTION lift (additive "canard control")
          │         └ canard α-coupling  (PARAMETRIC "stability modulation")
          └ aft-fin restoring (∝ α)
```

Measured for margin modulation (δ=0): **M_can,δ = 0%**, the maneuver is driven by the
α-coupling — i.e. genuinely stability modulation, not hidden canard control.

---

## 7. The Π authority–latency parameter

`k_eff = T L / I_yy` is the angular-acceleration gain [rad/s² per rad gimbal]; per **control
unit**, `k_eff,CU = k_eff · (π/180 · 15/12)`. With control-loop latency `τ = lat·dt`:

```
Π = k_eff,CU · lat²      (dimensionless, in the project's CU scale; Π_crit ≈ 275)
```

Physical meaning: angular displacement accumulated per control unit during one latency window —
the "blind-spot" the controller cannot correct. The bang-bang blind-spot impulse
`A_blind = ½ k_eff u_max τ² ∝ Π` predicts the saturation-regime onset and the agility/
controllability envelope. (This is the project's core result and is **independent of the
mechanism** above — see CLAUDE.md.)

---

## 8. Why margin modulation is stall-robust (the real comparison result)

The canard's lift saturates when its **local** angle exceeds stall:

- **Canard deflection control** commands large `δ`, so the local angle `α + δ` blows through
  `α_stall`; `C_N,c` plateaus, the inversion `δ = M_des/(…)` loses its assumed gain, and control
  degrades / reverses. (Measured: 83% → and at hard stall **0%** success, overshoot 95°.)
- **Margin modulation** runs `δ = 0`, so the canard's local angle is just `α` — it stalls far
  less, and the controller compensates for capped force by **deploying more** (longer arm).
  (Measured: stays 100% at the 30°/stall-15° case where deflection drops to 83%.)
- **TVC** is **not aerodynamic at all**, so it cannot stall — it is the most robust of the three
  *whenever thrust is present*; it is dead in coast.

### Honest comparison summary (matched control law, `vsr_full_comparison.py`)

| regime | TVC | canard deflection | margin modulation |
|---|---|---|---|
| benign air | ✅ clean | ✅ **cleanest** | ✅ (slightly slower settle) |
| stalling air, moderate | ✅ (immune) | ⚠️ degrades (83%) | ✅ robust (100%) |
| stalling air, aggressive | ✅ (immune, needs thrust) | ❌ diverges | ⚠️ degrades but bounded |
| **coast (no thrust)** | ❌ **dead** | ⚠️ stall-prone | ✅ **best available** |

**Conclusion.** Margin modulation is *not* a general winner — in calm air conventional canard
deflection is cleaner, and TVC is the most stall-robust whenever the motor is burning. Its
genuine, mechanism-backed niche is **aerodynamic control in the high-α / stall regime,
especially in coast where TVC is unavailable**: there it keeps a control surface unstalled by
steering through the airframe's stability instead of a deflected (stalling) surface. Combined
with plausible novelty (triangulated; nearest prior art = moving-mass control, which shifts CG
in the *stable* regime), that is the defensible contribution — with the Π envelope as the
overarching, mechanism-independent result.

### Open caveats (do not omit when citing)
- Stall model is a gentle plateau; a realistic post-stall **CN drop** could shrink the margin
  advantage and must be re-tested.
- α and q must be **estimated in flight** for the FL controllers (flow vane / accel-based /
  observer) — the real hardware barrier; TVC needs neither.
- Novelty is "plausibly novel, pending a DTIC/AIAA full-text search," not confirmed.
