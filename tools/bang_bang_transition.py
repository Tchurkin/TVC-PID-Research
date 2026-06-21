"""
tools/bang_bang_transition.py  --  Visualize the PID → bang-bang transition at Kp_ceiling.

The gain ceiling Kp_max ≈ 380/latency predicts the Kp at which proportional control
transitions from linear to bang-bang (servo permanently saturated).

This is the visual/empirical companion to the DIPDT ceiling derivation:
  As Kp increases through [Kp_floor, Kp_ceiling], slew_sat_frac goes from ~0 to ~1.
  At Kp_ceiling, the servo is saturated nearly every timestep → bang-bang dynamics.

Method: 4 representative designs at latency=3 (15ms), spanning td=[10, 80, 200, 400].
  For each: sweep Kp from Kp_floor/4 to Kp_ceiling×2.5 (20 log-spaced points).
  At each Kp: 5 seeds, measure SR, RMS, slew_sat_frac.
  Kd = opt_Kd (frozen; this is a Kp-only sweep to show the transition).

Output:
  experiments/results/bang_bang_transition_py.csv
  outputs/bang_bang_transition.html
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from design_space import build_plant, build_actuator, build_sensor, build_disturbance, build_scenario
from controller import PIDParams
from simulator import simulate
from fidelity_config import FidelityConfig, apply_fidelity_config

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'
OUTPUTS = ROOT / 'outputs'

# ── Select designs ────────────────────────────────────────────────────────────
wr = pd.read_csv(RESULTS / 'window_ratio_v2_py.csv')
wr_valid = wr[
    ~wr['floor_censored'] & ~wr['ceil_censored'] &
    wr['window_ratio'].notna() & (wr['window_ratio'] > 0)
].copy()

# Want 4 designs at lat=3, spanning td = 10 / 80 / 200 / 400 rad/s²
TARGET_TD   = [15, 80, 200, 400]
TARGET_LAT  = 3
N_DESIGNS   = 4

selected = []
for td_target in TARGET_TD:
    sub = wr_valid[wr_valid['latency_steps'] == TARGET_LAT].copy()
    if len(sub) == 0:
        # fall back to lat=2 or 4
        sub = wr_valid[wr_valid['latency_steps'].isin([2, 3, 4])].copy()
    sub['_dist'] = (sub['td'] - td_target).abs()
    row = sub.sort_values('_dist').iloc[0]
    selected.append(row)
    print(f"td_target={td_target}: {row['rocket_id']}  td={row['td']:.0f}  lat={int(row['latency_steps'])}  "
          f"floor={row['kp_floor']:.1f}  ceil={row['kp_ceiling']:.1f}  window={row['window_ratio']:.0f}x")


def build_physics(row_dict):
    fid = FidelityConfig(
        wind=True, sensor_noise=True, slew=True, backlash=True,
        latency=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )
    plant       = build_plant(row_dict)
    actuator    = build_actuator(row_dict)
    sensor      = build_sensor(row_dict)
    disturbance = build_disturbance(row_dict)
    scenario    = build_scenario(theta0_bias_std=0.0)
    actuator, sensor, disturbance, scenario = apply_fidelity_config(
        actuator, sensor, disturbance, scenario, fid)
    return plant, actuator, sensor, disturbance, scenario


# ── Run sweep ─────────────────────────────────────────────────────────────────
SEEDS      = [1, 3, 7, 11, 17]
N_KP       = 24

records = []
for row in selected:
    d       = row.to_dict()
    kd      = float(row['opt_Kd'])
    floor   = float(row['kp_floor'])
    ceiling = float(row['kp_ceiling'])

    kp_lo = max(0.3, floor / 4.0)
    kp_hi = min(800, ceiling * 2.5)
    kp_grid = np.geomspace(kp_lo, kp_hi, N_KP)

    print(f"\n{row['rocket_id']}  td={row['td']:.0f}  lat={int(row['latency_steps'])}  "
          f"kd={kd}  sweep [{kp_lo:.1f}, {kp_hi:.1f}]")

    for kp in kp_grid:
        sr_vals   = []
        slew_vals = []
        rms_vals  = []
        for s in SEEDS:
            pid = PIDParams(Kp=float(kp), Kd=kd, Ki=0.0)
            plant, act, sen, dis, sc = build_physics(d)
            res = simulate(pid, plant, act, sen, dis, sc, seed=s)
            sr_vals.append(float(res.success))
            slew_vals.append(res.slew_sat_frac)
            rms_vals.append(res.rms_error_deg)

        records.append({
            'rocket_id':    row['rocket_id'],
            'td':           row['td'],
            'keff':         row['keff'],
            'latency':      int(row['latency_steps']),
            'kp_floor':     floor,
            'kp_ceiling':   ceiling,
            'window_ratio': row['window_ratio'],
            'Kp':           kp,
            'kp_norm':      kp / ceiling,          # normalized to ceiling
            'sr_mean':      np.mean(sr_vals),
            'slew_frac':    np.mean(slew_vals),
            'rms_mean':     np.mean(rms_vals),
        })
        print(f"  Kp={kp:6.1f} ({kp/ceiling:.2f}×ceil)  SR={np.mean(sr_vals):.2f}  "
              f"slew={np.mean(slew_vals):.2f}  rms={np.mean(rms_vals):.1f}°")

df = pd.DataFrame(records)
df.to_csv(RESULTS / 'bang_bang_transition_py.csv', index=False)
print(f"\nSaved data -> {RESULTS / 'bang_bang_transition_py.csv'}")


# ── Figure ────────────────────────────────────────────────────────────────────
designs = df['rocket_id'].unique()
COLORS  = ['#44dd88', '#55aaff', '#ffbb33', '#ff5544']
LABEL_MAP = {r['rocket_id']: f"td={r['td']:.0f} lat={int(r['latency'])}"
             for _, r in df.groupby('rocket_id').first().reset_index().iterrows()}

fig = make_subplots(
    rows=2, cols=1,
    shared_xaxes=True,
    subplot_titles=[
        'Success Rate (SR ≥ 0.80 = PASS)',
        'Slew Saturation Fraction (1.0 = bang-bang)'
    ],
    vertical_spacing=0.08,
)

for i, (rid, color) in enumerate(zip(designs, COLORS)):
    sub = df[df['rocket_id'] == rid].sort_values('kp_norm')
    label = LABEL_MAP[rid]
    floor_norm   = float(sub['kp_floor'].iloc[0]   / sub['kp_ceiling'].iloc[0])
    window       = float(sub['window_ratio'].iloc[0])

    # SR trace
    fig.add_trace(go.Scatter(
        x=sub['kp_norm'], y=sub['sr_mean'],
        mode='lines+markers',
        name=f"{label} (win={window:.0f}×)",
        line=dict(color=color, width=2),
        marker=dict(size=5),
        showlegend=True,
        legendgroup=rid,
    ), row=1, col=1)

    # Slew_sat trace
    fig.add_trace(go.Scatter(
        x=sub['kp_norm'], y=sub['slew_frac'],
        mode='lines+markers',
        name=f"{label} slew",
        line=dict(color=color, width=2, dash='dash'),
        marker=dict(size=4, symbol='diamond'),
        showlegend=False,
        legendgroup=rid,
    ), row=2, col=1)

    # Vertical lines for floor and ceiling in kp_norm
    for row_idx in [1, 2]:
        fig.add_vline(x=floor_norm, row=row_idx, col=1,
                      line=dict(color=color, width=1, dash='dot'), opacity=0.5)
        fig.add_vline(x=1.0, row=row_idx, col=1,
                      line=dict(color=color, width=1, dash='dash'), opacity=0.4)

# Reference lines
fig.add_hline(y=0.80, row=1, col=1, line=dict(color='rgba(255,255,100,0.5)', width=1.5, dash='dash'))
fig.add_hline(y=1.00, row=2, col=1, line=dict(color='rgba(255,100,100,0.3)', width=1, dash='dot'))

# Annotations
fig.add_annotation(
    x=0.98, y=0.82, xref='x', yref='y',
    text='SR = 0.80 (pass threshold)',
    font=dict(size=9, color='rgba(255,255,100,0.8)'),
    showarrow=False, row=1, col=1,
)
fig.add_annotation(
    x=1.05, y=0.15, xref='x2', yref='y2',
    text='← Kp_ceiling (norm=1.0) | slew approaches 1.0 here',
    font=dict(size=9, color='rgba(200,200,200,0.7)'),
    showarrow=False,
)

fig.update_layout(
    title=dict(
        text=(
            'PID → Bang-Bang Transition at Kp_ceiling<br>'
            '<sup>X axis normalized: 1.0 = Kp_ceiling for each design  |  '
            'At ceiling, slew saturation fraction → 1.0 (servo permanently maxed out)  |  '
            'Left vertical dotted lines = Kp_floor  |  '
            'Each line: latency = 3 steps (15 ms at 200 Hz)</sup>'
        ),
        x=0.5, font=dict(size=13),
    ),
    height=680,
    paper_bgcolor='rgb(10,10,20)',
    plot_bgcolor='rgb(10,10,20)',
    font=dict(color='rgba(220,220,240,0.9)', family='Arial', size=11),
    legend=dict(
        x=1.02, y=0.99,
        bgcolor='rgba(0,0,0,0.55)',
        bordercolor='rgba(255,255,255,0.15)',
        borderwidth=1,
    ),
    margin=dict(l=60, r=180, t=110, b=50),
)
for r in [1, 2]:
    fig.update_xaxes(
        title_text='Kp / Kp_ceiling (normalized)' if r == 2 else '',
        type='log',
        gridcolor='rgba(255,255,255,0.07)',
        row=r, col=1,
    )
    fig.update_yaxes(
        gridcolor='rgba(255,255,255,0.07)',
        zerolinecolor='rgba(255,255,255,0.15)',
        row=r, col=1,
    )
fig.update_yaxes(title_text='SR (5-seed mean)', range=[-0.05, 1.1], row=1, col=1)
fig.update_yaxes(title_text='Slew sat frac', range=[-0.05, 1.1], row=2, col=1)

out = OUTPUTS / 'bang_bang_transition.html'
fig.write_html(str(out), include_plotlyjs='cdn')
print(f"Saved figure -> {out}")

# ── Print the key numbers ─────────────────────────────────────────────────────
print("\n" + "=" * 60)
print("SLEW SATURATION AT Kp = Kp_ceiling (norm=1.0):")
print("=" * 60)
for rid in designs:
    sub = df[df['rocket_id'] == rid]
    # Find row closest to kp_norm = 1.0
    closest = sub.iloc[(sub['kp_norm'] - 1.0).abs().argsort().iloc[0]]
    print(f"  {rid}  td={closest['td']:.0f}  slew@ceiling={closest['slew_frac']:.2f}  "
          f"SR@ceiling={closest['sr_mean']:.2f}")
