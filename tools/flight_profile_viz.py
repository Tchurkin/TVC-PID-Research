"""
tools/flight_profile_viz.py  --  Visual sanity check: flight profiles across the design space.

Picks 6 representative designs spanning (td, latency):
  1. Low td, low latency   — wide window, easy
  2. Low td, high latency  — latency-compressed ceiling
  3. Med td, low latency   — moderate window
  4. Med td, high latency  — tight window, near-FRAGILE
  5. High td, low latency  — high authority, aggressive
  6. High td, high latency — FRAGILE territory

For each design: runs 3 seeds at (a) mid-window Kp and (b) Kp=2 (probe).
Plots theta (deg), q (deg/s), u_act vs time in 6×2 subplot grid.
Output: outputs/flight_profiles.html
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

# ── Load window_ratio dataset ─────────────────────────────────────────────────
wr = pd.read_csv(RESULTS / 'window_ratio_v2_py.csv')
wr_valid = wr[~wr['floor_censored'] & ~wr['ceil_censored']].copy()
wr_valid['mid_kp'] = np.sqrt(wr_valid['kp_floor'] * wr_valid['kp_ceiling'])

# ── Select 6 representative designs ──────────────────────────────────────────
DESIGNS_SPEC = [
    # (label, td_lo, td_hi, lat_lo, lat_hi)
    ('Low td / Low lat',   0,   60, 1, 2),
    ('Low td / High lat',  0,   60, 5, 6),
    ('Med td / Low lat',   80, 160, 1, 2),
    ('Med td / High lat',  80, 160, 5, 6),
    ('High td / Low lat', 200, 600, 1, 2),
    ('High td / High lat',200, 600, 5, 6),
]

selected = []
for (label, td_lo, td_hi, lat_lo, lat_hi) in DESIGNS_SPEC:
    subset = wr_valid[
        (wr_valid['td'] >= td_lo) & (wr_valid['td'] < td_hi) &
        (wr_valid['latency_steps'] >= lat_lo) & (wr_valid['latency_steps'] <= lat_hi)
    ]
    if len(subset) == 0:
        # Fall back: closest td, any latency in range
        subset = wr_valid[
            (wr_valid['latency_steps'] >= lat_lo) & (wr_valid['latency_steps'] <= lat_hi)
        ].copy()
        if len(subset) == 0:
            subset = wr_valid.copy()
        subset = subset.copy()
        subset['_dist'] = (subset['td'] - (td_lo + td_hi) / 2).abs()
        subset = subset.sort_values('_dist')
    design = subset.sort_values('window_ratio').iloc[len(subset) // 2]  # median window
    selected.append((label, design))
    print(f"{label}: {design['rocket_id']}  td={design['td']:.0f}  lat={int(design['latency_steps'])}  "
          f"window={design['window_ratio']:.0f}x  kp_floor={design['kp_floor']:.1f}  "
          f"kp_ceil={design['kp_ceiling']:.1f}  mid_kp={design['mid_kp']:.1f}")


# ── Simulation helpers ────────────────────────────────────────────────────────
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


def run_one(row, kp, seed):
    d = row.to_dict()
    kd  = float(row['opt_Kd'])
    pid = PIDParams(Kp=float(kp), Kd=kd, Ki=0.0)
    plant, act, sen, dis, sc = build_physics(d)
    return simulate(pid, plant, act, sen, dis, sc, seed=seed)


# ── Run simulations ───────────────────────────────────────────────────────────
print("\nRunning simulations...")
PROBE_KP = 2.0
SEEDS    = [1, 5, 13]

sim_data = []
for label, row in selected:
    kp_mid   = float(row['mid_kp'])
    kp_floor = float(row['kp_floor'])
    kp_ceil  = float(row['kp_ceiling'])
    print(f"  {label}: Kp_mid={kp_mid:.1f}", end='', flush=True)

    best_sims  = [run_one(row, kp_mid,   s) for s in SEEDS]
    probe_sims = [run_one(row, PROBE_KP, s) for s in SEEDS]
    print(f"  SR_best={np.mean([r.success for r in best_sims]):.2f}  "
          f"SR_probe={np.mean([r.success for r in probe_sims]):.2f}")

    sim_data.append({
        'label': label, 'row': row,
        'kp_mid': kp_mid, 'kp_floor': kp_floor, 'kp_ceil': kp_ceil,
        'best': best_sims, 'probe': probe_sims,
    })

print("Done.\n")


# ── Build figure ──────────────────────────────────────────────────────────────
# 6 rows × 2 cols; each cell shows theta, q (scaled), u_act (scaled) for 3 seeds
N = len(sim_data)

# Build column headers (subtitle text embedded in first trace names)
fig = make_subplots(
    rows=N, cols=2,
    vertical_spacing=0.035,
    horizontal_spacing=0.06,
    column_titles=[
        'Best gains (mid-window Kp)',
        'Probe flight (Kp = 2)',
    ],
    row_titles=[d['label'] for d in sim_data],
)

SEED_ALPHA = [1.0, 0.6, 0.35]
C_BEST  = '#44dd88'   # green
C_PROBE = '#ff5544'   # red
C_QDOT  = '#88bbff'   # blue
C_UACT  = '#ffcc44'   # amber


def add_row(fig, results, row_idx, col_idx, kp_val, c_theta):
    for si, (res, alpha) in enumerate(zip(results, SEED_ALPHA)):
        t      = res.t
        theta  = np.degrees(res.theta_true)      # radians → degrees
        qdeg   = np.degrees(res.q_true)           # rad/s → deg/s
        u_act  = res.u_act                        # code units (typically ±12)

        sr_lbl = 'PASS' if res.success else 'FAIL'
        name   = f"seed {SEEDS[si]} ({sr_lbl})"

        # theta — main line
        fig.add_trace(go.Scatter(
            x=t, y=theta,
            mode='lines',
            name=name,
            line=dict(color=c_theta, width=2.0 if si == 0 else 1.2),
            opacity=alpha,
            showlegend=(si == 0 and row_idx == 1),
            legendgroup='theta',
            hovertemplate='t=%{x:.2f}s  θ=%{y:.1f}°<extra></extra>',
        ), row=row_idx, col=col_idx)

        # theta_dot — scaled to ±20 for overlay visibility
        q_scale = 20.0 / max(np.percentile(np.abs(qdeg), 98), 1.0)
        fig.add_trace(go.Scatter(
            x=t, y=qdeg * q_scale,
            mode='lines',
            name='θ̇ (scaled)',
            line=dict(color=C_QDOT, width=1.0, dash='dash'),
            opacity=alpha * 0.6,
            showlegend=(si == 0 and row_idx == 1 and col_idx == 1),
            legendgroup='qdot',
            hovertemplate='t=%{x:.2f}s  θ̇=%{y:.1f} (scaled)<extra></extra>',
        ), row=row_idx, col=col_idx)

        # u_act — servo command
        u_scale = 15.0 / max(np.percentile(np.abs(u_act), 98), 0.1)
        fig.add_trace(go.Scatter(
            x=t, y=u_act * u_scale,
            mode='lines',
            name='u_act (scaled)',
            line=dict(color=C_UACT, width=0.8, dash='dot'),
            opacity=alpha * 0.45,
            showlegend=(si == 0 and row_idx == 1 and col_idx == 1),
            legendgroup='uact',
            hovertemplate='t=%{x:.2f}s  u=%{y:.1f} (scaled)<extra></extra>',
        ), row=row_idx, col=col_idx)


for i, d in enumerate(sim_data):
    ri = i + 1
    add_row(fig, d['best'],  ri, 1, d['kp_mid'],  C_BEST)
    add_row(fig, d['probe'], ri, 2, PROBE_KP,     C_PROBE)

    # Reference lines ±15° (warn) and ±70° (fail)
    for ci in [1, 2]:
        fig.add_hline(y= 15,  row=ri, col=ci, line=dict(color='rgba(255,200,0,0.35)',  width=1, dash='dash'))
        fig.add_hline(y=-15,  row=ri, col=ci, line=dict(color='rgba(255,200,0,0.35)',  width=1, dash='dash'))
        fig.add_hline(y= 70,  row=ri, col=ci, line=dict(color='rgba(255,80,0,0.25)',   width=1, dash='dot'))
        fig.add_hline(y=-70,  row=ri, col=ci, line=dict(color='rgba(255,80,0,0.25)',   width=1, dash='dot'))

# Axis styling
for i in range(1, N + 1):
    for ci in [1, 2]:
        fig.update_yaxes(
            range=[-80, 80],
            gridcolor='rgba(255,255,255,0.07)',
            zerolinecolor='rgba(255,255,255,0.2)',
            title_text='θ (°)' if ci == 1 else '',
            row=i, col=ci,
        )
        fig.update_xaxes(
            gridcolor='rgba(255,255,255,0.07)',
            title_text='time (s)' if i == N else '',
            row=i, col=ci,
        )

# Build annotations for design info (one per row, left panel)
annotations = []
for i, d in enumerate(sim_data):
    ri = i + 1
    # Compute mean SR for labeling
    sr_b = np.mean([r.success for r in d['best']])
    sr_p = np.mean([r.success for r in d['probe']])
    rms_b = np.mean([r.rms_error_deg for r in d['best']])

    # Left panel info box
    info_text = (
        f"{d['row']['rocket_id']} | td={d['row']['td']:.0f} lat={int(d['row']['latency_steps'])}<br>"
        f"keff={d['row']['keff']:.1f} win={d['row']['window_ratio']:.0f}x<br>"
        f"Kp={d['kp_mid']:.0f} [floor={d['kp_floor']:.0f}..ceil={d['kp_ceil']:.0f}]<br>"
        f"SR={sr_b:.2f} RMS={rms_b:.1f}°"
    )
    # Right panel SR
    probe_text = f"SR@Kp=2: {sr_p:.2f}"

    annotations.append(dict(
        xref=f'x{(i*2+1) if i>0 else ""} domain', yref=f'y{(i*2+1) if i>0 else ""} domain',
        x=0.02, y=0.97,
        text=info_text,
        xanchor='left', yanchor='top',
        font=dict(size=8, color='rgba(200,220,255,0.9)'),
        showarrow=False, bgcolor='rgba(0,0,0,0.45)', bordercolor='rgba(255,255,255,0.1)',
    ))
    annotations.append(dict(
        xref=f'x{(i*2+2)} domain', yref=f'y{(i*2+2)} domain',
        x=0.02, y=0.97,
        text=probe_text,
        xanchor='left', yanchor='top',
        font=dict(size=9, color='rgba(255,200,150,0.9)'),
        showarrow=False, bgcolor='rgba(0,0,0,0.45)', bordercolor='rgba(255,255,255,0.1)',
    ))

fig.update_layout(
    title=dict(
        text=(
            'TVC Flight Profiles — Sanity Check Across Design Space<br>'
            '<sup>Green = best gains (mid-window Kp) | Red = Kp=2 probe | '
            'Dashed blue = θ̇ (scaled) | Dotted amber = servo u_act (scaled) | '
            'Yellow dashed = ±15° | Orange dotted = ±70° fail limit</sup>'
        ),
        x=0.5, font=dict(size=13),
    ),
    height=230 * N + 120,
    paper_bgcolor='rgb(10,10,20)',
    plot_bgcolor='rgb(10,10,20)',
    font=dict(color='rgba(220,220,240,0.9)', family='Arial', size=10),
    legend=dict(
        x=1.02, y=0.99,
        bgcolor='rgba(0,0,0,0.55)',
        bordercolor='rgba(255,255,255,0.15)',
        borderwidth=1,
        font=dict(size=10),
        traceorder='normal',
    ),
    margin=dict(l=60, r=180, t=100, b=50),
    annotations=annotations,
)

out_path = OUTPUTS / 'flight_profiles.html'
fig.write_html(str(out_path), include_plotlyjs='cdn')
print(f"Saved -> {out_path}")

# ── Console summary ───────────────────────────────────────────────────────────
print("\n" + "=" * 72)
print(f"{'Design':<24}  {'td':>5}  {'lat':>3}  {'win':>6}  {'SR_best':>8}  {'SR_p2':>7}  {'RMS_b':>7}  {'slew':>5}")
print("-" * 72)
for d in sim_data:
    sr_b   = np.mean([r.success      for r in d['best']])
    sr_p   = np.mean([r.success      for r in d['probe']])
    rms_b  = np.mean([r.rms_error_deg for r in d['best']])
    slew_b = np.mean([r.slew_sat_frac for r in d['best']])
    print(f"{d['label']:<24}  {d['row']['td']:5.0f}  {int(d['row']['latency_steps']):3d}  "
          f"{d['row']['window_ratio']:6.0f}  {sr_b:8.2f}  {sr_p:7.2f}  {rms_b:7.1f}°  {slew_b:5.2f}")
