# -*- coding: utf-8 -*-
"""
tools/gain_window_3d_v2.py  --  FIXED 3D gain-window visualization

Key improvements:
  1. Z axis normalized to Kp / (380 / latency). Ceiling is a flat plane at Z=1.
  2. X axis is keff (NOT theta_ddot_max). keff = T*motor_scale*CU_RAD*l/Iyy is the
     mechanistically correct parameter — ceiling is keff-independent, floor rises with
     keff (not with theta_ddot_max which also includes max_gimbal, an extraneous factor).

What you see:
  - Orange plane at Z = 1:  theoretical ceiling (Kp_max = 380/latency); keff-independent
  - Green band:  valid Kp range (SR >= 0.80); bottom of band = floor (rises with keff)
  - Red points:  failing Kp values (too high = above ceiling, or too low = below floor)
  - As keff increases (left -> right): the green band's BOTTOM creeps UP toward Z = 1,
    squeezing the window (the "double squeeze").
  - At high latency (front -> back): BOTH floor and ceiling compress further.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
import plotly.graph_objects as go

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'
OUTPUTS = ROOT / 'outputs'

SR_PASS = 0.80

# ── Load data ─────────────────────────────────────────────────────────────────

meta  = pd.read_csv(RESULTS / 'window_ratio_v2_py.csv')
sweep = pd.read_csv(RESULTS / 'window_ratio_v2_sweep_py.csv')

sweep = sweep.merge(
    meta[['rocket_id', 'td', 'keff', 'latency_steps', 'servo_slew',
          'kp_floor', 'kp_ceiling', 'window_ratio', 'floor_censored', 'ceil_censored']],
    on='rocket_id', how='left'
)
sweep['pass'] = sweep['sr'] >= SR_PASS

# Normalize Z: Kp relative to the theoretical ceiling for this latency level
sweep['Kp_norm'] = sweep['Kp'] * sweep['latency_steps'] / 380.0

# ── Select designs: stratify across td × latency ──────────────────────────────
# Pick non-censored designs spanning the full td and latency range

meta2 = meta.copy()
meta2['keff_bin'] = pd.cut(meta2['keff'],
    bins=[0, 5, 10, 18, 9999],
    labels=['keff<5', 'keff 5-10', 'keff 10-18', 'keff>18'])
meta2['lat_bin'] = pd.cut(meta2['latency_steps'],
    bins=[0, 2, 4, 99],
    labels=['lat 1-2', 'lat 3-4', 'lat 5-6'])

selected_ids = []
for (tb, lb), grp in meta2.groupby(['keff_bin', 'lat_bin'], observed=True):
    sub = grp[~grp['floor_censored'] & ~grp['ceil_censored']]
    if len(sub) == 0:
        sub = grp
    n = min(4, len(sub))
    picked = sub.sample(n=n, random_state=42)
    selected_ids.extend(picked['rocket_id'].tolist())

selected_ids = list(set(selected_ids))
print(f"Selected {len(selected_ids)} designs across td x latency bins")

plot_data = sweep[sweep['rocket_id'].isin(selected_ids)].copy()

# Small jitter on latency so stacked designs are distinguishable
rng = np.random.default_rng(42)
plot_data['lat_j'] = (plot_data['latency_steps']
                      + rng.uniform(-0.12, 0.12, len(plot_data)))

pass_df = plot_data[plot_data['pass']]
fail_df = plot_data[~plot_data['pass']]
print(f"Points: {len(pass_df)} pass (green), {len(fail_df)} fail (red)")

# ── Build figure ──────────────────────────────────────────────────────────────

fig = go.Figure()

# --- Red: failing Kp values (both below-floor and above-ceiling failures) ---
# Color code: above-ceiling failures (Kp_norm > 1) are brighter red; below-floor (< 0.05) are darker
ceil_fail = fail_df[fail_df['Kp_norm'] > 1.0]
floor_fail = fail_df[fail_df['Kp_norm'] <= 1.0]

fig.add_trace(go.Scatter3d(
    x=ceil_fail['keff'],
    y=ceil_fail['lat_j'],
    z=ceil_fail['Kp_norm'],
    mode='markers',
    name='Fails: Kp too HIGH (above ceiling)',
    marker=dict(size=3, color='orangered', opacity=0.25, line=dict(width=0)),
    customdata=np.column_stack([ceil_fail['latency_steps'], ceil_fail['sr'],
                                ceil_fail['Kp']]),
    hovertemplate=(
        'keff: %{x:.2f} rad/s2/CU<br>'
        'Latency: %{customdata[0]:.0f} steps<br>'
        'Kp: %{customdata[2]:.1f}  (norm: %{z:.2f})<br>'
        'SR: %{customdata[1]:.2f}<br>'
        '<extra>ABOVE CEILING</extra>'
    ),
))

fig.add_trace(go.Scatter3d(
    x=floor_fail['keff'],
    y=floor_fail['lat_j'],
    z=floor_fail['Kp_norm'],
    mode='markers',
    name='Fails: Kp too LOW (below floor)',
    marker=dict(size=3, color='firebrick', opacity=0.20, line=dict(width=0)),
    customdata=np.column_stack([floor_fail['latency_steps'], floor_fail['sr'],
                                floor_fail['Kp']]),
    hovertemplate=(
        'keff: %{x:.2f} rad/s2/CU<br>'
        'Latency: %{customdata[0]:.0f} steps<br>'
        'Kp: %{customdata[2]:.1f}  (norm: %{z:.2f})<br>'
        'SR: %{customdata[1]:.2f}<br>'
        '<extra>BELOW FLOOR</extra>'
    ),
))

# --- Green: passing Kp values (inside the window) ---
fig.add_trace(go.Scatter3d(
    x=pass_df['keff'],
    y=pass_df['lat_j'],
    z=pass_df['Kp_norm'],
    mode='markers',
    name='Passes (SR >= 0.80)',
    marker=dict(size=4, color='limegreen', opacity=0.85, line=dict(width=0)),
    customdata=np.column_stack([pass_df['latency_steps'], pass_df['sr'],
                                pass_df['Kp']]),
    hovertemplate=(
        'keff: %{x:.2f} rad/s2/CU<br>'
        'Latency: %{customdata[0]:.0f} steps<br>'
        'Kp: %{customdata[2]:.1f}  (norm: %{z:.2f})<br>'
        'SR: %{customdata[1]:.2f}<br>'
        '<extra>PASS</extra>'
    ),
))

# --- Orange horizontal plane at Z = 1: the normalized ceiling ---
# Show it as a flat surface across (td, latency) space
keff_vals = np.linspace(1, 30, 40)   # keff range for ceiling surface
lat_vals  = np.linspace(1, 6, 40)
TD_grid, LAT_grid = np.meshgrid(keff_vals, lat_vals)
Z_ceil = np.ones_like(TD_grid)

fig.add_trace(go.Surface(
    x=TD_grid,
    y=LAT_grid,
    z=Z_ceil,
    colorscale=[[0, 'rgba(255,140,0,0.28)'], [1, 'rgba(255,140,0,0.28)']],
    showscale=False,
    name='Theoretical ceiling (Kp = 380/latency)',
    hovertemplate='Theoretical ceiling: Kp = 380 / latency<extra></extra>',
    showlegend=True,
    legendgroup='ceiling',
))

# --- Layout ---
fig.update_layout(
    title=dict(
        text=(
            'TVC Gain Window (Normalized): Kp / (380/latency) vs keff vs Latency<br>'
            '<sup>Z=1 = theoretical ceiling (keff-independent)  |  Green = working gains  |  '
            'Floor (bottom of green) rises with keff — ceiling does not</sup>'
        ),
        font=dict(size=14),
        x=0.5,
    ),
    scene=dict(
        xaxis=dict(
            title=dict(text='keff [rad/s²/CU] = T × motor_scale × 0.02182 × l / Iyy', font=dict(size=11)),
            type='log',
            tickvals=[2, 5, 10, 20, 30],
            ticktext=['2', '5', '10', '20', '30'],
            gridcolor='rgba(255,255,255,0.12)',
            backgroundcolor='rgb(20,20,35)',
        ),
        yaxis=dict(
            title=dict(text='Latency (steps at 200 Hz)', font=dict(size=11)),
            tickvals=[1, 2, 3, 4, 5, 6],
            ticktext=['1', '2', '3', '4', '5', '6'],
            gridcolor='rgba(255,255,255,0.12)',
            backgroundcolor='rgb(20,20,35)',
        ),
        zaxis=dict(
            title=dict(text='Kp / (380/latency) — normalized gain', font=dict(size=11)),
            range=[0, 2.5],
            tickvals=[0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5],
            ticktext=['0', '0.25', '0.5', '0.75', '1.0 (ceiling)', '1.5', '2.0', '2.5'],
            gridcolor='rgba(255,255,255,0.12)',
            backgroundcolor='rgb(20,20,35)',
        ),
        camera=dict(eye=dict(x=1.5, y=-1.8, z=0.9)),
        bgcolor='rgb(12,12,22)',
        aspectmode='manual',
        aspectratio=dict(x=1.4, y=0.9, z=1.0),
    ),
    paper_bgcolor='rgb(12,12,22)',
    plot_bgcolor='rgb(12,12,22)',
    font=dict(color='white', family='Arial', size=12),
    legend=dict(
        x=0.01, y=0.99,
        bgcolor='rgba(0,0,0,0.5)',
        bordercolor='rgba(255,255,255,0.2)',
        borderwidth=1,
        font=dict(size=11),
    ),
    margin=dict(l=0, r=0, t=100, b=20),
    annotations=[
        dict(
            text=(
                'Ceiling (orange plane at Z=1) is keff-INDEPENDENT — it only depends on latency.  '
                'The gain FLOOR (bottom of green band) rises with keff.  '
                'Window = floor-to-ceiling gap — shrinks as keff or latency increases.'
            ),
            xref='paper', yref='paper',
            x=0.5, y=-0.01,
            xanchor='center', yanchor='top',
            font=dict(size=10, color='rgba(200,200,200,0.9)'),
            showarrow=False,
        )
    ],
)

out_path = OUTPUTS / 'gain_window_3d_v3.html'
fig.write_html(str(out_path), include_plotlyjs='cdn')
print(f"Saved -> {out_path}")
