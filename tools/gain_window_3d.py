# -*- coding: utf-8 -*-
"""
tools/gain_window_3d.py

3D gain-window visualization.
  X = theta_ddot_max (authority)
  Y = latency_steps
  Z = Kp (the gain being tested)
  Green = SR >= 0.80 (valid gain), Red = SR < 0.80 (fails)

Shows intuitively how the green "window" of working Kp values narrows
as authority increases and as latency increases.
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

# ── Select designs: stratify across td × latency ──────────────────────────────

meta2 = meta.copy()
meta2['td_bin'] = pd.cut(meta2['td'],
    bins=[0, 50, 100, 180, 9999],
    labels=['td<50', 'td 50-100', 'td 100-180', 'td>180'])
meta2['lat_bin'] = pd.cut(meta2['latency_steps'],
    bins=[0, 2, 4, 99],
    labels=['lat 1-2', 'lat 3-4', 'lat 5-6'])

# Pick up to 4 designs per cell, prefer non-censored, widest range of td
selected_ids = []
for (tb, lb), grp in meta2.groupby(['td_bin', 'lat_bin'], observed=True):
    sub = grp[~grp['floor_censored'] & ~grp['ceil_censored']]
    if len(sub) == 0:
        sub = grp
    n = min(4, len(sub))
    picked = sub.sample(n=n, random_state=42)
    selected_ids.extend(picked['rocket_id'].tolist())

selected_ids = list(set(selected_ids))
print(f"Selected {len(selected_ids)} designs")

plot_data = sweep[sweep['rocket_id'].isin(selected_ids)].copy()

# Small jitter on latency so stacked dots are distinguishable
rng = np.random.default_rng(42)
plot_data = plot_data.copy()
plot_data['lat_j'] = (plot_data['latency_steps']
                      + rng.uniform(-0.12, 0.12, len(plot_data)))

pass_df = plot_data[plot_data['pass']]
fail_df = plot_data[~plot_data['pass']]
print(f"Points: {len(pass_df)} pass (green), {len(fail_df)} fail (red)")

# ── Colour fail points by how far outside window they are ─────────────────────
# Use SR directly for alpha: near-miss failures are lighter, hard fails are solid red

# ── Build figure ──────────────────────────────────────────────────────────────

fig = go.Figure()

# --- Red: failing Kp values ---
fig.add_trace(go.Scatter3d(
    x=fail_df['td'],
    y=fail_df['lat_j'],
    z=fail_df['Kp'],
    mode='markers',
    name='Fails (SR < 0.80)',
    marker=dict(
        size=3,
        color='crimson',
        opacity=0.20,
        line=dict(width=0),
    ),
    customdata=np.column_stack([fail_df['latency_steps'], fail_df['sr']]),
    hovertemplate=(
        'td_max: %{x:.0f} rad/s<sup>2</sup><br>'
        'Latency: %{customdata[0]} steps<br>'
        'Kp: %{z:.2f}<br>'
        'SR: %{customdata[1]:.2f}<br>'
        '<extra>FAIL</extra>'
    ),
))

# --- Green: passing Kp values ---
fig.add_trace(go.Scatter3d(
    x=pass_df['td'],
    y=pass_df['lat_j'],
    z=pass_df['Kp'],
    mode='markers',
    name='Passes (SR >= 0.80)',
    marker=dict(
        size=4,
        color='limegreen',
        opacity=0.80,
        line=dict(width=0),
    ),
    customdata=np.column_stack([pass_df['latency_steps'], pass_df['sr']]),
    hovertemplate=(
        'td_max: %{x:.0f} rad/s<sup>2</sup><br>'
        'Latency: %{customdata[0]} steps<br>'
        'Kp: %{z:.2f}<br>'
        'SR: %{customdata[1]:.2f}<br>'
        '<extra>PASS</extra>'
    ),
))

# --- Orange dashed lines: theoretical ceiling Kp_max = 380/latency ---
td_line = np.linspace(4, 500, 80)
for lat in [1, 2, 3, 4, 5, 6]:
    kp_ceil = 380.0 / lat
    fig.add_trace(go.Scatter3d(
        x=td_line,
        y=np.full_like(td_line, lat),
        z=np.full_like(td_line, kp_ceil),
        mode='lines',
        line=dict(color='orange', width=4),
        name=f'Ceiling lat={lat} (Kp={kp_ceil:.0f})',
        showlegend=(lat == 1),
        legendgroup='ceiling',
        hovertemplate=f'Kp ceiling={kp_ceil:.0f} at latency={lat}<extra></extra>',
    ))

# --- Layout ---
fig.update_layout(
    title=dict(
        text=(
            'TVC Gain Window: which Kp values work at each (authority, latency)?<br>'
            '<sup>Green = SR>=0.80 (flies well) | Red = fails | '
            'Orange = theoretical ceiling Kp~380/latency</sup>'
        ),
        font=dict(size=15),
        x=0.5,
    ),
    scene=dict(
        xaxis=dict(
            title=dict(text='Authority  (theta_ddot_max, rad/s^2)', font=dict(size=12)),
            type='log',
            tickvals=[10, 30, 100, 200, 500],
            ticktext=['10', '30', '100', '200', '500'],
            gridcolor='rgba(255,255,255,0.12)',
            backgroundcolor='rgb(20,20,35)',
        ),
        yaxis=dict(
            title=dict(text='Latency (steps at 200 Hz)', font=dict(size=12)),
            tickvals=[1, 2, 3, 4, 5, 6],
            ticktext=['1', '2', '3', '4', '5', '6'],
            gridcolor='rgba(255,255,255,0.12)',
            backgroundcolor='rgb(20,20,35)',
        ),
        zaxis=dict(
            title=dict(text='Kp  (proportional gain)', font=dict(size=12)),
            type='log',
            tickvals=[0.2, 0.5, 1, 3, 10, 30, 100, 300, 800],
            ticktext=['0.2', '0.5', '1', '3', '10', '30', '100', '300', '800'],
            gridcolor='rgba(255,255,255,0.12)',
            backgroundcolor='rgb(20,20,35)',
        ),
        camera=dict(eye=dict(x=1.6, y=-1.7, z=0.9)),
        bgcolor='rgb(12,12,22)',
        aspectmode='manual',
        aspectratio=dict(x=1.4, y=0.9, z=1.2),
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
    margin=dict(l=0, r=0, t=80, b=20),
    annotations=[
        dict(
            text=(
                'The green band = valid gain window  |  '
                'It narrows (vertically) as authority increases (left->right)  |  '
                'And compresses further as latency increases (front->back)'
            ),
            xref='paper', yref='paper',
            x=0.5, y=-0.01,
            xanchor='center', yanchor='top',
            font=dict(size=10, color='rgba(200,200,200,0.9)'),
            showarrow=False,
        )
    ],
)

out_path = OUTPUTS / 'gain_window_3d.html'
fig.write_html(str(out_path), include_plotlyjs='cdn')
print(f"Saved -> {out_path}")
