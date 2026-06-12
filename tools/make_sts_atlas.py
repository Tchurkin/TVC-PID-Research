"""
Regenerate atlas_3d_interactive.html with regime classification story.
New story: FRAGILE is MECHANICAL (theta_ddot predicts, not wind).
Axes: theta_ddot_max x wind_strength x Iyy with axis-swap dropdowns.
"""
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from pathlib import Path

# ── Load data ──────────────────────────────────────────────────────────────────
exp1 = pd.read_csv('experiments/results/exp1_regime_index_py.csv')
s2r  = pd.read_csv('experiments/results/exp4_s2r_gains_py.csv')

F15_T_AVG, L_NOZZLE = 14.4, 0.25
CU_TO_RAD = (np.pi/180) * (15/12)
exp1['keff_full']      = F15_T_AVG * exp1['motor_scale'] * CU_TO_RAD * L_NOZZLE / exp1['Iyy']
exp1['u_max']          = exp1['max_gimbal_deg'] * 12/15
exp1['theta_ddot_max'] = exp1['keff_full'] * exp1['u_max']

exp1 = exp1.merge(
    s2r[['rocket_id','kp_simple','kp_full','sr_simple_in_full','false_rejection','false_approval']],
    on='rocket_id', how='left'
)

REGIME_COLORS  = {'EASY': '#2ecc71', 'FRAGILE': '#e74c3c', 'MARGINAL': '#e67e22', 'INFEASIBLE': '#8e44ad'}
REGIME_SIZES   = {'EASY': 4, 'FRAGILE': 12, 'MARGINAL': 9, 'INFEASIBLE': 14}
REGIME_SYMBOLS = {'EASY': 'circle', 'FRAGILE': 'diamond', 'MARGINAL': 'square', 'INFEASIBLE': 'cross'}
ORDER = ['EASY', 'MARGINAL', 'FRAGILE', 'INFEASIBLE']

def _hover(row):
    fr_tag = ""
    if row.false_rejection:
        fr_tag = " WARNING: FALSE REJECTION"
    elif row.false_approval:
        fr_tag = " WARNING: FALSE APPROVAL"
    else:
        fr_tag = " OK"
    ks = row.kp_simple if not np.isnan(row.kp_simple) else 0
    kf = row.kp_full   if not np.isnan(row.kp_full)   else 0
    return (
        f"<b>{row.rocket_id}</b>  [{row.regime_label}]<br>"
        f"theta_ddot_max = {row.theta_ddot_max:.1f} rad/s^2<br>"
        f"Iyy = {row.Iyy:.4f} kg-m^2<br>"
        f"wind = {row.wind_strength:.3f}<br>"
        f"static_margin = {row.static_margin:.2f} cal<br>"
        f"max_gimbal = {row.max_gimbal_deg:.1f} deg<br>"
        f"motor_scale = {row.motor_scale:.2f}<br>"
        f"keff_full = {row.keff_full:.2f} rad/s^2/CU<br>"
        f"best_Kp = {row.best_Kp:.1f}  nom_sr = {row.nominal_success_rate:.2f}<br>"
        f"S2R: Kp_simple={ks:.1f} vs Kp_full={kf:.1f}{fr_tag}"
    )

# ── Build traces ───────────────────────────────────────────────────────────────
traces = []
for rname in ORDER:
    sub = exp1[exp1['regime_label'] == rname].copy()
    hover = [_hover(r) for _, r in sub.iterrows()]
    traces.append(go.Scatter3d(
        x=sub['theta_ddot_max'].tolist(),
        y=sub['wind_strength'].tolist(),
        z=sub['Iyy'].tolist(),
        mode='markers',
        name=f"{rname} (n={len(sub)})",
        marker=dict(
            size=REGIME_SIZES[rname],
            color=REGIME_COLORS[rname],
            symbol=REGIME_SYMBOLS[rname],
            opacity=0.60 if rname == 'EASY' else 0.92,
            line=dict(color='black', width=0.4 if rname == 'EASY' else 0.9),
        ),
        hovertext=hover, hoverinfo='text',
    ))

# Threshold plane at theta_ddot = 70
wmin, wmax = float(exp1.wind_strength.min()), float(exp1.wind_strength.max())
imin, imax = float(exp1.Iyy.min()), float(exp1.Iyy.max())
traces.append(go.Surface(
    x=[[70,70],[70,70]],
    y=[[wmin,wmax],[wmin,wmax]],
    z=[[imin,imin],[imax,imax]],
    showscale=False,
    colorscale=[[0,'rgba(255,200,0,0.22)'],[1,'rgba(255,200,0,0.22)']],
    hoverinfo='skip',
    name='theta_ddot=70 threshold',
    showlegend=True,
    opacity=0.28,
))

# ── Axis-swap dropdowns ────────────────────────────────────────────────────────
AXIS_OPTIONS = [
    ('theta_ddot_max', 'theta_ddot_max (rad/s^2)'),
    ('wind_strength',  'Wind strength'),
    ('Iyy',            'Iyy (kg-m^2)'),
    ('static_margin',  'Static margin (cal)'),
    ('max_gimbal_deg', 'Max gimbal (deg)'),
    ('keff_full',      'keff_full (rad/s^2/CU)'),
    ('best_Kp',        'Best Kp (full physics)'),
]

def make_axis_btns(scene_xyz):
    btns = []
    for col, label in AXIS_OPTIONS:
        new_vals = []
        for rname in ORDER:
            sub = exp1[exp1['regime_label'] == rname]
            new_vals.append(sub[col].tolist())
        new_vals.append([None])  # threshold surface — unchanged
        btns.append(dict(
            label=label.split(' ')[0],
            method='restyle',
            args=[{scene_xyz: new_vals}],
        ))
    return btns

R1, LOFF = 0.975, 0.030
layout = go.Layout(
    paper_bgcolor='rgb(250,250,253)',
    title=dict(
        text=(
            "Regime Atlas: theta_ddot_max x Wind x Iyy  "
            "(Finding 2: FRAGILE is a mechanical property, not wind-driven)<br>"
            "<sub>n=1200 LHS designs  |  "
            "FRAGILE=red diamonds  EASY=green circles  MARGINAL=orange squares  INFEASIBLE=purple cross  |  "
            "Yellow plane: theta_ddot=70 rad/s^2 threshold  (AUC=0.855)</sub>"
        ),
        font=dict(size=11, color='#222'), x=0.5, xanchor='center', y=0.990,
    ),
    scene=dict(
        domain=dict(x=[0.0, 1.0], y=[0.0, 0.80]),
        xaxis=dict(title='theta_ddot_max (rad/s^2)',
                   backgroundcolor='rgb(240,245,255)', gridcolor='#cdd', showbackground=True),
        yaxis=dict(
            title='Wind strength',
            tickvals=[0.05, 0.15, 0.25, 0.35, 0.45],
            ticktext=['0.05 calm','0.15 light','0.25 mod.','0.35 gusty','0.45 severe'],
            backgroundcolor='rgb(240,255,240)', gridcolor='#cdd', showbackground=True,
        ),
        zaxis=dict(title='Iyy (kg-m^2)',
                   backgroundcolor='rgb(255,245,240)', gridcolor='#cdd', showbackground=True),
        camera=dict(eye=dict(x=1.55, y=-1.45, z=0.85)),
    ),
    updatemenus=[
        dict(buttons=make_axis_btns('x'), direction='down', showactive=True,
             x=0.00, y=R1, xanchor='left', yanchor='top',
             bgcolor='white', bordercolor='#ccc', borderwidth=1,
             font=dict(size=10, color='#222'), pad=dict(t=3,b=3,l=4,r=4)),
        dict(buttons=make_axis_btns('y'), direction='down', showactive=True,
             x=0.32, y=R1, xanchor='left', yanchor='top',
             bgcolor='white', bordercolor='#ccc', borderwidth=1,
             font=dict(size=10, color='#222'), pad=dict(t=3,b=3,l=4,r=4)),
        dict(buttons=make_axis_btns('z'), direction='down', showactive=True,
             x=0.64, y=R1, xanchor='left', yanchor='top',
             bgcolor='white', bordercolor='#ccc', borderwidth=1,
             font=dict(size=10, color='#222'), pad=dict(t=3,b=3,l=4,r=4)),
    ],
    annotations=[
        dict(text='<b>X axis</b>', x=0.00, y=R1+LOFF, xref='paper', yref='paper',
             showarrow=False, font=dict(size=10, color='#444'), xanchor='left'),
        dict(text='<b>Y axis</b>', x=0.32, y=R1+LOFF, xref='paper', yref='paper',
             showarrow=False, font=dict(size=10, color='#444'), xanchor='left'),
        dict(text='<b>Z axis</b>', x=0.64, y=R1+LOFF, xref='paper', yref='paper',
             showarrow=False, font=dict(size=10, color='#444'), xanchor='left'),
        dict(
            text=(
                'circle=EASY  diamond=FRAGILE  square=MARGINAL  x=INFEASIBLE  |  '
                'Key finding: FRAGILE designs (red) do NOT cluster at high wind  --  '
                'r(wind, FRAGILE) = -0.001  =>  gain sensitivity is a HARDWARE property'
            ),
            x=0.5, y=0.824, xref='paper', yref='paper', showarrow=False,
            font=dict(size=9, color='#666'), xanchor='center',
        ),
    ],
    legend=dict(
        x=0.01, y=0.78,
        bgcolor='rgba(255,255,255,0.88)', bordercolor='#ccc', borderwidth=1,
        font=dict(size=10),
    ),
    margin=dict(l=10, r=10, b=10, t=30),
    width=1120, height=820,
)

fig = go.Figure(data=traces, layout=layout)
out = Path('outputs/atlas_figures/pd/requirement_atlas/atlas_3d_interactive.html')
out.parent.mkdir(parents=True, exist_ok=True)
fig.write_html(
    str(out), include_plotlyjs='cdn',
    config={'toImageButtonOptions': {'format': 'png', 'width': 1200, 'height': 850}},
)
print(f"Saved atlas: {out}  ({out.stat().st_size/1024:.0f} KB)")

# Also save a copy alongside the other STS-gold files
out2 = Path('outputs/sts_gold_0_atlas_3d.html')
fig.write_html(str(out2), include_plotlyjs='cdn')
print(f"Saved copy: {out2}")
