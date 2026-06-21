"""
tools/regen_sts_gold_suite.py

Regenerate STS-gold visualization suite (figures 0-3, 5-6) from n=2400 data.
Figure 4 (flight detection) is regenerated separately after flight_sig_rerun_n45.py.

Outputs:
  outputs/sts_gold_0_atlas_3d.html     — 3D regime atlas
  outputs/sts_gold_1_regime_scatter.html — theta_ddot vs regime strip plot
  outputs/sts_gold_2_gain_windows.html  — Kp sweep SR curves (relay study)
  outputs/sts_gold_3_s2r_mismatch.html  — Kp_simple vs Kp_full scatter
  outputs/sts_gold_5_authority_trap.html — FRAGILE in theta_ddot x latency space
  outputs/sts_gold_6_product_distribution.html — theta_ddot x latency product by regime
  outputs/sts_gold_index.html           — landing page
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
from pathlib import Path

OUT = Path('outputs')
OUT.mkdir(exist_ok=True)

F15_T_AVG, L_NOZZLE = 14.4, 0.25
CU_TO_RAD = (np.pi / 180) * (15 / 12)

COLORS  = {'EASY': '#2ecc71', 'FRAGILE': '#e74c3c', 'MARGINAL': '#e67e22', 'INFEASIBLE': '#8e44ad'}
SIZES   = {'EASY': 4, 'FRAGILE': 12, 'MARGINAL': 10, 'INFEASIBLE': 14}
SYMBOLS = {'EASY': 'circle', 'FRAGILE': 'diamond', 'MARGINAL': 'square', 'INFEASIBLE': 'cross'}
ORDER   = ['EASY', 'MARGINAL', 'FRAGILE', 'INFEASIBLE']

# ── Load data ─────────────────────────────────────────────────────────────────
print("Loading data...")
# Use the twice-corrected final population (n=36 FRAGILE, AUC=0.975, Cohen d=3.71).
# final_label is aliased to regime_label so all downstream filtering still works.
exp1 = pd.read_csv('experiments/results/exp1_final_population_py.csv')
exp1['regime_label'] = exp1['final_label']  # EASY=2362, FRAGILE=36, INFEASIBLE=2
s2r  = pd.read_csv('experiments/results/exp4_s2r_gains_py.csv')

exp1['keff_full'] = F15_T_AVG * exp1['motor_scale'] * CU_TO_RAD * L_NOZZLE / exp1['Iyy']
exp1['u_max']     = exp1['max_gimbal_deg'] * 12.0 / 15.0
exp1['td_max']    = exp1['keff_full'] * exp1['u_max']
exp1['log_td']    = np.log(exp1['td_max'].clip(lower=0.01))
exp1['td_x_lat']  = exp1['td_max'] * exp1['latency_steps']
exp1['log_td_lat']= np.log(exp1['td_x_lat'].clip(lower=0.01))

exp1 = exp1.merge(
    s2r[['rocket_id','kp_simple','kp_full','sr_simple_in_full','false_rejection','false_approval']],
    on='rocket_id', how='left'
)

n_total = len(exp1)
n_frag  = (exp1['regime_label'] == 'FRAGILE').sum()
n_easy  = (exp1['regime_label'] == 'EASY').sum()
print(f"  n={n_total}: EASY={n_easy}, FRAGILE={n_frag}")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 0 — 3D Regime Atlas
# ═══════════════════════════════════════════════════════════════════════════════
print("\nFigure 0: 3D Regime Atlas (theta_ddot x Wind x Iyy)...")

AXIS_OPTS = [
    ('td_max',       'θ̈_max (rad/s²)'),
    ('wind_strength','Wind strength'),
    ('Iyy',          'Iyy (kg·m²)'),
    ('static_margin','Static margin (cal)'),
    ('max_gimbal_deg','Max gimbal (deg)'),
    ('keff_full',    'keff_full (rad/s²/CU)'),
    ('latency_steps','Latency steps'),
]

traces0 = []
for rname in ORDER:
    sub = exp1[exp1['regime_label'] == rname].copy()
    if len(sub) == 0:
        continue
    hover = [
        f"<b>{r.get('rocket_id', r.get('design_id','?'))}</b> [{rname}]<br>"
        f"θ̈_max={r.td_max:.1f}  Iyy={r.Iyy:.4f}  wind={r.wind_strength:.3f}<br>"
        f"gimbal={r.max_gimbal_deg:.1f}°  lat={r.latency_steps}  Kp={r.best_Kp:.1f}"
        for _, r in sub.iterrows()
    ]
    traces0.append(go.Scatter3d(
        x=sub['td_max'].tolist(), y=sub['wind_strength'].tolist(), z=sub['Iyy'].tolist(),
        mode='markers',
        name=f"{rname} (n={len(sub)})",
        marker=dict(size=SIZES[rname], color=COLORS[rname], symbol=SYMBOLS[rname],
                    opacity=0.55 if rname=='EASY' else 0.92,
                    line=dict(color='black', width=0.3 if rname=='EASY' else 0.8)),
        hovertext=hover, hoverinfo='text',
    ))

# Threshold plane at Youden-J = 54.8 rad/s²
wmin, wmax = float(exp1.wind_strength.min()), float(exp1.wind_strength.max())
imin, imax = float(exp1.Iyy.min()), float(exp1.Iyy.max())
THRESH = 54.8
traces0.append(go.Surface(
    x=[[THRESH, THRESH],[THRESH, THRESH]],
    y=[[wmin, wmax],[wmin, wmax]],
    z=[[imin, imin],[imax, imax]],
    showscale=False, name=f'θ̈_max={THRESH} threshold (Youden-J)', showlegend=True,
    colorscale=[[0,'rgba(255,200,0,0.22)'],[1,'rgba(255,200,0,0.22)']],
    hoverinfo='skip', opacity=0.30,
))

def _axis_btns(scene_xyz):
    btns = []
    for col, label in AXIS_OPTS:
        new_vals = [exp1[exp1['regime_label']==rn][col].tolist() for rn in ORDER if len(exp1[exp1['regime_label']==rn])>0]
        new_vals.append([None])  # surface
        btns.append(dict(label=label.split(' ')[0], method='restyle', args=[{scene_xyz: new_vals}]))
    return btns

fig0 = go.Figure(data=traces0)
fig0.update_layout(
    title=dict(
        text=f"Regime Atlas: θ̈_max × Wind × Iyy  (n={n_total}, AUC=0.975 final pop)<br>"
             f"<sub>FRAGILE is a HARDWARE property — r(wind, FRAGILE)=−0.008  |  "
             f"Yellow plane: θ̈_max = {THRESH} rad/s² (Youden-J threshold)</sub>",
        font=dict(size=11), x=0.5, xanchor='center'),
    scene=dict(
        xaxis=dict(title='θ̈_max (rad/s²)', backgroundcolor='rgb(240,245,255)', gridcolor='#cdd', showbackground=True),
        yaxis=dict(title='Wind strength', backgroundcolor='rgb(240,255,240)', gridcolor='#cdd', showbackground=True),
        zaxis=dict(title='Iyy (kg·m²)', backgroundcolor='rgb(255,245,240)', gridcolor='#cdd', showbackground=True),
        camera=dict(eye=dict(x=1.55, y=-1.45, z=0.85)),
    ),
    updatemenus=[
        dict(buttons=_axis_btns('x'), direction='down', showactive=True, x=0.00, y=0.975,
             xanchor='left', yanchor='top', bgcolor='white', bordercolor='#ccc', font=dict(size=10)),
        dict(buttons=_axis_btns('y'), direction='down', showactive=True, x=0.32, y=0.975,
             xanchor='left', yanchor='top', bgcolor='white', bordercolor='#ccc', font=dict(size=10)),
        dict(buttons=_axis_btns('z'), direction='down', showactive=True, x=0.64, y=0.975,
             xanchor='left', yanchor='top', bgcolor='white', bordercolor='#ccc', font=dict(size=10)),
    ],
    annotations=[
        dict(text='<b>X axis</b>', x=0.00, y=1.008, xref='paper', yref='paper', showarrow=False, font=dict(size=10)),
        dict(text='<b>Y axis</b>', x=0.32, y=1.008, xref='paper', yref='paper', showarrow=False, font=dict(size=10)),
        dict(text='<b>Z axis</b>', x=0.64, y=1.008, xref='paper', yref='paper', showarrow=False, font=dict(size=10)),
    ],
    legend=dict(x=0.01, y=0.78, bgcolor='rgba(255,255,255,0.88)', bordercolor='#ccc', borderwidth=1, font=dict(size=10)),
    margin=dict(l=10, r=10, b=10, t=60), width=1120, height=820,
    paper_bgcolor='rgb(250,250,253)',
)
f0 = OUT / 'sts_gold_0_atlas_3d.html'
fig0.write_html(str(f0), include_plotlyjs='cdn')
print(f"  Saved: {f0}  ({f0.stat().st_size/1024:.0f} KB)")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 1 — θ̈_max distribution by regime (violin + strip)
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 1: theta_ddot_max regime scatter...")

fig1 = go.Figure()
for rname in ['EASY', 'FRAGILE']:
    sub = exp1[exp1['regime_label'] == rname]
    fig1.add_trace(go.Violin(
        y=sub['td_max'], name=f"{rname} (n={len(sub)})",
        box_visible=True, meanline_visible=True,
        points='outliers', jitter=0.05, pointpos=-1.5,
        marker=dict(color=COLORS[rname], size=3, opacity=0.5),
        line_color=COLORS[rname], fillcolor=COLORS[rname],
        opacity=0.7, hoveron='violins+points',
        hovertemplate=f'{rname}<br>θ̈_max=%{{y:.1f}} rad/s²<extra></extra>',
    ))

# Threshold line
fig1.add_hline(y=54.8, line_dash='dash', line_color='#e67e22', line_width=2,
    annotation_text='Youden-J = 54.8 rad/s² (TPR=0.96, FPR=0.12)',
    annotation_position='top right', annotation_font=dict(size=10, color='#e67e22'))

frag_td = exp1.loc[exp1['regime_label']=='FRAGILE','td_max']
easy_td = exp1.loc[exp1['regime_label']=='EASY','td_max']
ann1 = (
    f"<b>θ̈_max = T·sin(δ)·L/Iyy</b><br>"
    f"n={n_total} designs ({n_frag} FRAGILE)<br><br>"
    f"<b>FRAGILE:</b> mean={frag_td.mean():.1f} rad/s²<br>"
    f"              median={frag_td.median():.1f}<br>"
    f"              range=[{frag_td.min():.1f}, {frag_td.max():.1f}]<br><br>"
    f"<b>EASY:</b>    mean={easy_td.mean():.1f} rad/s²<br>"
    f"              median={easy_td.median():.1f}<br><br>"
    f"Ratio: {frag_td.mean()/easy_td.mean():.1f}×<br>"
    f"Cohen d=3.71, p=3.6e-13<br><br>"
    f"<b>AUC = 0.975 (final pop, n=36)</b><br>"
    f"(orig 3-seed: AUC=0.944, n=45)"
)
fig1.update_layout(
    title=dict(
        text=f"Finding 2: θ̈_max predicts gain sensitivity (AUC=0.975 final, n={n_total})<br>"
             f"<sub>FRAGILE mean={frag_td.mean():.1f} vs EASY mean={easy_td.mean():.1f} rad/s²  |  "
             f"Cohen d=3.71, p=3.6×10⁻¹³  |  Formula from hardware specs alone</sub>",
        x=0.5, xanchor='center', font=dict(size=12)),
    yaxis=dict(title='θ̈_max = T·sin(δ_max)·L_nozzle / Iyy  (rad/s²)', type='log'),
    xaxis=dict(title='Regime'),
    paper_bgcolor='rgb(252,252,255)', plot_bgcolor='rgb(248,248,255)',
    width=700, height=580,
    legend=dict(x=0.01, y=0.99, bgcolor='rgba(255,255,255,0.88)', bordercolor='#ccc', borderwidth=1),
    annotations=[dict(text=ann1, x=0.98, y=0.98, xref='paper', yref='paper', showarrow=False,
                      align='left', font=dict(size=10), bgcolor='rgba(255,255,220,0.92)',
                      bordercolor='#aaa', borderwidth=1, xanchor='right', yanchor='top')],
)
f1 = OUT / 'sts_gold_1_regime_scatter.html'
fig1.write_html(str(f1), include_plotlyjs='cdn')
print(f"  Saved: {f1}  ({f1.stat().st_size/1024:.0f} KB)")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 2 — Gain window sweep (from relay_easy_comparison_py.csv)
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 2: Gain windows...")

sweep_csv = Path('experiments/results/kp_window_sweep_v2_py.csv')
if sweep_csv.exists():
    sweep = pd.read_csv(sweep_csv)
    td_col = 'theta_ddot' if 'theta_ddot' in sweep.columns else 'theta_ddot_max'

    fig2 = go.Figure()
    frag_rids = sweep[sweep['regime'] == 'FRAGILE']['rocket_id'].unique()
    easy_rids  = sweep[sweep['regime'] == 'EASY']['rocket_id'].unique()

    for rid in sorted(easy_rids):
        sub = sweep[sweep['rocket_id'] == rid].sort_values('Kp')
        td_val = sub[td_col].iloc[0] if td_col in sub.columns else 0
        fig2.add_trace(go.Scatter(
            x=sub['Kp'], y=sub['sr'], mode='lines',
            name=f'EASY {rid} (td={td_val:.0f})',
            line=dict(color='#2ecc71', width=1.5), opacity=0.7,
            hovertemplate=f'{rid}<br>Kp=%{{x}}<br>SR=%{{y:.2f}}<extra></extra>',
        ))

    for rid in sorted(frag_rids):
        sub = sweep[sweep['rocket_id'] == rid].sort_values('Kp')
        td_val = sub[td_col].iloc[0] if td_col in sub.columns else 0
        fig2.add_trace(go.Scatter(
            x=sub['Kp'], y=sub['sr'], mode='lines+markers',
            name=f'FRAGILE {rid} (td={td_val:.0f})',
            line=dict(color='#e74c3c', width=2.5),
            marker=dict(size=5),
            hovertemplate=f'{rid}<br>Kp=%{{x}}<br>SR=%{{y:.2f}}<extra></extra>',
        ))

    fig2.add_hline(y=0.80, line_dash='dot', line_color='#999',
        annotation_text='SR=0.80 threshold', annotation_position='bottom right',
        annotation_font=dict(size=9, color='#666'))

    fig2.update_layout(
        title=dict(
            text="Finding 2 mechanism: FRAGILE designs have narrow gain windows<br>"
                 "<sub>SR vs Kp curves — green=EASY (wide window), red=FRAGILE (narrow window)  |  "
                 "FRAGILE window collapses when gain ceiling ≈ wind-rejection floor</sub>",
            x=0.5, xanchor='center', font=dict(size=12)),
        xaxis=dict(title='Proportional gain Kp', type='log'),
        yaxis=dict(title='Success rate (3-seed)', range=[-0.05, 1.05]),
        paper_bgcolor='rgb(252,252,255)', plot_bgcolor='rgb(248,248,255)',
        width=820, height=520,
        legend=dict(x=1.01, y=1.0, font=dict(size=9)),
    )
    f2 = OUT / 'sts_gold_2_gain_windows.html'
    fig2.write_html(str(f2), include_plotlyjs='cdn')
    print(f"  Saved: {f2}  ({f2.stat().st_size/1024:.0f} KB)")
else:
    print(f"  SKIP: {sweep_csv} not found")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 3 — S2R mismatch scatter (Kp_simple vs Kp_full)
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 3: S2R mismatch...")

fig3 = go.Figure()
for rname in ['EASY', 'FRAGILE', 'MARGINAL']:
    sub = exp1[(exp1['regime_label'] == rname) & exp1['kp_simple'].notna() & exp1['kp_full'].notna()]
    if len(sub) == 0:
        continue
    fr_mask = sub['false_rejection'].fillna(False).astype(bool)
    fr_sub  = sub[fr_mask]
    ok_sub  = sub[~fr_mask]

    for data, marker_sym, marker_line, legend_sfx in [
        (ok_sub, SYMBOLS[rname], dict(color='black', width=0.3), ''),
        (fr_sub, 'x', dict(color='black', width=1.5), ' [FALSE REJECTION]'),
    ]:
        if len(data) == 0:
            continue
        fig3.add_trace(go.Scatter(
            x=data['kp_simple'], y=data['kp_full'],
            mode='markers',
            name=f"{rname}{legend_sfx} (n={len(data)})",
            marker=dict(symbol=marker_sym, size=SIZES[rname],
                        color=COLORS[rname], opacity=0.7 if not legend_sfx else 1.0,
                        line=marker_line),
            hovertemplate=f'{rname}<br>Kp_simple=%{{x:.1f}}<br>Kp_full=%{{y:.1f}}<extra></extra>',
        ))

# Diagonal (perfect agreement)
kmax = max(exp1['kp_simple'].max(), exp1['kp_full'].max())
fig3.add_trace(go.Scatter(x=[1, kmax], y=[1, kmax], mode='lines',
    name='Kp_simple = Kp_full', line=dict(color='#555', dash='dash', width=1)))

fr_rate_frag = s2r[s2r['regime_label']=='FRAGILE']['false_rejection'].mean() if 'regime_label' in s2r.columns else 0.578
fr_rate_easy = s2r[s2r['regime_label']=='EASY']['false_rejection'].mean() if 'regime_label' in s2r.columns else 0.098

ann3 = (
    f"<b>S2R gain transfer (n={n_total})</b><br>"
    f"Simple model: theta0=10°, no wind/noise<br>"
    f"Full physics: wind + slew + latency + aero<br><br>"
    f"<b>False rejection rates:</b><br>"
    f"  EASY:     9.8%   (n=2347)<br>"
    f"  FRAGILE:  57.8%  (n=45)<br>"
    f"  MARGINAL: 60.0%  (n=5)<br><br>"
    f"<b>False approval: 0.0%</b> (never dangerous)<br><br>"
    f"Median Kp_simple: EASY=36.7, FRAGILE=88.8<br>"
    f"Median Kp_full:   EASY=69.6, FRAGILE=88.8<br>"
    f"MARGINAL: simple=320, full=59 (5.4× overtuned)"
)
fig3.update_layout(
    title=dict(
        text=f"Finding 4 (S2R): Simple-model gains cause 57.8% false rejection for FRAGILE designs<br>"
             f"<sub>X marks = false rejections (gain picked in simple sim fails in full physics)  |  "
             f"False approval = 0.0% everywhere — simple model is never dangerous</sub>",
        x=0.5, xanchor='center', font=dict(size=12)),
    xaxis=dict(title='Kp selected in simple (disturbance-free) simulator', type='log'),
    yaxis=dict(title='Optimal Kp in full-physics simulator', type='log'),
    paper_bgcolor='rgb(252,252,255)', plot_bgcolor='rgb(248,248,255)',
    width=820, height=600,
    legend=dict(x=1.01, y=1.0, font=dict(size=9)),
    annotations=[dict(text=ann3, x=0.01, y=0.99, xref='paper', yref='paper', showarrow=False,
                      align='left', font=dict(size=10), bgcolor='rgba(255,255,220,0.92)',
                      bordercolor='#aaa', borderwidth=1, xanchor='left', yanchor='top')],
)
f3 = OUT / 'sts_gold_3_s2r_mismatch.html'
fig3.write_html(str(f3), include_plotlyjs='cdn')
print(f"  Saved: {f3}  ({f3.stat().st_size/1024:.0f} KB)")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 5 — Authority Trap (θ̈_max × latency_steps)
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 5: Authority Trap...")

fig5 = go.Figure()
for rname in ['EASY', 'FRAGILE']:
    sub = exp1[exp1['regime_label'] == rname]
    jitter = np.random.default_rng(42).uniform(-0.15, 0.15, len(sub))
    fig5.add_trace(go.Scatter(
        x=sub['td_max'], y=sub['latency_steps'] + jitter,
        mode='markers',
        name=f"{rname} (n={len(sub)})",
        marker=dict(size=SIZES[rname], color=COLORS[rname], symbol=SYMBOLS[rname],
                    opacity=0.45 if rname=='EASY' else 0.90,
                    line=dict(color='black', width=0.2 if rname=='EASY' else 0.8)),
        hovertemplate=f'{rname}<br>θ̈_max=%{{x:.1f}}<br>latency=%{{y:.1f}} steps<extra></extra>',
    ))

fig5.add_vline(x=54.8, line_dash='dash', line_color='#e67e22', line_width=2,
    annotation_text='θ̈_max=54.8 rad/s² (Youden-J)', annotation_position='top right',
    annotation_font=dict(size=10, color='#e67e22'))
fig5.add_hline(y=4.5, line_dash='dot', line_color='#9b59b6', line_width=1.5,
    annotation_text='latency ≥ 5 steps (25ms) — FN zone', annotation_position='bottom right',
    annotation_font=dict(size=9, color='#9b59b6'))

n_frag_hi = ((exp1['regime_label']=='FRAGILE') & (exp1['td_max']>=54.8)).sum()
n_frag_lo = ((exp1['regime_label']=='FRAGILE') & (exp1['td_max']<54.8)).sum()
ann5 = (
    f"<b>Combined predictor</b><br>"
    f"log(θ̈_max × latency) → AUC=0.972<br><br>"
    f"<b>Two FRAGILE paths:</b><br>"
    f"1. High authority: θ̈_max > 55 rad/s²<br>"
    f"   (catches {n_frag_hi}/{n_frag} FRAGILE)<br>"
    f"2. High latency: ≥5 steps AND moderate θ̈<br>"
    f"   (FN zone: 2 designs with lat=6)<br><br>"
    f"Phase lag at 5 Hz:<br>"
    f"  1-step → 9°   |   6-step → 54°<br><br>"
    f"FRAGILE base rate: 1.5% ({n_frag}/{n_total}) — final pop"
)
fig5.update_layout(
    title=dict(
        text="Finding 2: Authority × Latency — two paths to FRAGILE classification (FINAL POPULATION)<br>"
             "<sub>High θ̈_max (mechanical over-actuation) OR high latency (phase lag) compresses gain ceiling  |  "
             "AUC=0.975 from θ̈_max alone (final pop, n=36 FRAGILE)</sub>",
        x=0.5, xanchor='center', font=dict(size=12)),
    xaxis=dict(title='θ̈_max (rad/s²)', type='log'),
    yaxis=dict(title='Latency steps (5ms each at 200Hz)', dtick=1, range=[0.5, 6.8]),
    paper_bgcolor='rgb(252,252,255)', plot_bgcolor='rgb(248,248,255)',
    width=820, height=560,
    legend=dict(x=0.01, y=0.99, bgcolor='rgba(255,255,255,0.88)', bordercolor='#ccc', borderwidth=1),
    annotations=[dict(text=ann5, x=0.99, y=0.99, xref='paper', yref='paper', showarrow=False,
                      align='left', font=dict(size=10), bgcolor='rgba(255,255,220,0.92)',
                      bordercolor='#aaa', borderwidth=1, xanchor='right', yanchor='top')],
)
f5 = OUT / 'sts_gold_5_authority_trap.html'
fig5.write_html(str(f5), include_plotlyjs='cdn')
print(f"  Saved: {f5}  ({f5.stat().st_size/1024:.0f} KB)")


# ═══════════════════════════════════════════════════════════════════════════════
# Figure 6 — Product distribution (θ̈_max × latency) by regime
# ═══════════════════════════════════════════════════════════════════════════════
print("Figure 6: Product distribution...")

fig6 = go.Figure()
for rname in ['EASY', 'FRAGILE']:
    sub = exp1[exp1['regime_label'] == rname]
    prod = sub['td_x_lat']
    fig6.add_trace(go.Histogram(
        x=prod, name=f"{rname} (n={len(sub)})",
        xbins=dict(start=0, end=prod.quantile(0.995) if len(prod) else 2000, size=30),
        marker=dict(color=COLORS[rname], opacity=0.65, line=dict(color='white', width=0.5)),
        hovertemplate=f'{rname}<br>θ̈_max×lat=%{{x:.0f}}<br>count=%{{y}}<extra></extra>',
    ))

# Median product for FRAGILE
frag_prod_med = exp1.loc[exp1['regime_label']=='FRAGILE','td_x_lat'].median()
easy_prod_med = exp1.loc[exp1['regime_label']=='EASY','td_x_lat'].median()
fig6.add_vline(x=frag_prod_med, line_dash='dash', line_color='#e74c3c', line_width=2,
    annotation_text=f'FRAGILE median={frag_prod_med:.0f}',
    annotation_position='top right', annotation_font=dict(size=10, color='#e74c3c'))
fig6.add_vline(x=easy_prod_med, line_dash='dash', line_color='#2ecc71', line_width=2,
    annotation_text=f'EASY median={easy_prod_med:.0f}',
    annotation_position='top left', annotation_font=dict(size=10, color='#27ae60'))

ann6 = (
    f"<b>Combined predictor</b><br>"
    f"log(θ̈_max × latency_steps)<br>"
    f"[rad/s² × steps]<br><br>"
    f"AUC=0.972 [0.962, 0.980]<br>"
    f"10-fold CV: 0.972 ± 0.011<br><br>"
    f"Physical meaning:<br>"
    f"θ̈_max × τ_latency = max angular<br>"
    f"velocity before controller acts<br><br>"
    f"FRAGILE designs: upper tail<br>"
    f"EASY designs: lower main body"
)
fig6.update_layout(
    barmode='overlay',
    title=dict(
        text="Combined predictor: θ̈_max × latency — AUC=0.972 [0.962, 0.980]<br>"
             "<sub>Product = angular velocity before loop responds  |  "
             "FRAGILE designs concentrate in upper tail of distribution</sub>",
        x=0.5, xanchor='center', font=dict(size=12)),
    xaxis=dict(title='θ̈_max × latency_steps  (rad/s² · steps)', range=[0, 2000]),
    yaxis=dict(title='Count'),
    paper_bgcolor='rgb(252,252,255)', plot_bgcolor='rgb(248,248,255)',
    width=820, height=520,
    legend=dict(x=0.65, y=0.99, bgcolor='rgba(255,255,255,0.88)', bordercolor='#ccc', borderwidth=1),
    annotations=[dict(text=ann6, x=0.99, y=0.99, xref='paper', yref='paper', showarrow=False,
                      align='left', font=dict(size=10), bgcolor='rgba(255,255,220,0.92)',
                      bordercolor='#aaa', borderwidth=1, xanchor='right', yanchor='top')],
)
f6 = OUT / 'sts_gold_6_product_distribution.html'
fig6.write_html(str(f6), include_plotlyjs='cdn')
print(f"  Saved: {f6}  ({f6.stat().st_size/1024:.0f} KB)")


# ═══════════════════════════════════════════════════════════════════════════════
# Index page
# ═══════════════════════════════════════════════════════════════════════════════
print("Writing index...")

index_html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>STS-Gold Visualization Suite — n=2400 Definitive</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
         max-width: 900px; margin: 40px auto; padding: 0 20px; color: #222; }}
  h1   {{ color: #1a1a2e; }}
  h2   {{ color: #555; font-size: 1em; font-weight: normal; }}
  .card {{ background: #f9f9ff; border: 1px solid #dde; border-radius: 8px;
           padding: 16px 20px; margin: 12px 0; }}
  .card a {{ font-size: 1.1em; font-weight: bold; color: #2563eb; text-decoration: none; }}
  .card a:hover {{ text-decoration: underline; }}
  .tag  {{ display: inline-block; background: #e74c3c; color: white; font-size: 0.75em;
           padding: 2px 7px; border-radius: 4px; margin-left: 8px; }}
  .tag.green {{ background: #27ae60; }}
  .meta {{ color: #777; font-size: 0.88em; margin-top: 4px; }}
  .badge {{ background: #1a1a2e; color: #fff; padding: 2px 8px; border-radius: 4px;
            font-size: 0.8em; }}
</style>
</head><body>
<h1>TVC Rocket Research — STS-Gold Visualization Suite</h1>
<h2>Definitive results: n=2400 designs, n=45 FRAGILE  |  2026-06-13</h2>

<div class="card">
  <a href="sts_gold_0_atlas_3d.html">Fig 0: 3D Regime Atlas</a>
  <span class="tag green">Updated n=2400</span>
  <div class="meta">θ̈_max × Wind × Iyy — FRAGILE is a hardware property, not wind-driven.
  r(wind, FRAGILE)=−0.008. Yellow plane: Youden-J threshold = 54.8 rad/s².</div>
</div>

<div class="card">
  <a href="sts_gold_1_regime_scatter.html">Fig 1: θ̈_max by Regime</a>
  <span class="tag green">Updated n=2400</span>
  <div class="meta">FRAGILE mean=124.5 vs EASY mean=28.6 rad/s² (4.4×).
  AUC=0.944 [0.927, 0.959]. Cohen d=1.74, p=1.3×10⁻⁵⁸.</div>
</div>

<div class="card">
  <a href="sts_gold_2_gain_windows.html">Fig 2: Gain Window Curves</a>
  <div class="meta">SR(Kp) curves from relay study — FRAGILE windows narrow to
  the wind-rejection floor. EASY designs have wide flat regions.</div>
</div>

<div class="card">
  <a href="sts_gold_3_s2r_mismatch.html">Fig 3: Sim-to-Real Gain Mismatch</a>
  <span class="tag green">Updated n=2400</span>
  <div class="meta">Kp_simple vs Kp_full. False rejection 57.8% (FRAGILE) / 9.8% (EASY).
  False approval = 0.0% everywhere — simple model is never dangerous.</div>
</div>

<div class="card">
  <a href="sts_gold_4_flight_detection.html">Fig 4: Flight Signature Detection</a>
  <span class="tag">Pending rerun on n=45</span>
  <div class="meta">Single test flight at Kp=2 detects FRAGILE. AUC=0.870 (7-seed, n=41 relay study).
  Rerun on n=45 FRAGILE labels in progress.</div>
</div>

<div class="card">
  <a href="sts_gold_5_authority_trap.html">Fig 5: Authority × Latency Space</a>
  <span class="tag green">Updated n=2400</span>
  <div class="meta">Two paths to FRAGILE: high θ̈_max OR high latency. Combined AUC=0.972.
  Both FNs have latency=6 (54° phase lag at 5 Hz).</div>
</div>

<div class="card">
  <a href="sts_gold_6_product_distribution.html">Fig 6: Product Distribution</a>
  <span class="tag green">Updated n=2400</span>
  <div class="meta">θ̈_max × latency_steps distribution by regime. FRAGILE designs in upper tail.
  Combined predictor AUC=0.972 [0.962, 0.980].</div>
</div>

<hr style="margin: 30px 0; border-color: #dde;">
<p style="color:#888; font-size:0.85em;">
  Generated 2026-06-13 | n={n_total} designs | EASY={n_easy} | FRAGILE={n_frag} |
  AUC=0.944 (θ̈_max alone) | AUC=0.972 (combined)
</p>
</body></html>"""

idx = OUT / 'sts_gold_index.html'
idx.write_text(index_html, encoding='utf-8')
print(f"  Saved: {idx}")

print("\n=== SUITE COMPLETE ===")
print("Figures 0, 1, 3, 5, 6: updated with n=2400 data")
print("Figure 2: from relay study (design IDs change per LHS; qualitative pattern valid)")
print("Figure 4: run flight_sig_rerun_n45.py then regen_sts_gold_fig4.py")
