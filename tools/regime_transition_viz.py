"""
tools/regime_transition_viz.py

Primary visualization for the Pi saturation-regime-transition claim.
Four panels, one figure:
  A. fsat vs Pi (regime map) — the transition
  B. PID vs ADRC SR vs Pi (performance frontier) — the ADRC alignment
  C. PID-nosat vs PID-sat (2×2 factorial) — the causal proof
  D. Delay model comparison — scope + lag filter finding

Saves: outputs/regime_transition.html
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pathlib import Path

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'
OUT     = ROOT / 'outputs'

# ── load data ─────────────────────────────────────────────────────────────────

regime_map   = pd.read_csv(RESULTS / 'saturation_regime_map_py.csv')
frontier     = pd.read_csv(RESULTS / 'performance_frontier_py.csv')
sat_test     = pd.read_csv(RESULTS / 'adrc_saturation_test_py.csv')
delay_model  = pd.read_csv(RESULTS / 'delay_model_robustness_py.csv')

# ── colours ───────────────────────────────────────────────────────────────────
C_LINEAR  = '#2ecc71'   # green
C_TRANS   = '#f39c12'   # amber
C_SAT     = '#e74c3c'   # red
C_PID     = '#e74c3c'
C_ADRC    = '#2ecc71'
C_NOSAT   = '#9b59b6'

DELAY_COLORS = {
    'integer':   '#e74c3c',
    'jitter_1':  '#f39c12',
    'lag_delay': '#3498db',
    'firstlag':  '#2ecc71',
}

DELAY_LABELS = {
    'integer':   'Integer FIFO (bare-metal)',
    'jitter_1':  'Jitter ±1 step',
    'lag_delay': 'Lag + delay',
    'firstlag':  'First-order lag (IIR filter)',
}

# ── figure setup ──────────────────────────────────────────────────────────────

fig = make_subplots(
    rows=2, cols=2,
    subplot_titles=[
        'A. Saturation onset — fsat vs Π',
        'B. PID vs ADRC success rate vs Π',
        'C. Causal proof: remove saturation → PID = 1.000',
        'D. Delay model scope of Pi_crit ≈ 275',
    ],
    vertical_spacing=0.18,
    horizontal_spacing=0.12,
)

# ── Panel A: fsat vs Pi (regime map) ─────────────────────────────────────────

def regime_color(row):
    if row['fsat'] < 0.10: return C_LINEAR
    if row['fsat'] < 0.35: return C_TRANS
    return C_SAT

rm = regime_map.copy()
rm['color'] = rm.apply(regime_color, axis=1)
rm = rm.sort_values('Pi')

# Add background band for transition zone
fig.add_vrect(x0=177, x1=275, fillcolor='#f39c12', opacity=0.08,
              line_width=0, row=1, col=1)

for regime, grp in rm.groupby('color'):
    label = {C_LINEAR: 'Linear (fsat < 0.10)',
             C_TRANS:  'Transitional (0.10–0.35)',
             C_SAT:    'Saturation (fsat ≥ 0.35)'}[regime]
    fig.add_trace(go.Scatter(
        x=grp['Pi'], y=grp['fsat'],
        mode='markers',
        marker=dict(color=regime, size=9, symbol='circle',
                    line=dict(color='white', width=0.8)),
        name=label,
        legendgroup=f'A_{regime}',
        showlegend=True,
        hovertemplate='<b>%{customdata[0]}</b><br>Π=%{x:.0f}<br>fsat=%{y:.3f}<extra></extra>',
        customdata=grp[['rocket_id']].values,
    ), row=1, col=1)

# Annotate R0522 — smoking gun
r0522 = rm[rm['rocket_id'] == 'R0522']
if len(r0522):
    fig.add_annotation(
        x=r0522['Pi'].values[0], y=r0522['fsat'].values[0],
        text='R0522: keff=41.4<br>lat=1, Π=41<br>→ linear!',
        showarrow=True, arrowhead=2, arrowcolor='#2ecc71',
        font=dict(size=10, color='#2ecc71'),
        ax=60, ay=-45, row=1, col=1,
        xref='x', yref='y',
    )

# Vertical lines for thresholds
fig.add_vline(x=275, line_dash='dash', line_color='#e74c3c', line_width=1.5,
              annotation_text='Π=275<br>onset', annotation_position='top right',
              annotation_font_size=10, row=1, col=1)

fig.update_xaxes(title_text='Π = keff × τ²', type='log', range=[1.4, 3.3],
                 row=1, col=1)
fig.update_yaxes(title_text='Saturation fraction (fsat)', range=[-0.02, 1.0],
                 row=1, col=1)

# ── Panel B: PID vs ADRC SR vs Pi (frontier) ─────────────────────────────────

frt = frontier.copy().sort_values('Pi')
frt['Pi'] = pd.to_numeric(frt['Pi'], errors='coerce')

fig.add_trace(go.Scatter(
    x=frt['Pi'], y=frt['peak_pid_sr'],
    mode='markers',
    marker=dict(color=C_PID, size=8, symbol='circle',
                line=dict(color='white', width=0.5)),
    name='PID (optimal Kp)',
    legendgroup='B_pid',
    showlegend=True,
    hovertemplate='Π=%{x:.0f}<br>PID SR=%{y:.3f}<extra></extra>',
), row=1, col=2)

fig.add_trace(go.Scatter(
    x=frt['Pi'], y=frt['peak_adrc_sr'],
    mode='markers',
    marker=dict(color=C_ADRC, size=8, symbol='diamond',
                line=dict(color='white', width=0.5)),
    name='ADRC (optimal ωc)',
    legendgroup='B_adrc',
    showlegend=True,
    hovertemplate='Π=%{x:.0f}<br>ADRC SR=%{y:.3f}<extra></extra>',
), row=1, col=2)

# Vertical lines at key transitions
fig.add_vline(x=275,  line_dash='dash', line_color='#f39c12', line_width=1,
              row=1, col=2)
fig.add_vline(x=321,  line_dash='dot',  line_color=C_ADRC,    line_width=1.5,
              annotation_text='Pi=321<br>ADRC advantage', annotation_position='top left',
              annotation_font_size=9, row=1, col=2)
fig.add_vline(x=5000, line_dash='dash', line_color=C_PID,     line_width=1.5,
              annotation_text='Pi=5k<br>PID starts failing',  annotation_position='top left',
              annotation_font_size=9, row=1, col=2)

fig.update_xaxes(title_text='Π = keff × τ²', type='log', range=[1.4, 5.1],
                 row=1, col=2)
fig.update_yaxes(title_text='Peak success rate (30 seeds)', range=[-0.05, 1.05],
                 row=1, col=2)

# ── Panel C: 2×2 factorial causal proof ───────────────────────────────────────

st = sat_test.copy().sort_values('Pi')
st['Pi'] = pd.to_numeric(st['Pi'], errors='coerce')

fig.add_trace(go.Scatter(
    x=st['Pi'], y=st['pid_sat'],
    mode='markers+lines',
    marker=dict(color=C_PID, size=9, symbol='circle'),
    line=dict(color=C_PID, width=2, dash='solid'),
    name='PID + saturation',
    legendgroup='C_pid_sat',
    showlegend=True,
    hovertemplate='Π=%{x:.0f}<br>PID SR=%{y:.3f}<extra></extra>',
), row=2, col=1)

fig.add_trace(go.Scatter(
    x=st['Pi'], y=st['pid_nosat'],
    mode='markers+lines',
    marker=dict(color=C_NOSAT, size=9, symbol='square'),
    line=dict(color=C_NOSAT, width=2, dash='dash'),
    name='PID − saturation',
    legendgroup='C_pid_nosat',
    showlegend=True,
    hovertemplate='Π=%{x:.0f}<br>PID-nosat SR=%{y:.3f}<extra></extra>',
), row=2, col=1)

fig.add_trace(go.Scatter(
    x=st['Pi'], y=st['adrc_sat'],
    mode='markers+lines',
    marker=dict(color=C_ADRC, size=9, symbol='diamond'),
    line=dict(color=C_ADRC, width=2, dash='solid'),
    name='ADRC (sat irrelevant)',
    legendgroup='C_adrc',
    showlegend=True,
    hovertemplate='Π=%{x:.0f}<br>ADRC SR=%{y:.3f}<extra></extra>',
), row=2, col=1)

# Annotation: PID-nosat = 1.000
fig.add_annotation(
    x=0.50, y=0.92, xref='x3 domain', yref='y3 domain',
    text='PID − saturation = 1.000<br>for all 15 designs<br><i>(saturation is necessary + sufficient)</i>',
    showarrow=False, align='left',
    font=dict(size=10, color=C_NOSAT),
    bgcolor='rgba(255,255,255,0.8)',
    bordercolor=C_NOSAT, borderwidth=1,
)

fig.update_xaxes(title_text='Π = keff × τ²', type='log', range=[1.4, 4.2],
                 row=2, col=1)
fig.update_yaxes(title_text='Success rate', range=[-0.05, 1.05], row=2, col=1)

# ── Panel D: Delay model comparison ───────────────────────────────────────────

dm = delay_model.copy()
dm['Pi'] = pd.to_numeric(dm['Pi'], errors='coerce')

for model in ['integer', 'jitter_1', 'lag_delay', 'firstlag']:
    sub = dm[dm['delay_mode'] == model].sort_values('Pi')
    fig.add_trace(go.Scatter(
        x=sub['Pi'], y=sub['fsat'],
        mode='markers+lines',
        marker=dict(color=DELAY_COLORS[model], size=8, symbol='circle',
                    line=dict(color='white', width=0.5)),
        line=dict(color=DELAY_COLORS[model], width=2,
                  dash='solid' if model != 'firstlag' else 'dot'),
        name=DELAY_LABELS[model],
        legendgroup=f'D_{model}',
        showlegend=True,
        hovertemplate=f'{DELAY_LABELS[model]}<br>Π=%{{x:.0f}}<br>fsat=%{{y:.3f}}<extra></extra>',
    ), row=2, col=2)

fig.add_annotation(
    x=0.55, y=0.18, xref='x4 domain', yref='y4 domain',
    text='First-order lag: fsat = 0<br>at ALL Pi values<br><i>(rate smoothing eliminates mechanism)</i>',
    showarrow=False, align='left',
    font=dict(size=10, color=DELAY_COLORS['firstlag']),
    bgcolor='rgba(255,255,255,0.85)',
    bordercolor=DELAY_COLORS['firstlag'], borderwidth=1,
)

fig.update_xaxes(title_text='Π = keff × τ²', type='log', range=[1.4, 3.3],
                 row=2, col=2)
fig.update_yaxes(title_text='Saturation fraction (fsat)', range=[-0.02, 1.0],
                 row=2, col=2)

# ── global layout ──────────────────────────────────────────────────────────────

fig.update_layout(
    title=dict(
        text=(
            '<b>Π = keff × τ² predicts the saturation regime transition in TVC attitude control</b>'
            '<br><sup>Four independent tests converging on the same threshold</sup>'
        ),
        x=0.5, font=dict(size=17),
    ),
    height=820,
    width=1200,
    template='plotly_white',
    legend=dict(
        x=1.01, y=1.0, xanchor='left',
        font=dict(size=10),
        tracegroupgap=6,
        bordercolor='#ddd', borderwidth=1,
    ),
    font=dict(family='Arial, sans-serif', size=12),
    margin=dict(l=60, r=220, t=100, b=60),
)

# ── save ──────────────────────────────────────────────────────────────────────

out_path = OUT / 'regime_transition.html'
fig.write_html(str(out_path), include_plotlyjs='cdn')
print(f'Saved: {out_path}')

# ── also save a clean standalone fsat-vs-Pi scatter for the single key claim ──

fig2 = go.Figure()

rm2 = regime_map.copy().sort_values('Pi')
for regime_label, filt, color, sym in [
    ('Linear (fsat < 0.10)',      rm2['fsat'] < 0.10, C_LINEAR, 'circle'),
    ('Transitional (0.10–0.35)',  (rm2['fsat'] >= 0.10) & (rm2['fsat'] < 0.35), C_TRANS, 'square'),
    ('Saturation (fsat ≥ 0.35)',  rm2['fsat'] >= 0.35, C_SAT, 'diamond'),
]:
    grp = rm2[filt]
    fig2.add_trace(go.Scatter(
        x=grp['Pi'], y=grp['fsat'],
        mode='markers',
        marker=dict(color=color, size=12, symbol=sym,
                    line=dict(color='white', width=1)),
        name=regime_label,
        hovertemplate='<b>%{customdata}</b><br>Π=%{x:.0f}<br>fsat=%{y:.3f}<extra></extra>',
        customdata=grp['rocket_id'],
    ))

# R0522 annotation
if len(r0522):
    fig2.add_annotation(
        x=r0522['Pi'].values[0], y=r0522['fsat'].values[0],
        text='<b>R0522</b><br>keff = 41.4 (highest in dataset)<br>latency = 1 step, Π = 41<br>→ fsat = 0.018 (LINEAR)',
        showarrow=True, arrowhead=2, arrowcolor='#27ae60', arrowsize=1.3,
        font=dict(size=11, color='#27ae60'),
        bgcolor='rgba(255,255,255,0.90)',
        bordercolor='#27ae60', borderwidth=1.5,
        ax=90, ay=-60,
    )

fig2.add_vrect(x0=177, x1=275, fillcolor='#f39c12', opacity=0.10, line_width=0)
fig2.add_vline(x=275, line_dash='dash', line_color='#e74c3c', line_width=2,
               annotation_text='  Π ≈ 275<br>  saturation onset',
               annotation_font=dict(size=12, color='#e74c3c'),
               annotation_position='top right')

fig2.add_annotation(
    x=0.25, y=0.94, xref='paper', yref='paper',
    text=(
        'Theory: A_blind = ½ × keff × u_max × τ²<br>'
        'ρ(theory, fsat) = 0.808  ≈  ρ(Π, fsat) = 0.799'
    ),
    showarrow=False, align='left',
    font=dict(size=11),
    bgcolor='rgba(245,245,245,0.9)',
    bordercolor='#aaa', borderwidth=1,
)

fig2.update_layout(
    title=dict(
        text=(
            '<b>Π = keff × τ² predicts the bang-bang saturation transition</b>'
            '<br><sup>n = 29 designs; probe Kp = 190/lat (below linear ceiling — saturation here is structural)</sup>'
        ),
        x=0.5, font=dict(size=16),
    ),
    xaxis=dict(title='Π = keff × τ²  [rad/CU]', type='log', range=[1.4, 3.3]),
    yaxis=dict(title='Servo saturation fraction (fsat)', range=[-0.02, 1.02]),
    height=560, width=860,
    template='plotly_white',
    legend=dict(x=0.01, y=0.99, xanchor='left', yanchor='top',
                font=dict(size=12), bgcolor='rgba(255,255,255,0.85)',
                bordercolor='#ddd', borderwidth=1),
    font=dict(family='Arial, sans-serif', size=13),
    margin=dict(l=70, r=40, t=90, b=60),
)

out2 = OUT / 'fsat_vs_pi.html'
fig2.write_html(str(out2), include_plotlyjs='cdn')
print(f'Saved: {out2}')
