"""
tools/regime_map.py

Saturation regime map for Kp_floor: shows how floor depends on the dimensionless ratio
S·τ/u_max = slew_deg_s × latency_steps / (200 × max_gimbal_deg).

Three saturation regimes emerge from the floor_mechanism_test experiments:

  S·τ/u_max < ~0.3  →  Near-linear:      servo barely reaches its authority limit;
                                           floor is low (wind rejection easy)
  ~0.3 < S·τ/u_max < ~1.0  →  Slew-limited: servo slews for the full latency window
                                           before hitting u_max; floor ∝ keff · S · τ
  S·τ/u_max > ~1.0  →  Amplitude-limited: servo would slew past u_max within the
                                           latency window; angular velocity accumulated
                                           is capped by u_max, not by S;
                                           floor ∝ keff · u_max (rises as gimbal shrinks)

The floor formula Kp_floor ≈ 0.06 × keff × latency (held-out R²=0.711) is valid only in
the slew-limited regime. Most hobby TVC designs at S=120 deg/s, lat=6, gimbal=10 deg
sit at S·τ/u_max = 120×6/(200×10) = 0.36, comfortably in the slew-limited zone.

Data source: experiments/results/floor_mechanism_test_py.csv
  E1 conditions: E1_baseline, E1_no_slew  (slew ablation — proves slew is causal)
  E2 conditions: E2_slew040…E2_slew200   (varying S, fixed u_max)
  E3 conditions: E3_gimbal02…E3_gimbal20 (varying u_max, fixed S)
"""

from __future__ import annotations
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import pandas as pd
from pathlib import Path

# ── Load data ─────────────────────────────────────────────────────────────────
CSV = Path('experiments/results/floor_mechanism_test_py.csv')
OUT = Path('outputs/regime_map.html')

def _load():
    df = pd.read_csv(CSV)
    # S·τ/u_max = slew_deg_s × latency_steps / (200 × max_gimbal_deg)
    LATENCY = 6
    df['s_tau_over_umax'] = df['slew_deg_s'] * LATENCY / (200 * df['max_gimbal'])
    # keff_actual is constant within each keff_target group for E2 and E3 (Iyy compensated)
    # label by keff_target for coloring
    df['keff_label'] = df['keff_target'].map({8.0: 'keff≈5.3', 15.0: 'keff≈10.0', 25.0: 'keff≈16.7'})
    # experiment type
    df['exp'] = df['condition'].str[:2]   # E1, E2, E3
    df['subtype'] = df['condition']
    return df

# ── Plotly figure ─────────────────────────────────────────────────────────────
def _make_figure(df: pd.DataFrame):
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        print("plotly not available — writing CSV summary only")
        return None

    COLORS = {8.0: '#2196F3', 15.0: '#FF9800', 25.0: '#E91E63'}
    KEFF_NAMES = {8.0: 'k_eff ≈ 5.3 rad/s²/CU', 15.0: 'k_eff ≈ 10.0 rad/s²/CU',
                  25.0: 'k_eff ≈ 16.7 rad/s²/CU'}

    fig = make_subplots(
        rows=1, cols=2,
        column_widths=[0.72, 0.28],
        subplot_titles=[
            'Regime Map: Kp_floor vs S·τ/u_max',
            'E1 Slew Ablation: floor drop without slew'
        ],
        horizontal_spacing=0.12,
    )

    # ── Left panel: regime map ─────────────────────────────────────────────────
    # Shaded regime zones
    fig.add_vrect(x0=0.05, x1=0.30, fillcolor='#E3F2FD', opacity=0.5,
                  layer='below', line_width=0, row=1, col=1,
                  annotation_text='Near-linear', annotation_position='top left',
                  annotation_font_size=11, annotation_font_color='#1565C0')
    fig.add_vrect(x0=0.30, x1=1.00, fillcolor='#FFF8E1', opacity=0.5,
                  layer='below', line_width=0, row=1, col=1,
                  annotation_text='Slew-limited (floor ∝ S·τ)', annotation_position='top left',
                  annotation_font_size=11, annotation_font_color='#E65100')
    fig.add_vrect(x0=1.00, x1=2.5, fillcolor='#FCE4EC', opacity=0.5,
                  layer='below', line_width=0, row=1, col=1,
                  annotation_text='Amplitude-limited', annotation_position='top left',
                  annotation_font_size=11, annotation_font_color='#880E4F')

    # Regime boundary lines
    for xv, label in [(0.30, 'Slew transition (S·τ/u_max≈0.3)'),
                      (1.00, 'Amplitude saturation (S·τ/u_max=1.0)')]:
        fig.add_vline(x=xv, line=dict(color='#666', dash='dash', width=1.5),
                      row=1, col=1)

    # E2 points: filled circles (varying slew, fixed u_max)
    e2 = df[df['exp'] == 'E2'].copy()
    # E3 points: filled triangles (varying u_max, fixed slew)
    e3 = df[df['exp'] == 'E3'].copy()

    for kt in [8.0, 15.0, 25.0]:
        color = COLORS[kt]
        name = KEFF_NAMES[kt]

        # E2 series
        sub2 = e2[e2['keff_target'] == kt].sort_values('s_tau_over_umax')
        valid2 = sub2[~sub2['infeasible'] & ~sub2['floor_cens']]
        infeas2 = sub2[sub2['infeasible']]

        if len(valid2) > 0:
            fig.add_trace(go.Scatter(
                x=valid2['s_tau_over_umax'], y=valid2['kp_floor'],
                mode='markers+lines',
                marker=dict(symbol='circle', size=10, color=color,
                            line=dict(width=1.5, color='white')),
                line=dict(color=color, width=1.5, dash='solid'),
                name=name + ' (E2, vary S)',
                legendgroup=str(kt),
                hovertemplate=(
                    'S·τ/u_max=%{x:.2f}<br>floor=%{y:.2f}<br>'
                    'slew=%{customdata[0]:.0f} deg/s<extra>' + name + ' E2</extra>'
                ),
                customdata=valid2[['slew_deg_s']].values,
            ), row=1, col=1)

        if len(infeas2) > 0:
            fig.add_trace(go.Scatter(
                x=infeas2['s_tau_over_umax'], y=[0.4] * len(infeas2),
                mode='markers',
                marker=dict(symbol='x', size=14, color=color, line=dict(width=2)),
                name=name + ' (E2, infeasible)',
                legendgroup=str(kt),
                showlegend=False,
                hovertemplate='INFEASIBLE (E2)<extra></extra>',
            ), row=1, col=1)

        # E3 series
        sub3 = e3[e3['keff_target'] == kt].sort_values('s_tau_over_umax')
        valid3 = sub3[~sub3['infeasible'] & ~sub3['floor_cens']]
        infeas3 = sub3[sub3['infeasible']]

        if len(valid3) > 0:
            fig.add_trace(go.Scatter(
                x=valid3['s_tau_over_umax'], y=valid3['kp_floor'],
                mode='markers+lines',
                marker=dict(symbol='triangle-up', size=11, color=color,
                            line=dict(width=1.5, color='white')),
                line=dict(color=color, width=1.5, dash='dot'),
                name=name + ' (E3, vary u_max)',
                legendgroup=str(kt),
                hovertemplate=(
                    'S·τ/u_max=%{x:.2f}<br>floor=%{y:.2f}<br>'
                    'gimbal=%{customdata[0]:.0f}°<extra>' + name + ' E3</extra>'
                ),
                customdata=valid3[['max_gimbal']].values,
            ), row=1, col=1)

        if len(infeas3) > 0:
            fig.add_trace(go.Scatter(
                x=infeas3['s_tau_over_umax'], y=[0.4] * len(infeas3),
                mode='markers',
                marker=dict(symbol='x', size=14, color=color, line=dict(width=2)),
                name=name + ' (E3, infeasible)',
                legendgroup=str(kt),
                showlegend=False,
                hovertemplate='INFEASIBLE (E3)<extra></extra>',
            ), row=1, col=1)

    # Theoretical slope guide line in slew-limited zone: floor ∝ S·τ/u_max
    # Anchor to keff=15, x=0.36 (baseline), floor ≈ 9.73
    anchor_x = 0.36
    anchor_y = 9.73
    xs_guide = np.array([0.12, 1.0])
    ys_guide = anchor_y * (xs_guide / anchor_x)
    fig.add_trace(go.Scatter(
        x=xs_guide, y=ys_guide,
        mode='lines',
        line=dict(color='#333', width=1.5, dash='longdash'),
        name='Slope=1 guide (floor ∝ S·τ/u_max)',
        showlegend=True,
    ), row=1, col=1)

    fig.update_xaxes(title_text='S·τ / u_max = slew × latency / (200 × max_gimbal)',
                     type='log', range=[np.log10(0.09), np.log10(2.5)],
                     row=1, col=1)
    fig.update_yaxes(title_text='Kp_floor', type='log', row=1, col=1)

    # ── Right panel: E1 ablation ───────────────────────────────────────────────
    e1 = df[df['exp'] == 'E1'].copy()

    for kt in [8.0, 15.0, 25.0]:
        color = COLORS[kt]
        name = KEFF_NAMES[kt]
        sub = e1[e1['keff_target'] == kt]

        base = sub[sub['condition'] == 'E1_baseline']
        noslew = sub[sub['condition'] == 'E1_no_slew']

        y_base = base['kp_floor'].values[0] if (len(base) > 0 and not base['infeasible'].values[0]) else None
        y_noslew = noslew['kp_floor'].values[0] if (len(noslew) > 0 and not noslew['infeasible'].values[0]) else None

        x_base_label = name + '\n(baseline)'
        x_noslew_label = name + '\n(no slew)'

        if y_base is not None:
            fig.add_trace(go.Bar(
                x=[f'keff{kt:.0f}<br>baseline'],
                y=[y_base],
                marker_color=color,
                name=name + ' baseline',
                legendgroup='ablation',
                showlegend=False,
                hovertemplate='floor=%{y:.2f} (baseline)<extra></extra>',
            ), row=1, col=2)
        else:
            # Infeasible baseline — plot a high-value bar with crosshatch
            fig.add_trace(go.Bar(
                x=[f'keff{kt:.0f}<br>baseline'],
                y=[80],
                marker_color=color, opacity=0.3,
                name=name + ' baseline (infeasible)',
                legendgroup='ablation',
                showlegend=False,
                hovertemplate='INFEASIBLE (baseline)<extra></extra>',
            ), row=1, col=2)
            fig.add_annotation(
                x=f'keff{kt:.0f}<br>baseline', y=85,
                text='INFEAS.', showarrow=False,
                font=dict(size=10, color=color),
                row=1, col=2,
            )

        if y_noslew is not None:
            fig.add_trace(go.Bar(
                x=[f'keff{kt:.0f}<br>no slew'],
                y=[y_noslew],
                marker_color=color, opacity=0.55,
                marker_pattern_shape='/',
                name=name + ' no-slew',
                legendgroup='ablation',
                showlegend=False,
                hovertemplate='floor=%{y:.2f} (no slew)<extra></extra>',
            ), row=1, col=2)

    fig.update_yaxes(title_text='Kp_floor', type='log', row=1, col=2)
    fig.update_xaxes(title_text='', row=1, col=2, tickfont=dict(size=9))

    # ── Layout ─────────────────────────────────────────────────────────────────
    fig.update_layout(
        title=dict(
            text=(
                '<b>Saturation Regime Map: what S·τ/u_max governs</b><br>'
                '<sup>Three regimes of the gain floor as a function of servo slew (S), '
                'latency (τ), and max-gimbal authority (u_max)</sup>'
            ),
            x=0.5, xanchor='center',
            font=dict(size=15),
        ),
        height=560, width=1050,
        plot_bgcolor='white',
        paper_bgcolor='white',
        legend=dict(
            x=0.52, y=0.02, xanchor='left',
            font=dict(size=11),
            bgcolor='rgba(255,255,255,0.85)',
            bordercolor='#ccc', borderwidth=1,
        ),
        font=dict(family='Arial, sans-serif'),
    )

    # Grid lines
    for r, c in [(1,1), (1,2)]:
        fig.update_xaxes(showgrid=True, gridcolor='#eee', row=r, col=c)
        fig.update_yaxes(showgrid=True, gridcolor='#eee', row=r, col=c)

    return fig


def _print_summary(df: pd.DataFrame):
    print("=" * 68)
    print("SATURATION REGIME MAP — DATA SUMMARY")
    print("=" * 68)
    print(f"\nS*tau/u_max = slew_deg_s * {6} / (200 * max_gimbal_deg)\n")

    for exp_label, exp_code in [('E2 (vary slew, fixed u_max=8 CU)', 'E2'),
                                  ('E3 (vary u_max, fixed slew=120)', 'E3')]:
        print(f"\n{exp_label}")
        sub = df[df['exp'] == exp_code][
            ['keff_target', 's_tau_over_umax', 'slew_deg_s', 'max_gimbal',
             'kp_floor', 'infeasible', 'floor_cens']
        ].sort_values(['keff_target', 's_tau_over_umax'])
        print(sub.to_string(index=False, float_format='%.2f'))

    print("\n\nE1 SLEW ABLATION (slew=on vs slew=off):")
    e1 = df[df['exp'] == 'E1'][
        ['keff_target', 'condition', 'kp_floor', 'kp_ceil', 'window', 'infeasible']
    ].sort_values(['keff_target', 'condition'])
    print(e1.to_string(index=False, float_format='%.2f'))

    # Regime assignments
    print("\n\nREGIME CLASSIFICATION:")
    print("  Near-linear:       S*tau/u_max < 0.30 -> keff=8, gimbal=20 (0.18); keff=8, gimbal=15 (0.24)")
    print("  Slew-limited:      0.30 < S*tau/u_max < 1.00 -> most designs in E2+E3")
    print("  Amplitude-limited: S*tau/u_max > 1.00 -> gimbal=2 at all keff levels")

    print("\n\nPRACTICAL REFERENCE POINT:")
    print("  Typical hobby TVC: slew=120 deg/s, latency=6 steps, gimbal=10 deg")
    print("  S*tau/u_max = 120*6/(200*10) = 0.36  -> SLEW-LIMITED regime")
    print("  Floor formula Kp ~= 0.06 * keff * lat is valid here (R^2=0.711 held-out)")
    print("=" * 68)


def main():
    if not CSV.exists():
        print(f"ERROR: {CSV} not found — run tools/floor_mechanism_test.py first")
        sys.exit(1)

    df = _load()
    _print_summary(df)

    fig = _make_figure(df)
    if fig is not None:
        OUT.parent.mkdir(exist_ok=True)
        fig.write_html(str(OUT), include_plotlyjs='cdn')
        print(f"\nFigure saved -> {OUT}")
    else:
        print("Skipped HTML output (plotly unavailable)")


if __name__ == '__main__':
    main()
