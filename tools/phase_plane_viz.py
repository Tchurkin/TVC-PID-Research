"""
tools/phase_plane_viz.py  (2026-06-22)

Phase-plane and limit-cycle visualization across the Pi range.

Shows the floor mechanism transition from linear disturbance rejection
to bang-bang limit cycle as Pi increases.

DESIGNS: 5 rockets from fsat dataset spanning Pi ~ [50, 150, 300, 500, 800]
         Each evaluated at kp_floor (where the regime transition is most visible)
         and at two additional Kp values for context.

OUTPUT: outputs/phase_plane_viz.html
        Plotly figure with subplots per design showing:
          - Phase portrait: theta vs theta_dot
          - Time series: theta(t), u_act(t), slew_sat indicator
          - Each with 3 seed overlays

Seeds: 60001-60003 (3 seeds for clean overlay; not a large-sample test)
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from pathlib import Path

from design_space import (build_plant, build_actuator, build_sensor,
                           build_disturbance, build_scenario, sample_lhs)
from simulator import simulate
from controller import PIDParams
from fidelity_config import FidelityConfig, apply_fidelity_config
from units import F15_AVG_THRUST_N, REF_MAX_GIMBAL_DEG, REF_U_MAX

CU_TO_RAD = np.pi / 180 * REF_MAX_GIMBAL_DEG / REF_U_MAX
L_NOZZLE  = 0.25

OUT_HTML = Path('outputs/phase_plane_viz.html')

# ---- 5 designs across Pi range ----
# From fsat dataset, manually confirmed
DESIGNS = [
    dict(rocket_id='R2792', Pi_approx=52,  keff=13.1, lat=2, fsat=0.18, label='Pi~52  (linear regime)'),
    dict(rocket_id='R7309', Pi_approx=150, keff=37.4, lat=2, fsat=0.44, label='Pi~150 (transitional)'),
    dict(rocket_id='R1143', Pi_approx=297, keff=11.9, lat=5, fsat=0.42, label='Pi~300 (transitional)'),
    dict(rocket_id='R5897', Pi_approx=491, keff=30.7, lat=4, fsat=0.65, label='Pi~491 (bang-bang)'),
    dict(rocket_id='R3332', Pi_approx=815, keff=22.6, lat=6, fsat=0.00, label='Pi~815 (deep narrow)'),
]

SEEDS = [60001, 60002, 60003]


def _physics_cfg():
    return FidelityConfig(
        wind=True, backlash=True, slew=True, latency=True,
        sensor_noise=True, thrust_var=False, deadband=True,
        nonlinear_aero=True, dyn_aero=True, thrust_curve=True, cg_shift=True,
    )


def run_trajectories(pool):
    """For each design, run at 3 Kp levels x 3 seeds. Return trajectory dicts."""

    fsat_df = pd.read_csv('experiments/results/fsat_window_test_py.csv')

    all_trajs = {}

    for d in DESIGNS:
        rid = d['rocket_id']
        print(f"  {d['label']} ({rid})")

        # Get floor, ceiling from fsat
        fsat_row = fsat_df[fsat_df.rocket_id == rid]
        if len(fsat_row) == 0:
            print(f"    WARNING: {rid} not in fsat dataset; using estimates")
            kp_floor = max(0.1, d['keff'] * 0.06 * d['lat'])
            kp_ceil  = 380.0 / d['lat']
        else:
            fr = fsat_row.iloc[0]
            kp_floor = float(fr['kp_floor']) if not pd.isna(fr['kp_floor']) else max(0.1, d['keff'] * 0.06 * d['lat'])
            kp_ceil  = float(fr['kp_ceiling']) if not pd.isna(fr['kp_ceiling']) else 380.0 / d['lat']
            opt_Kd   = float(fr['opt_Kd']) if not pd.isna(fr['opt_Kd']) else 1.0

        d['kp_floor'] = kp_floor
        d['kp_ceil']  = kp_ceil
        d['opt_Kd']   = opt_Kd if 'opt_Kd' in dir() else 1.0

        # 3 Kp levels: floor, geometric midpoint, near ceiling
        kp_mid  = np.sqrt(kp_floor * kp_ceil)
        kp_high = kp_ceil * 0.85
        kp_levels = {
            'floor': kp_floor,
            'mid':   kp_mid,
            'high':  kp_high,
        }

        pool_row = pool[pool.rocket_id == rid].iloc[0]
        plant = build_plant(pool_row)
        act   = build_actuator(pool_row)
        sen   = build_sensor(pool_row)
        dis   = build_disturbance(pool_row)
        sc    = build_scenario(theta0_bias_std=0.0)
        act, sen, dis, sc = apply_fidelity_config(act, sen, dis, sc, _physics_cfg())

        trajs_for_design = {}
        for kp_label, Kp in kp_levels.items():
            trajs_for_kp = []
            for seed in SEEDS:
                pid = PIDParams(Kp=float(Kp), Kd=float(opt_Kd),
                                Ki=0.0, u_max=act.u_max, i_lim=act.u_max)
                res = simulate(pid, plant, act, sen, dis, sc, seed=seed)
                trajs_for_kp.append({
                    't':         res.t,
                    'theta':     np.degrees(res.theta_true),
                    'q':         np.degrees(res.q_true),
                    'u_act':     res.u_act,
                    'slew_sat':  np.abs(res.u_act) >= act.u_max * 0.98,
                    'success':   res.success,
                    'rms':       res.rms_error_deg,
                    'slew_frac': res.slew_sat_frac,
                    'Kp':        Kp,
                    'seed':      seed,
                })
            trajs_for_design[kp_label] = trajs_for_kp
            sr = np.mean([t['success'] for t in trajs_for_kp])
            sfrac = np.mean([t['slew_frac'] for t in trajs_for_kp])
            print(f"    Kp={Kp:.2f} ({kp_label}): SR={sr:.2f}, slew_frac={sfrac:.3f}")

        all_trajs[rid] = {'design': d, 'trajs': trajs_for_design, 'u_max': act.u_max}

    return all_trajs


def build_html(all_trajs):
    """Build a Plotly-based HTML visualization."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        print("plotly not available; saving trajectory data to CSV instead")
        _save_csv(all_trajs)
        return

    n_designs = len(DESIGNS)
    # Layout: 5 rows (designs) x 3 cols (phase portrait | time theta | time u_act)
    subplot_titles = []
    for d in DESIGNS:
        subplot_titles += [
            f"<b>{d['label']}</b> -- Phase portrait (theta vs dtheta/dt)",
            "theta(t) [deg]",
            "u_act(t) [CU] + saturation",
        ]

    fig = make_subplots(
        rows=n_designs, cols=3,
        subplot_titles=subplot_titles,
        horizontal_spacing=0.08,
        vertical_spacing=0.07,
    )

    COLORS = {'floor': '#d62728', 'mid': '#ff7f0e', 'high': '#1f77b4'}
    ALPHA_SEED = [1.0, 0.55, 0.30]
    KP_STYLES  = {'floor': 'solid', 'mid': 'dot', 'high': 'dash'}

    for row_idx, d in enumerate(DESIGNS, start=1):
        rid = d['rocket_id']
        traj_data = all_trajs[rid]
        u_max = traj_data['u_max']
        trajs  = traj_data['trajs']

        kp_floor = d.get('kp_floor', 1.0)
        kp_ceil  = d.get('kp_ceil', 100.0)

        for kp_label, kp_trajs in trajs.items():
            color = COLORS[kp_label]
            Kp    = kp_trajs[0]['Kp']
            sr    = np.mean([t['success'] for t in kp_trajs])
            sfrac = np.mean([t['slew_frac'] for t in kp_trajs])

            for si, traj in enumerate(kp_trajs):
                opacity = ALPHA_SEED[si]
                name = f"Kp={Kp:.1f} ({kp_label}), seed={traj['seed']}"
                show_legend = (row_idx == 1 and si == 0)

                # Phase portrait: theta vs theta_dot
                fig.add_trace(go.Scatter(
                    x=traj['theta'], y=traj['q'],
                    mode='lines',
                    name=name,
                    line=dict(color=color, width=1.2, dash=KP_STYLES[kp_label]),
                    opacity=opacity,
                    showlegend=show_legend,
                    legendgroup=kp_label,
                ), row=row_idx, col=1)

                # Time series: theta
                fig.add_trace(go.Scatter(
                    x=traj['t'], y=traj['theta'],
                    mode='lines',
                    name=name,
                    line=dict(color=color, width=1.2, dash=KP_STYLES[kp_label]),
                    opacity=opacity,
                    showlegend=False,
                    legendgroup=kp_label,
                ), row=row_idx, col=2)

                # Time series: u_act + saturation indicator
                fig.add_trace(go.Scatter(
                    x=traj['t'], y=traj['u_act'],
                    mode='lines',
                    name=name,
                    line=dict(color=color, width=1.2, dash=KP_STYLES[kp_label]),
                    opacity=opacity,
                    showlegend=False,
                    legendgroup=kp_label,
                ), row=row_idx, col=3)

        # Add saturation lines to u_act panel
        fig.add_hline(y=u_max,  line_dash='dot', line_color='black', line_width=1,
                      opacity=0.5, row=row_idx, col=3)
        fig.add_hline(y=-u_max, line_dash='dot', line_color='black', line_width=1,
                      opacity=0.5, row=row_idx, col=3)
        # Add +/-15 deg lines to theta panel
        fig.add_hline(y=15,  line_dash='dot', line_color='gray', line_width=1,
                      opacity=0.4, row=row_idx, col=2)
        fig.add_hline(y=-15, line_dash='dot', line_color='gray', line_width=1,
                      opacity=0.4, row=row_idx, col=2)

    fig.update_layout(
        title=dict(
            text="Phase-Plane and Limit-Cycle Analysis: Pi-regime transition<br>"
                 "<sup>Floor Kp (red) | Mid Kp (orange) | Near-ceiling Kp (blue) | "
                 "Dashed lines: u_sat limits; Gray lines: 15-deg success boundary</sup>",
            font=dict(size=14),
        ),
        height=280 * n_designs,
        width=1400,
        template='plotly_white',
        legend=dict(orientation='h', y=1.02, x=0),
    )

    # Axis labels
    for row_idx, d in enumerate(DESIGNS, start=1):
        fig.update_xaxes(title_text='theta [deg]', row=row_idx, col=1)
        fig.update_yaxes(title_text='dtheta/dt [deg/s]', row=row_idx, col=1)
        fig.update_xaxes(title_text='t [s]', row=row_idx, col=2)
        fig.update_yaxes(title_text='theta [deg]', row=row_idx, col=2)
        fig.update_xaxes(title_text='t [s]', row=row_idx, col=3)
        fig.update_yaxes(title_text='u_act [CU]', row=row_idx, col=3)

    OUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(OUT_HTML))
    print(f"\nSaved to {OUT_HTML}")


def _save_csv(all_trajs):
    """Fallback: save trajectory data as CSV."""
    rows = []
    for rid, traj_data in all_trajs.items():
        d = traj_data['design']
        for kp_label, kp_trajs in traj_data['trajs'].items():
            for traj in kp_trajs:
                for i in range(len(traj['t'])):
                    rows.append(dict(
                        rocket_id=rid, Pi=d['Pi_approx'], keff=d['keff'],
                        lat=d['lat'], kp_label=kp_label, Kp=traj['Kp'],
                        seed=traj['seed'], t=traj['t'][i],
                        theta_deg=traj['theta'][i], q_deg_s=traj['q'][i],
                        u_act=traj['u_act'][i], slew_sat=int(traj['slew_sat'][i]),
                    ))
    csv_path = str(OUT_HTML).replace('.html', '.csv')
    pd.DataFrame(rows).to_csv(csv_path, index=False)
    print(f"Saved trajectory CSV to {csv_path}")


def run():
    print("phase_plane_viz.py")
    print(f"  Designs: {len(DESIGNS)}")
    print(f"  Seeds: {SEEDS}")
    print(f"  Kp levels per design: 3 (floor, mid, near-ceiling)")
    print(f"  Total sims: {len(DESIGNS) * 3 * len(SEEDS)}")

    print("\nLoading design pool (seed=8888)...")
    pool = sample_lhs(10000, seed=8888)

    print("\nRunning trajectories...")
    all_trajs = run_trajectories(pool)

    print("\nBuilding HTML visualization...")
    build_html(all_trajs)


if __name__ == '__main__':
    run()
