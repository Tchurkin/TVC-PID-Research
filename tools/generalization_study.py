"""
tools/generalization_study.py  (2026-06-23)

GENERALIZATION STUDY: Does Pi = keff * tau^2 predict control difficulty
across structurally different second-order angular control systems?

Metric: PEAK ACHIEVABLE SUCCESS RATE (best SR over a normalized Kp sweep).
This avoids the saturation-type incompatibility between systems:
  - TVC: slew-rate saturation (slew_sat_frac was the diagnostic)
  - Quad / pendulum: amplitude saturation (different mechanism)
  - SR is the common engineering outcome regardless of saturation type.

Three systems:
  A. Quadrotor roll       -- aerodynamic wind (d ∝ keff), amplitude saturation
  B. Inverted pendulum    -- gravitational instability, FIXED disturbance (d fixed)
  C. No-wind pure delay   -- zero external disturbance (isolates latency-only ceiling effect)

Expected rho(log Pi, peak_SR):
  TVC reference:   rho ≈ -0.67 (from performance_frontier experiment)
  Quad (d ∝ keff): rho ≈ -0.5 to -0.7 (similar double-squeeze as TVC)
  Pendulum (fixed): rho ≈ -0.3 to -0.5 (ceiling effect only; floor barely rises)
  Delay-only:      rho ≈ -0.2 to -0.4 (very weak; no disturbance → no floor)

Kp sweep: 12 log-spaced points in [Kp_min, Kp_max] where
  Kp_min = 100 / (keff * lat),  Kp_max = 10000 / (keff * lat)
  (normalized loop-gain range; same ceiling fraction across all systems)

Files out:
  experiments/results/gen_quad_py.csv
  experiments/results/gen_pendulum_py.csv
  experiments/results/gen_nodelay_py.csv (zero-disturbance reference)
  outputs/generalization_study.html
"""

import sys, os, warnings
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))
warnings.filterwarnings('ignore')

import numpy as np
import pandas as pd
from scipy import stats
from pathlib import Path
from joblib import Parallel, delayed
import plotly.graph_objects as go
from plotly.subplots import make_subplots

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'
OUT     = ROOT / 'outputs'

DT       = 0.005     # 200 Hz
T_END    = 3.0
N_STEPS  = int(T_END / DT)
MAX_ANG  = np.deg2rad(30.0)   # success: |theta| < 30 deg throughout

# Normalized Kp sweep range: loop_gain = Kp * keff * lat in these units
LOOP_GAIN_MIN =   100.0   # low end (below floor for high-Pi designs)
LOOP_GAIN_MAX = 10000.0   # high end (above ceiling for moderate-lat designs)
N_KP          = 12        # Kp points per design
KD_FRAC       = 0.003     # Kd = KD_FRAC / lat (normalized; heuristic from TVC Z-N)

N_DESIGNS = 25
N_EVAL    = 15
EVAL_SEED0   = 210001
DESIGN_SEED0 = 210501


def ou_series(n_steps, tau_ou, sigma, rng):
    alpha = DT / tau_ou
    noise = rng.normal(0, sigma * np.sqrt(2 * alpha), n_steps)
    d = np.zeros(n_steps)
    for i in range(1, n_steps):
        d[i] = d[i-1] * (1 - alpha) + noise[i]
    return d


def simulate_generic(keff, latency_steps, Kp, Kd, disturbance,
                     extra_accel_fn=None, u_max=1.0, theta0=0.0):
    """Euler PD + FIFO delay + amplitude clip. Returns (success, max_angle_deg)."""
    theta, thetadot = theta0, 0.0
    buf_th  = np.zeros(latency_steps + 1)
    buf_thd = np.zeros(latency_steps + 1)
    max_th  = abs(theta0)

    for k in range(N_STEPS):
        u_app = np.clip(-Kp * buf_th[-1] - Kd * buf_thd[-1], -u_max, u_max)
        d_k   = disturbance[k] if k < len(disturbance) else 0.0
        extra = extra_accel_fn(theta, thetadot) if extra_accel_fn else 0.0
        thdd  = keff * u_app + extra + d_k
        thetadot += thdd * DT
        theta    += thetadot * DT
        if abs(theta) > max_th:
            max_th = abs(theta)
        buf_th  = np.roll(buf_th,  1); buf_th[0]  = theta
        buf_thd = np.roll(buf_thd, 1); buf_thd[0] = thetadot

    return max_th < MAX_ANG, np.rad2deg(max_th)


def run_kp_sweep(eval_fn, design, Kp_list, Kd, seeds):
    """Evaluate success rate at each Kp; return (best_sr, best_Kp, sr_list)."""
    sr_list = []
    for Kp in Kp_list:
        srs = [eval_fn(design, Kp, Kd, s) for s in seeds]
        sr_list.append(float(np.mean([s for s, _ in srs])))
    best_idx = int(np.argmax(sr_list))
    return sr_list[best_idx], Kp_list[best_idx], sr_list


# ── System A: Quadrotor roll ──────────────────────────────────────────────────
# theta_ddot = keff * u + d_aero(t),   d_aero proportional to keff (aerodynamic)

def sample_quad(n, rng):
    designs = []
    while len(designs) < n:
        arm   = rng.uniform(0.03, 0.18)
        F_max = rng.uniform(0.05, 1.5)
        Ixx   = rng.uniform(0.0002, 0.006)
        lat   = int(rng.choice([1, 2, 3, 4, 5, 6]))
        gust  = rng.uniform(0.08, 0.35)
        keff  = F_max * arm / Ixx
        designs.append(dict(
            system='quad', lat=lat, keff=round(keff, 4),
            Pi=round(keff * lat**2, 2),
            d_sigma=gust * keff * 0.08,  # proportional to keff (aerodynamic coupling)
            d_tau=0.30,
        ))
    return designs


def eval_one_quad(d, Kp, Kd, seed):
    rng  = np.random.default_rng(seed)
    dist = ou_series(N_STEPS, d['d_tau'], d['d_sigma'], rng)
    th0  = float(rng.normal(0, np.deg2rad(2)))
    return simulate_generic(d['keff'], d['lat'], Kp, Kd, dist, theta0=th0)


# ── System B: Inverted pendulum ───────────────────────────────────────────────
# theta_ddot = keff * u + (g/l) * sin(theta) + d_fixed(t),   d FIXED

def sample_pendulum(n, rng):
    g = 9.81
    designs = []
    attempts = 0
    while len(designs) < n and attempts < n * 30:
        attempts += 1
        l     = rng.uniform(0.15, 0.80)
        m     = rng.uniform(0.05, 0.40)
        I     = m * l**2 * rng.uniform(1.1, 1.6)
        tau_w = rng.uniform(0.05, 4.0)
        lat   = int(rng.choice([1, 2, 3, 4, 5, 6]))
        d_amp = rng.uniform(0.10, 0.50)

        keff    = tau_w / I
        gol     = g / l

        # Must have enough authority to stabilize: effective ceiling Kp_eff > gol/keff
        # with normalized probe: Kp_max = LOOP_GAIN_MAX / (keff * lat) > gol / keff
        # => LOOP_GAIN_MAX / lat > gol  => gol < 10000/lat_min=1 => gol < 10000 (always true)
        # But need actual stability: Kp * keff > gol (static stabilization)
        # With Kp range floor = LOOP_GAIN_MIN/(keff*lat), the max loop gain at lat=1 = 10000
        # keff must be high enough: keff * (10000/keff/lat) = 10000/lat > gol
        if keff < gol * 1.5:   # need 50% headroom above instability eigenvalue
            continue

        designs.append(dict(
            system='pendulum', lat=lat, keff=round(keff, 4),
            Pi=round(keff * lat**2, 2),
            g_over_l=round(gol, 3),
            d_sigma=d_amp,   # FIXED disturbance — NOT proportional to keff
            d_tau=0.50,
        ))
    return designs


def eval_one_pendulum(d, Kp, Kd, seed):
    rng  = np.random.default_rng(seed)
    dist = ou_series(N_STEPS, d['d_tau'], d['d_sigma'], rng)
    th0  = float(rng.normal(0, np.deg2rad(4)))

    def gravity(theta, thetadot):
        return d['g_over_l'] * np.sin(theta)   # destabilizing

    return simulate_generic(d['keff'], d['lat'], Kp, Kd, dist,
                             extra_accel_fn=gravity, theta0=th0)


# ── process one design: Kp sweep → best SR ───────────────────────────────────

def process_design(args):
    design, eval_one_fn, design_idx = args
    keff = design['keff']
    lat  = design['lat']

    # Normalized Kp sweep
    loop_gains = np.logspace(np.log10(LOOP_GAIN_MIN),
                              np.log10(LOOP_GAIN_MAX), N_KP)
    Kp_list = loop_gains / (keff * lat)
    Kd      = KD_FRAC / lat    # heuristic normalized Kd

    seeds = list(range(EVAL_SEED0 + design_idx * N_EVAL,
                        EVAL_SEED0 + design_idx * N_EVAL + N_EVAL))

    sr_list = []
    for Kp in Kp_list:
        srs = [eval_one_fn(design, Kp, Kd, s)[0] for s in seeds]
        sr_list.append(float(np.mean(srs)))

    best_sr  = max(sr_list)
    best_idx = int(np.argmax(sr_list))
    best_Kp  = Kp_list[best_idx]
    best_lg  = loop_gains[best_idx]

    return {
        **design,
        'design_idx': design_idx,
        'best_sr':    round(best_sr, 4),
        'best_Kp':    round(best_Kp, 4),
        'best_lg':    round(best_lg, 1),
        'sr_list':    sr_list,
    }


def run_system(name, sample_fn, eval_one_fn, n=N_DESIGNS, seed=42):
    print(f'\n{"="*60}')
    print(f'System: {name.upper()}  (n={n})')
    print('='*60)
    rng     = np.random.default_rng(seed)
    designs = sample_fn(n, rng)

    keffs = [d['keff'] for d in designs]; pis = [d['Pi'] for d in designs]
    print(f'  keff: [{min(keffs):.1f}, {max(keffs):.1f}]   Pi: [{min(pis):.0f}, {max(pis):.0f}]')

    args = [(d, eval_one_fn, i) for i, d in enumerate(designs)]
    raw  = Parallel(n_jobs=-1, verbose=0)(delayed(process_design)(a) for a in args)
    df   = pd.DataFrame(raw)

    rho, p = stats.spearmanr(np.log10(df['Pi'] + 0.1), df['best_sr'])
    print(f'\nSpearman rho(log Pi, best_SR) = {rho:+.3f}  p = {p:.2e}')

    bins = [(0, 100), (100, 300), (300, 800), (800, 3000), (3000, 1e6)]
    for lo, hi in bins:
        sub = df[(df['Pi'] >= lo) & (df['Pi'] < hi)]
        if len(sub):
            print(f'  Pi [{lo:5.0f}-{hi:5.0f}]: n={len(sub):2d}  mean_SR={sub["best_sr"].mean():.3f}')

    return df


def load_tvc_frontier():
    p = RESULTS / 'performance_frontier_py.csv'
    if p.exists():
        df = pd.read_csv(p)
        # Keep relevant columns; use peak_pid_sr as analogous metric
        return df[['rocket_id', 'Pi', 'keff', 'latency', 'peak_pid_sr']].rename(
            columns={'peak_pid_sr': 'best_sr', 'latency': 'lat'}
        )
    return None


def make_figure(dfs, names, tvc_df=None):
    LABELS = {
        'quad':     'Quadrotor roll (wind prop. keff)',
        'pendulum': 'Inverted pendulum (fixed disturbance)',
        'tvc':      'TVC rocket (reference, optimal PID)',
    }
    COLORS = {'quad': '#3498db', 'pendulum': '#e74c3c', 'tvc': '#95a5a6'}

    plot_items = []
    if tvc_df is not None:
        plot_items.append(('tvc', tvc_df))
    plot_items.extend(zip(names, dfs))

    n_panels = len(plot_items)
    fig = make_subplots(rows=1, cols=n_panels,
                        subplot_titles=[LABELS.get(n, n) for n, _ in plot_items],
                        horizontal_spacing=0.08)

    for col, (name, df) in enumerate(plot_items, 1):
        rho, p = stats.spearmanr(np.log10(df['Pi'] + 0.1), df['best_sr'])

        # Colour by lat
        lat_vals = df['lat'].values if 'lat' in df.columns else df.get('latency_steps', df['lat']).values
        lat_norm = (lat_vals - 1) / 5.0
        cmap = [f'rgb({int(50+200*v)},{int(100-80*v)},{int(220-180*v)})' for v in lat_norm]

        fig.add_trace(go.Scatter(
            x=df['Pi'], y=df['best_sr'],
            mode='markers',
            marker=dict(color=cmap, size=11, symbol='circle',
                        line=dict(color='white', width=0.8), opacity=0.88),
            showlegend=False,
            hovertemplate=(
                f'{LABELS.get(name,name)}<br>'
                'Pi=%{x:.1f}<br>best_SR=%{y:.3f}<extra></extra>'
            ),
        ), row=1, col=col)

        # Pi threshold reference
        fig.add_shape(type='line', x0=275, x1=275, y0=-0.02, y1=1.08,
                      line=dict(color='#e74c3c', width=1.5, dash='dash'),
                      xref=f'x{col}', yref=f'y{col}')
        fig.add_shape(type='line', x0=5000, x1=5000, y0=-0.02, y1=1.08,
                      line=dict(color='#c0392b', width=1.0, dash='dot'),
                      xref=f'x{col}', yref=f'y{col}')

        p_str = f'{p:.1e}'
        # Annotation at panel-relative paper coords
        x_paper = (col - 0.9) / n_panels + 0.01
        fig.add_annotation(
            x=x_paper, y=0.22,
            xref='paper', yref='paper',
            text=f'rho = {rho:+.3f}<br>p = {p_str}',
            showarrow=False, align='left',
            font=dict(size=11, color='#333'),
            bgcolor='rgba(255,255,255,0.88)',
            bordercolor='#999', borderwidth=1,
        )

        fig.update_xaxes(type='log', range=[0.5, 5.0],
                         title_text='Pi = keff x lat^2' if col == 2 else '',
                         row=1, col=col)
        fig.update_yaxes(range=[-0.05, 1.10],
                         title_text='Peak achievable SR (best Kp)' if col == 1 else '',
                         row=1, col=col)

    # Lat colour legend
    for lat_v in [1, 3, 6]:
        v = (lat_v - 1) / 5.0
        c = f'rgb({int(50+200*v)},{int(100-80*v)},{int(220-180*v)})'
        fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                                  marker=dict(color=c, size=9),
                                  name=f'lat = {lat_v} step(s)', showlegend=True))

    fig.update_layout(
        title=dict(
            text=(
                '<b>Pi = keff * tau^2 predicts peak achievable SR across physical systems</b><br>'
                '<sup>Colour = latency steps. Dashed: Pi_sat=275 (saturation onset) | '
                'Dotted: Pi_fail=5000 (PID failure onset). Kp swept via normalized loop-gain grid.</sup>'
            ),
            x=0.5, font=dict(size=13),
        ),
        height=500, width=420 * n_panels + 80,
        template='plotly_white',
        legend=dict(x=0.02, y=-0.20, orientation='h', font=dict(size=11)),
        margin=dict(l=65, r=30, t=120, b=110),
    )

    out = OUT / 'generalization_study.html'
    fig.write_html(str(out), include_plotlyjs='cdn')
    print(f'\nSaved: {out}')


if __name__ == '__main__':
    print('Generalization study -- Pi = keff * tau^2 across physical systems')
    print(f'  {N_DESIGNS} designs per system x {N_KP} Kp points x {N_EVAL} seeds')

    tvc_df = load_tvc_frontier()
    if tvc_df is not None:
        rho0, p0 = stats.spearmanr(np.log10(tvc_df['Pi'] + 0.1), tvc_df['best_sr'])
        print(f'  TVC reference: n={len(tvc_df)}  rho(log Pi, pid_sr)={rho0:+.3f}  p={p0:.1e}')

    all_dfs, all_names = [], []

    df_q = run_system('Quadrotor', sample_quad, eval_one_quad, seed=1001)
    df_q.to_csv(RESULTS / 'gen_quad_py.csv', index=False)
    all_dfs.append(df_q); all_names.append('quad')

    df_p = run_system('Inverted pendulum', sample_pendulum, eval_one_pendulum, seed=2001)
    df_p.to_csv(RESULTS / 'gen_pendulum_py.csv', index=False)
    all_dfs.append(df_p); all_names.append('pendulum')

    # Summary
    print('\n' + '='*65)
    print(f'{"System":<30}  {"rho":>7}  {"p-value":>12}')
    print('-'*65)
    rows = []
    if tvc_df is not None:
        rows.append(('TVC rocket (reference)', tvc_df))
    rows.extend(zip(['Quadrotor', 'Inverted pendulum'], all_dfs))
    for label, df in rows:
        rho, p = stats.spearmanr(np.log10(df['Pi'] + 0.1), df['best_sr'])
        print(f'{label:<30}  {rho:+.3f}  {p:>12.2e}')
    print('='*65)

    make_figure(all_dfs, all_names, tvc_df)
