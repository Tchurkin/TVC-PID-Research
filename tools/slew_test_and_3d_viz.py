"""
tools/slew_test_and_3d_viz.py

Two things:
1. Does adding log(slew) to the window_ratio regression improve CV R²?
   (Tests whether slew collapses into the Π formula or adds independent info)

2. 3D gain-window visualization:
   X = theta_ddot_max, Y = latency_steps, Z = Kp
   Green dots = SR >= 0.80 (valid gain), Red dots = SR < 0.80 (fail)
   Shows intuitively how the green window narrows with authority and latency.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sim'))

import numpy as np
import pandas as pd
from pathlib import Path
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import cross_val_score
import plotly.graph_objects as go

ROOT    = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'experiments' / 'results'
OUTPUTS = ROOT / 'outputs'

SR_PASS = 0.80

# ── PART 1: SLEW REGRESSION TEST ─────────────────────────────────────────────

print("=" * 60)
print("PART 1: Does slew improve the window_ratio regression?")
print("=" * 60)

wr = pd.read_csv(RESULTS / 'window_ratio_v2_py.csv')

# Filter: both floor and ceiling non-censored, window_ratio valid
valid = wr[
    ~wr['floor_censored'] &
    ~wr['ceil_censored'] &
    wr['window_ratio'].notna() &
    (wr['window_ratio'] > 0) &
    (wr['servo_slew'] > 0)
].copy()

print(f"Non-censored designs: {len(valid)} of {len(wr)}")

log_window = np.log(valid['window_ratio'].values)
log_keff   = np.log(valid['keff'].values)
log_lat    = np.log(valid['latency_steps'].values)
log_slew   = np.log(valid['servo_slew'].values)
log_td     = np.log(valid['td'].values)
log_Pi     = np.log(valid['td'].values * valid['latency_steps'].values**2)

lr = LinearRegression()

def cv_r2(X, y, cv=5):
    scores = cross_val_score(lr, X, y, cv=cv, scoring='r2')
    return scores.mean(), scores.std()

configs = {
    'keff + latency (baseline)': np.column_stack([log_keff, log_lat]),
    'keff + latency + slew':     np.column_stack([log_keff, log_lat, log_slew]),
    'Π = td × lat² (single param)': log_Pi.reshape(-1, 1),
    'Π + slew':                  np.column_stack([log_Pi, log_slew]),
    'td + latency (separate)':   np.column_stack([log_td, log_lat]),
    'td + latency + slew':       np.column_stack([log_td, log_lat, log_slew]),
}

results = {}
for name, X in configs.items():
    mean, std = cv_r2(X, log_window)
    lr.fit(X, log_window)
    r2_in = lr.score(X, log_window)
    results[name] = (mean, std, r2_in)
    delta = ''
    print(f"  {name:<40} CV R2={mean:.3f}+/-{std:.3f}  in-sample={r2_in:.3f}")

baseline_cv = results['keff + latency (baseline)'][0]
slew_cv     = results['keff + latency + slew'][0]
delta_cv    = slew_cv - baseline_cv

print(f"\nΔCV R² from adding slew to baseline: {delta_cv:+.3f}")
if delta_cv > 0.05:
    print("→ SLEW MATTERS: meaningful improvement (>0.05). Include in formula.")
elif delta_cv > 0.02:
    print("→ MARGINAL: small improvement. Slew partially collapses into current predictors.")
else:
    print("→ SLEW COLLAPSES: negligible improvement. Formula is complete for this range.")

# Slew correlation check
r_slew_window = np.corrcoef(log_slew, log_window)[0, 1]
r_slew_lat    = np.corrcoef(log_slew, log_lat)[0, 1]
r_slew_keff   = np.corrcoef(log_slew, log_keff)[0, 1]
print(f"\nSlew correlations:")
print(f"  r(log_slew, log_window) = {r_slew_window:+.3f}")
print(f"  r(log_slew, log_lat)    = {r_slew_lat:+.3f}")
print(f"  r(log_slew, log_keff)   = {r_slew_keff:+.3f}")

# Fit keff + latency and check slew residual
lr.fit(np.column_stack([log_keff, log_lat]), log_window)
resid = log_window - lr.predict(np.column_stack([log_keff, log_lat]))
r_resid_slew = np.corrcoef(log_slew, resid)[0, 1]
print(f"  r(log_slew, residual after keff+lat) = {r_resid_slew:+.3f}")
print(f"  → If near 0: slew adds nothing beyond keff+lat.")

# ── PART 2: 3D GAIN-WINDOW VISUALIZATION ─────────────────────────────────────

print("\n" + "=" * 60)
print("PART 2: Building 3D gain-window visualization")
print("=" * 60)

sweep = pd.read_csv(RESULTS / 'window_ratio_v2_sweep_py.csv')

# Join sweep with design metadata
sweep = sweep.merge(
    wr[['rocket_id', 'td', 'keff', 'latency_steps', 'servo_slew',
        'kp_floor', 'kp_ceiling', 'window_ratio', 'floor_censored', 'ceil_censored']],
    on='rocket_id', how='left'
)

sweep['pass'] = sweep['sr'] >= SR_PASS

# Stratify: pick representative designs across td × latency bins
# Want 3-4 td tiers × 2-3 latency levels = ~8-12 designs
wr_plot = wr.copy()
wr_plot['td_tier'] = pd.cut(wr_plot['td'],
    bins=[0, 60, 120, 200, 9999],
    labels=['td<60', 'td 60-120', 'td 120-200', 'td>200'])
wr_plot['lat_tier'] = pd.cut(wr_plot['latency_steps'],
    bins=[0, 2, 4, 99],
    labels=['lat 1-2', 'lat 3-4', 'lat 5-6'])

# Pick up to 3 designs per (td_tier × lat_tier) cell, prefer non-censored
selected_ids = []
for (td_t, lat_t), grp in wr_plot.groupby(['td_tier', 'lat_tier'], observed=True):
    sub = grp[~grp['floor_censored'] & ~grp['ceil_censored']]
    if len(sub) == 0:
        sub = grp
    picked = sub.nlargest(min(3, len(sub)), 'td')  # highest td within tier
    selected_ids.extend(picked['rocket_id'].tolist())

selected_ids = list(set(selected_ids))
print(f"Selected {len(selected_ids)} designs for 3D visualization")

plot_data = sweep[sweep['rocket_id'].isin(selected_ids)].copy()

# Jitter latency slightly so overlapping designs are distinguishable
rng = np.random.default_rng(42)
plot_data = plot_data.copy()
plot_data['lat_jitter'] = plot_data['latency_steps'] + rng.uniform(-0.15, 0.15, len(plot_data))

pass_data = plot_data[plot_data['pass']]
fail_data = plot_data[~plot_data['pass']]

print(f"Total points: {len(plot_data)} ({len(pass_data)} pass, {len(fail_data)} fail)")

# Build figure
fig = go.Figure()

# Failing Kp values (red, smaller, semi-transparent)
fig.add_trace(go.Scatter3d(
    x=fail_data['td'],
    y=fail_data['lat_jitter'],
    z=fail_data['Kp'],
    mode='markers',
    marker=dict(
        size=3,
        color='red',
        opacity=0.25,
        line=dict(width=0),
    ),
    name='Kp fails (SR < 0.80)',
    hovertemplate=(
        'θ̈_max: %{x:.0f} rad/s²<br>'
        'Latency: %{customdata[0]} steps<br>'
        'Kp: %{z:.1f}<br>'
        'SR: %{customdata[1]:.2f}<br>'
        '<extra>FAIL</extra>'
    ),
    customdata=np.column_stack([fail_data['latency_steps'], fail_data['sr']]),
))

# Passing Kp values (green, larger, more opaque)
fig.add_trace(go.Scatter3d(
    x=pass_data['td'],
    y=pass_data['lat_jitter'],
    z=pass_data['Kp'],
    mode='markers',
    marker=dict(
        size=4,
        color='limegreen',
        opacity=0.75,
        line=dict(width=0),
    ),
    name='Kp passes (SR ≥ 0.80)',
    hovertemplate=(
        'θ̈_max: %{x:.0f} rad/s²<br>'
        'Latency: %{customdata[0]} steps<br>'
        'Kp: %{z:.1f}<br>'
        'SR: %{customdata[1]:.2f}<br>'
        '<extra>PASS</extra>'
    ),
    customdata=np.column_stack([pass_data['latency_steps'], pass_data['sr']]),
))

# Overlay the theoretical ceiling: Kp_max ≈ 380/latency (one line per latency level)
lat_vals = [1, 2, 3, 4, 5, 6]
td_range  = np.linspace(5, 400, 60)
for lat in lat_vals:
    kp_ceil = 380.0 / lat
    fig.add_trace(go.Scatter3d(
        x=td_range,
        y=np.full_like(td_range, lat),
        z=np.full_like(td_range, kp_ceil),
        mode='lines',
        line=dict(color='orange', width=3, dash='dash'),
        name=f'Ceiling (lat={lat})',
        showlegend=(lat == 1),
        hovertemplate=f'Ceiling Kp≈{kp_ceil:.0f} at lat={lat}<extra></extra>',
        legendgroup='ceiling',
    ))

fig.update_layout(
    title=dict(
        text='3D Gain Window: which Kp values work at each (authority, latency)?',
        font=dict(size=16),
    ),
    scene=dict(
        xaxis=dict(
            title='θ̈_max (rad/s²) — Authority',
            type='log',
            tickvals=[10, 30, 100, 300],
            ticktext=['10', '30', '100', '300'],
        ),
        yaxis=dict(
            title='Latency (steps)',
            tickvals=[1, 2, 3, 4, 5, 6],
        ),
        zaxis=dict(
            title='Kp',
            type='log',
            tickvals=[0.5, 1, 3, 10, 30, 100, 300, 800],
            ticktext=['0.5', '1', '3', '10', '30', '100', '300', '800'],
        ),
        camera=dict(
            eye=dict(x=1.5, y=-1.8, z=1.0),
        ),
        bgcolor='rgb(15, 15, 25)',
        xaxis_gridcolor='rgba(255,255,255,0.1)',
        yaxis_gridcolor='rgba(255,255,255,0.1)',
        zaxis_gridcolor='rgba(255,255,255,0.1)',
    ),
    paper_bgcolor='rgb(15, 15, 25)',
    font=dict(color='white', size=12),
    legend=dict(
        x=0.01, y=0.99,
        bgcolor='rgba(0,0,0,0.4)',
        bordercolor='rgba(255,255,255,0.2)',
        borderwidth=1,
    ),
    margin=dict(l=0, r=0, t=60, b=0),
    annotations=[
        dict(
            text=(
                '<b>Green</b> = SR ≥ 0.80 (valid gain)  |  '
                '<b>Red</b> = SR < 0.80 (too low or too high)  |  '
                '<b>Orange dashes</b> = theoretical ceiling Kp≈380/latency'
            ),
            xref='paper', yref='paper',
            x=0.5, y=0.01,
            xanchor='center', yanchor='bottom',
            font=dict(size=11, color='white'),
            showarrow=False,
        )
    ],
)

out_path = OUTPUTS / 'gain_window_3d.html'
fig.write_html(str(out_path), include_plotlyjs='cdn')
print(f"Saved: {out_path}")
print("\nDone.")
