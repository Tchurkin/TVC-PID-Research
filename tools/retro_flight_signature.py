"""
tools/retro_flight_signature.py

THE RETROSPECTIVE FLIGHT-SIGNATURE TEST.
Pre-registration: paper/RETRO_FLIGHT_SIG_SPEC.md (written and committed BEFORE this ran, 5f49641).

FROZEN RULE -- do not edit these three constants to make a result come out.
    Predict FAILING if max(RMS_x, RMS_y) >= 5.62 deg over the boost window, reference 0 deg.
    Grey zone [5.37, 5.62]: report INDETERMINATE, do not classify.
Both numbers are Youden-optimal cuts derived from the SIMULATION study (5.62 from the published
flight_sig_final_py.csv, 5.37 from the same protocol re-run on today's simulator). Zero parameters
are fit on flight data. The second feature (saturation) was DELETED at spec time -- u_cmd_sat_frac is
identically zero across all 504 sim runs, so no sim-derived threshold for it exists. Saturation is
reported below as DESCRIPTIVE ONLY and takes no part in any classification.

POWER CEILING, stated before running: 5 flights, 2 controlled. Perfect classification gives a
one-sided exact p of 1/C(5,2) = 0.10. This test can falsify C-FLIGHT; it cannot confirm it.

COLUMN SEMANTICS (verified against Firmware/Ascent_TVC/Ascent_TVC.ino, logData() @ line 1194):
    GyroX, GyroY  = gyro_x, gyro_y -- the TILT ESTIMATE in DEGREES (misleading column names)
    GyroZ         = gyro_z          -- integrated ROLL ANGLE in degrees
    AngVelX/Y     = ang_vel_x/y     -- filtered body rates, deg/s, where raw_avy = -imu_gy
    TVCx/TVCy     = COMMANDED TVC angle in deg, POST-clamp (already divided by SERVO_*_MULT)
"""

import sys, os, math
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'Firmware', 'sim_ascent'))
import io as _io, contextlib as _ctx
with _ctx.redirect_stdout(_io.StringIO()):                # quat_validate runs a demo on import
    from quat_validate import Estimator                   # verified firmware port

# ── THE FROZEN RULE ───────────────────────────────────────────────────────────
THRESH      = 5.62      # deg -- published sim Youden cut
GREY_LO     = 5.37      # deg -- same cut re-derived on today's simulator
MAX_TILT    = 5.0       # deg -- firmware clamp, all flights
SAT_FRAC    = 0.95      # a row counts as saturated at >= 0.95 * MAX_TILT
G           = 9.80665
ONSET_G     = 1.5       # thrust-onset threshold for logs with no Phase column

DATA = Path('Rocket data')

# Labels: from documented OUTCOME in FLIGHT_LOG.md only. Not from tilt, not from saturation.
FLIGHTS = [
    # id,        label(1=failing),  note
    ('ASC036',   0, 'PARTIAL - controlled boost, log ends at burnout'),
    ('ASC031',   0, 'SUCCESS - first controlled boost'),
    ('ASC007',   1, 'FAILURE - self-aborted mid-boost'),
    ('LOG001',   1, 'FAILURE - diverged, gimbal pinned'),
    ('ASC038',   1, 'FAILURE - uncontrolled, estimator froze'),
]
# Pipeline controls -- NOT part of the test, no labels, excluded from every statistic.
CONTROLS = [('RES005', 'real flight, TVC DISABLED -- saturation must be 0.000'),
            ('ASC037', 'GROUND HANDLING TEST, NO MOTOR -- |a| median 8.89 = 1g vs 11.96 in flight; '
                       'exactly 1 sample >1.5g; alt max 0.78 m. Specificity warning: hand motion '
                       'alone scores 7.6 deg, above the threshold. The signature is only meaningful '
                       'inside a real boost.')]


def load(fid):
    d = pd.read_csv(DATA / f'{fid}.CSV')
    d.columns = [c.strip() for c in d.columns]
    # LOG001/RES005 schema: ServoX/ServoY instead of TVCx/TVCy, and no Phase column
    if 'TVCx(deg)' not in d.columns:
        d = d.rename(columns={'ServoX': 'TVCx(deg)', 'ServoY': 'TVCy(deg)'})
    d['t'] = (d['Time(ms)'] - d['Time(ms)'].iloc[0]) / 1000.0
    return d


def boost_mask(d, fid):
    """Powered-flight rows. Phase==1 where the column exists, thrust onset otherwise."""
    if 'Phase' in d.columns:
        m = (d['Phase'] == 1).values
        if m.sum() >= 5:
            return m, 'Phase==1'
    hot = (d['AccelZ'] > ONSET_G * G).values
    if hot.sum() < 3:
        return np.zeros(len(d), bool), 'NO BURN FOUND'
    i0, i1 = np.argmax(hot), len(hot) - 1 - np.argmax(hot[::-1])
    m = np.zeros(len(d), bool); m[i0:i1 + 1] = True
    return m, f'axial>{ONSET_G}g'


def replay_attitude(d):
    """Re-integrate tilt with the firmware's roll-aware quaternion, from logged rates.

    imu_gx  ~  AngVelX          (ANGVEL_ALPHA=0.9 at 20 Hz is near pass-through)
    imu_gy  ~ -AngVelY          (firmware sets raw_avy = -imu_gy)
    imu_gz  ~  d(GyroZ)/dt      (roll RATE is not logged; roll ANGLE is, so difference it)

    Open-loop integration of filtered, decimated data -- drift is real and unquantified.
    """
    est = Estimator('quat')
    est.from_accel(d['AccelX'].iloc[0], d['AccelY'].iloc[0], d['AccelZ'].iloc[0])
    t    = d['t'].values
    roll = d['GyroZ'].values
    gx, gy = [], []
    for k in range(len(d)):
        dt = t[k] - t[k - 1] if k else 0.0
        if 0 < dt < 0.5:
            gz = (roll[k] - roll[k - 1]) / dt
            est.propagate(d['AngVelX'].values[k], d['AngVelY'].values[k], gz, dt)
        ubx, uby, ubz = est.up_in_body()
        gx.append(math.atan2(uby, ubz) * 180 / math.pi)
        gy.append(-math.atan2(ubx, ubz) * 180 / math.pi)
    return np.array(gx), np.array(gy)


def rms(v):
    return float(np.sqrt(np.mean(np.asarray(v, float) ** 2)))


def verdict(score):
    if score >= THRESH:  return 'FAILING'
    if score >= GREY_LO: return 'INDETERMINATE'
    return 'HEALTHY'


def analyse(fid):
    d = load(fid)
    m, how = boost_mask(d, fid)
    out = {'id': fid, 'n_boost': int(m.sum()), 'window': how,
           'dur': float(d['t'][m].max() - d['t'][m].min()) if m.sum() else 0.0,
           'hz': float(m.sum() / max(d['t'][m].max() - d['t'][m].min(), 1e-9)) if m.sum() else 0.0}
    if not m.sum():
        return out

    out['rms_x_log'] = rms(d['GyroX'].values[m])
    out['rms_y_log'] = rms(d['GyroY'].values[m])
    rx, ry = replay_attitude(d)
    out['rms_x_rep'] = rms(rx[m])
    out['rms_y_rep'] = rms(ry[m])
    out['score_log'] = max(out['rms_x_log'], out['rms_y_log'])
    out['score_rep'] = max(out['rms_x_rep'], out['rms_y_rep'])
    out['pred_log']  = verdict(out['score_log'])
    out['pred_rep']  = verdict(out['score_rep'])

    tv = np.maximum(d['TVCx(deg)'].values[m].__abs__(), d['TVCy(deg)'].values[m].__abs__())
    out['sat'] = float(np.mean(tv >= SAT_FRAC * MAX_TILT))     # DESCRIPTIVE ONLY
    out['roll_span'] = float(d['GyroZ'].values[m].max() - d['GyroZ'].values[m].min())
    return out


def confusion(rows, key):
    tp = sum(1 for r in rows if r['lab'] == 1 and r[key] == 'FAILING')
    fn = sum(1 for r in rows if r['lab'] == 1 and r[key] == 'HEALTHY')
    fp = sum(1 for r in rows if r['lab'] == 0 and r[key] == 'FAILING')
    tn = sum(1 for r in rows if r['lab'] == 0 and r[key] == 'HEALTHY')
    ind = sum(1 for r in rows if r[key] == 'INDETERMINATE')
    p = stats.fisher_exact([[tp, fn], [fp, tn]], alternative='greater')[1]
    return tp, fn, fp, tn, ind, p


def main():
    print("=" * 78)
    print("RETROSPECTIVE FLIGHT-SIGNATURE TEST")
    print(f"FROZEN RULE: FAILING if max(RMS_x, RMS_y) >= {THRESH} deg   "
          f"grey zone [{GREY_LO}, {THRESH}]")
    print("Sim reference: FRAGILE 13.3 +- 5.2 deg | EASY 3.8 +- 2.7 deg")
    print("=" * 78)

    rows = []
    for fid, lab, note in FLIGHTS:
        r = analyse(fid); r['lab'] = lab; r['note'] = note
        rows.append(r)

    print("\n-- BOOST WINDOWS " + "-" * 60)
    print(f"{'flight':8s} {'rows':>5s} {'dur_s':>6s} {'Hz':>5s}  {'window':<12s} roll_span")
    for r in rows:
        print(f"{r['id']:8s} {r['n_boost']:5d} {r['dur']:6.2f} {r['hz']:5.1f}  "
              f"{r['window']:<12s} {r['roll_span']:7.1f} deg")

    print("\n-- RMS ATTITUDE, PER AXIS, OVER THE BOOST (deg) " + "-" * 30)
    print(f"{'flight':8s} {'truth':>9s} | {'as-logged':>21s} | {'roll-aware replay':>21s}")
    print(f"{'':8s} {'':>9s} | {'RMSx':>6s} {'RMSy':>6s} {'->':>7s} | "
          f"{'RMSx':>6s} {'RMSy':>6s} {'->':>7s}")
    for r in rows:
        truth = 'FAILING' if r['lab'] else 'ok'
        print(f"{r['id']:8s} {truth:>9s} | {r['rms_x_log']:6.2f} {r['rms_y_log']:6.2f} "
              f"{r['pred_log'][:7]:>7s} | {r['rms_x_rep']:6.2f} {r['rms_y_rep']:6.2f} "
              f"{r['pred_rep'][:7]:>7s}")

    for key, name in (('pred_rep', 'PRIMARY -- roll-aware replay'),
                      ('pred_log', 'SECONDARY -- as-logged (what the flight computer believed)')):
        tp, fn, fp, tn, ind, p = confusion(rows, key)
        print(f"\n-- {name} " + "-" * max(4, 58 - len(name)))
        print(f"   TP {tp}  FN {fn}  FP {fp}  TN {tn}  INDETERMINATE {ind}")
        print(f"   one-sided exact p = {p:.3f}   (ceiling for this archive = 0.100)")

    print("\n-- SATURATION (DESCRIPTIVE ONLY -- not part of any rule) " + "-" * 20)
    print("   sim u_cmd_sat_frac is identically 0.000, so there is no sim threshold to compare to")
    for r in rows:
        print(f"   {r['id']:8s} {r['sat']*100:5.1f}% of boost rows at >= "
              f"{SAT_FRAC*MAX_TILT:.2f} deg")

    print("\n-- PIPELINE CONTROLS " + "-" * 56)
    for fid, note in CONTROLS:
        try:
            c = analyse(fid)
            sat = c.get('sat', float('nan'))
            print(f"   {fid:8s} rows {c['n_boost']:3d}  window {c['window']:<12s} "
                  f"sat {sat*100:5.1f}%   [{note}]")
        except Exception as e:
            print(f"   {fid:8s} FAILED: {e}")

    pd.DataFrame(rows).to_csv('paper/retro_flight_signature.csv', index=False)
    print("\nwrote paper/retro_flight_signature.csv")


if __name__ == '__main__':
    main()
