#!/usr/bin/env python3
"""Plot / summarize a SIL flight CSV produced by the firmware harness.

Usage:  python plot_flight.py [flight.csv]
Works without matplotlib (prints a text summary); plots flight.png if matplotlib is present.
"""
import csv, sys

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "flight.csv"
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    if not rows:
        print("empty CSV"); return
    fl = lambda r, k: float(r[k])

    # phase transitions
    print(f"{len(rows)} samples, t {fl(rows[0],'t'):.1f} -> {fl(rows[-1],'t'):.1f} s")
    last = None
    for r in rows:
        if r["state"] != last:
            print(f"  t={fl(r,'t'):6.2f}s  -> {r['state']:6s}  alt={fl(r,'pz'):6.1f}  vz={fl(r,'vz'):6.1f}  tilt={fl(r,'th_deg'):5.1f}")
            last = r["state"]
    apogee = max(fl(r, "pz") for r in rows)
    td = rows[-1]
    # worst estimate error vs truth (position)
    ez = max(abs(fl(r, "estZ") - fl(r, "pz")) for r in rows)
    ex = max(abs(fl(r, "estX") - fl(r, "px")) for r in rows)
    print(f"\napogee={apogee:.1f} m | touchdown x={fl(td,'px'):.2f} m, vz={fl(td,'vz'):.2f} m/s, tilt={fl(td,'th_deg'):.1f} deg")
    print(f"max EKF error: altitude {ez:.2f} m, horizontal {ex:.2f} m")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("\n(matplotlib not available — text summary only)")
        return

    t = [fl(r, "t") for r in rows]
    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    ax[0].plot(t, [fl(r,"pz") for r in rows], label="altitude (true)")
    ax[0].plot(t, [fl(r,"estZ") for r in rows], "--", label="altitude (EKF)")
    ax[0].set_ylabel("z (m)"); ax[0].legend(); ax[0].grid(alpha=.3)
    ax[1].plot(t, [fl(r,"th_deg") for r in rows], label="tilt (true)")
    ax[1].plot(t, [fl(r,"thEst_deg") for r in rows], "--", label="tilt (gyro est)")
    ax[1].set_ylabel("pitch (deg)"); ax[1].legend(); ax[1].grid(alpha=.3)
    ax[2].plot(t, [fl(r,"vz") for r in rows], label="vz (true)")
    ax[2].plot(t, [fl(r,"estVz") for r in rows], "--", label="vz (EKF)")
    ax[2].set_ylabel("vz (m/s)"); ax[2].legend(); ax[2].grid(alpha=.3)
    ax[3].plot(t, [fl(r,"uTVC") for r in rows], label="TVC cmd")
    ax[3].plot(t, [fl(r,"deploy") for r in rows], label="margin deploy")
    ax[3].plot(t, [fl(r,"keff") for r in rows], label="keff (RLS)")
    ax[3].set_ylabel("control"); ax[3].set_xlabel("t (s)"); ax[3].legend(); ax[3].grid(alpha=.3)
    fig.tight_layout(); fig.savefig("flight.png", dpi=110)
    print("wrote flight.png")

if __name__ == "__main__":
    main()
