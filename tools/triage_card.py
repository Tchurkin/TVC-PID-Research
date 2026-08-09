"""
tools/triage_card.py

Classify every flight-computer CSV in a directory as FLIGHT or GROUND/DRY RUN.

WHY: the SD card accumulates bench tests, pad dry runs and real flights side by side, with no RTC
and no marker distinguishing them. ASC037 sat in the repo unlogged and was briefly mistaken for a
static fire. Point this at the card before copying anything into `Rocket data/`.

    python tools/triage_card.py "Rocket data"
    python tools/triage_card.py E:/            # the card itself

WHAT DISCRIMINATES, and what does NOT
-------------------------------------
DECISIVE (all three agree on every file in the archive):
    altitude max        ground runs < 1 m      flights 12.8 - 45.7 m
    vertical velocity   ground runs < 1 m/s    flights 12.0 - 32.6 m/s
    samples above 1.5 g ground runs 0 - 1      flights 11 - 62

NOT RELIABLE -- do not use alone:
    peak AccelZ.        One knock gives 22 m/s^2. ASC037 has exactly ONE such sample and no motor.
    median |a|.         ASC031 is a REAL flight and reads 8.97 m/s^2 (~1 g) because its log spans
                        the coast, where the vehicle is near free-fall. ASC037 reads 8.89. These
                        are indistinguishable, so median |a| cannot separate flight from ground run.
                        (This tool exists partly to record that, after median |a| was written up as
                        the decisive test and turned out not to be.)

A static fire would look different again -- clamped, so altitude and velocity stay ~0 like a ground
run, but the thrust-sample count stays HIGH because the accelerometer reads the stand's reaction.
No such file exists in the archive; the branch is here so one would be recognised.
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

G        = 9.80665
ALT_MIN  = 5.0     # m    -- below this nothing left the pad
VZ_MIN   = 3.0     # m/s
HOT_MIN  = 3       # samples above 1.5 g; 1 is a knock, not a burn


def classify(path):
    try:
        d = pd.read_csv(path)
    except Exception:
        return None, 'unreadable (motor log / header-only capture?)', {}
    d.columns = [c.strip() for c in d.columns]
    if not {'AccelX', 'AccelY', 'AccelZ'} <= set(d.columns):
        return None, 'no IMU columns -- MTR/CTL capture, not a flight log', {}

    mag = np.sqrt(d.AccelX ** 2 + d.AccelY ** 2 + d.AccelZ ** 2)
    m = {
        'rows': len(d),
        'alt':  float(d['Altitude(m)'].max())   if 'Altitude(m)'  in d else float('nan'),
        'vz':   float(d['VertVel(m/s)'].max())  if 'VertVel(m/s)' in d else float('nan'),
        'hot':  int((d.AccelZ > 1.5 * G).sum()),
        'amed': float(mag.median()),
        'apk':  float(d.AccelZ.max()),
    }
    flew    = m['alt'] > ALT_MIN and m['vz'] > VZ_MIN and m['hot'] >= HOT_MIN
    clamped = (not flew) and m['hot'] >= HOT_MIN * 3
    if flew:      return True,  'FLIGHT', m
    if clamped:   return False, 'STATIC FIRE? clamped, sustained thrust -- verify by hand', m
    return False, 'ground / dry run -- never left the pad', m


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else 'Rocket data')
    # Windows globbing is case-insensitive, so '*.CSV' and '*.csv' return the same files.
    # Deduplicate on the resolved path rather than concatenating.
    files = sorted({p.resolve(): p for p in
                    list(root.glob('*.CSV')) + list(root.glob('*.csv'))}.values(),
                   key=lambda p: p.stem)
    if not files:
        print(f'no CSVs under {root}'); return 1

    print(f'{root}  --  {len(files)} file(s)\n')
    print(f"{'file':<12}{'rows':>6}{'alt_max':>9}{'vz_max':>8}{'n>1.5g':>8}"
          f"{'|a|med':>8}{'pkAccZ':>8}   verdict")
    print('-' * 96)
    flights = []
    for f in files:
        flew, verdict, m = classify(f)
        if not m:
            print(f'{f.stem:<12}{"":>39}   {verdict}'); continue
        print(f"{f.stem:<12}{m['rows']:>6}{m['alt']:>9.2f}{m['vz']:>8.2f}{m['hot']:>8}"
              f"{m['amed']:>8.2f}{m['apk']:>8.2f}   {verdict}")
        if flew:
            flights.append(f.stem)

    print(f"\n{len(flights)} flight(s): {', '.join(flights) if flights else '(none)'}")
    print("\nAny flight NOT already in FLIGHT_LOG.md needs a row -- and if it carries a labelled")
    print("outcome it can join the retrospective signature test, where n is what caps the p-value:")
    print("   2 healthy / 3 failed (today)  p_min = 0.100")
    print("   3 healthy / 3 failed          p_min = 0.050")
    print("   4 healthy / 4 failed          p_min = 0.014")
    return 0


if __name__ == '__main__':
    sys.exit(main())
