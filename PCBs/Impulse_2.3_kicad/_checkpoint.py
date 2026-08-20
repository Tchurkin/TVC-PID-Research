# Re-establish and record the board's verified state, then (optionally) commit it.
#
# WHY: the knowledge about this board lives in a conversation, and conversations end. Everything a
# future session needs to trust the board must be reproducible from the repo alone. This script
# regenerates the three things that can silently drift -- geom.json, DRC and ERC -- compares them
# against the accepted baseline, and stamps the result into CHECKPOINT.md.
#
# Run it after ANY change to the board, and before ordering.
#   python _checkpoint.py            # verify + update CHECKPOINT.md
#   python _checkpoint.py --commit   # ...and git-commit the result
import argparse, collections, datetime, json, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
CLI = r"C:/Program Files/KiCad/10.0/bin/kicad-cli.exe"
KPY = r"C:/Program Files/KiCad/10.0/bin/python.exe"
PCB = os.path.join(HERE, "Impulse_2.3.kicad_pcb")
SCH = os.path.join(HERE, "Impulse_2.3.kicad_sch")

# Accepted baseline. These match Impulse 2.2, which was fabricated and works; they are deliberate,
# not unfixed defects. Anything OUTSIDE this dict is a regression and fails the checkpoint.
ACCEPTED = {
    "courtyards_overlap": 20,     # TH connectors deliberately overlapping SMD courtyards, as on 2.2
    "pth_inside_courtyard": 14,   # same cause
    "lib_footprint_mismatch": 14, # local footprint edits vs stock lib -- cosmetic, pre-existing
    "silk_overlap": 11,           # cosmetic
    "silk_over_copper": 4,        # cosmetic
}
# 3 = GND pour Zone<->Zone fragments with ZERO stranded pads. Cosmetic; see CHECKPOINT.md.
ACCEPTED_UNCONNECTED = 3


def sh(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=HERE)
    return p.returncode, p.stdout + p.stderr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--commit", action="store_true", help="git-commit the checkpoint")
    ap.add_argument("--note", default="", help="one-line note for the checkpoint / commit")
    a = ap.parse_args()

    stamp = datetime.date.today().isoformat()
    ok = True
    lines = []

    def say(s):
        print(s)
        lines.append(s)

    say("Impulse 2.3 checkpoint -- %s" % stamp)
    say("")

    # ---- 1. geometry dump must be current, or every clearance number is stale ----
    # Only regenerate when actually stale: the dump is not atomic, so a needless rewrite can hand a
    # truncated geom.json to anything else reading it.
    gj = os.path.join(HERE, "geom.json")
    fresh = os.path.exists(gj) and os.path.getmtime(gj) >= os.path.getmtime(PCB)
    if not fresh:
        rc, out = sh([KPY, os.path.join(HERE, "_dump_geom.py"), PCB, gj])
        if rc != 0:
            say("geom.json   *** DUMP FAILED -- every clearance number below is stale ***")
            say(out.strip()[-400:])
            ok = False
    g = json.load(open(gj))
    say("geom.json   %d pads, %d tracks, %d vias  (%s)"
        % (len(g["pads"]), len(g["tracks"]), len(g["vias"]),
           "already current" if fresh else "regenerated from pcbnew"))

    # ---- 2. DRC with schematic parity ----
    drc = os.path.join(HERE, "drc_%s.json" % stamp)
    rc, out = sh([CLI, "pcb", "drc", "--schematic-parity", "--severity-all",
                  "--format", "json", "-o", drc, PCB])
    d = json.load(open(drc))
    counts = collections.Counter(v.get("type") for v in d["violations"])
    parity = len(d["schematic_parity"])
    unconn = len(d["unconnected_items"])

    say("")
    say("DRC   schematic parity : %d %s" % (parity, "OK" if parity == 0 else "*** REGRESSION ***"))
    if parity:
        ok = False
    say("      unconnected      : %d (accepted %d)%s"
        % (unconn, ACCEPTED_UNCONNECTED,
           "" if unconn <= ACCEPTED_UNCONNECTED else "   *** REGRESSION ***"))
    if unconn > ACCEPTED_UNCONNECTED:
        ok = False
    for t, n in sorted(counts.items()):
        base = ACCEPTED.get(t)
        if base is None:
            say("      %-22s: %d   *** NEW CLASS -- NOT ACCEPTED ***" % (t, n))
            ok = False
        elif n > base:
            say("      %-22s: %d (baseline %d)   *** REGRESSION ***" % (t, n, base))
            ok = False
        else:
            say("      %-22s: %d (baseline %d)  ok" % (t, n, base))
    for t, base in sorted(ACCEPTED.items()):
        if t not in counts:
            say("      %-22s: 0 (baseline %d)  improved" % (t, base))

    # ---- 3. ERC ----
    erc = os.path.join(HERE, "erc_%s.json" % stamp)
    rc, out = sh([CLI, "sch", "erc", "--severity-error", "--format", "json", "-o", erc, SCH])
    e = json.load(open(erc))
    nerr = sum(len(s.get("violations", [])) for s in e.get("sheets", []))
    say("")
    say("ERC   errors           : %d %s" % (nerr, "OK" if nerr == 0 else "*** REGRESSION ***"))
    if nerr:
        ok = False

    # ---- 4. fab package must be newer than the board, or it describes a different board ----
    say("")
    tb = os.path.getmtime(PCB)
    for f in ("fab/Impulse_2.3_BOM.csv", "fab/Impulse_2.3_CPL.csv", "fab/Impulse_2.3_gerbers.zip"):
        p = os.path.join(HERE, f)
        if not os.path.exists(p):
            say("fab   %-34s MISSING  ***" % f)
            ok = False
        elif os.path.getmtime(p) < tb - 1:
            say("fab   %-34s STALE -- older than the board  ***" % f)
            ok = False
        else:
            say("fab   %-34s current" % f)

    say("")
    say("RESULT: %s" % ("all checks at or better than baseline" if ok else "*** SOMETHING REGRESSED -- do not order ***"))
    if a.note:
        say("note: %s" % a.note)

    # ---- stamp into CHECKPOINT.md between the markers ----
    cp = os.path.join(HERE, "CHECKPOINT.md")
    block = "<!--VERIFY-->\n```\n%s\n```\n<!--/VERIFY-->" % "\n".join(lines)
    if os.path.exists(cp):
        t = open(cp, encoding="utf-8").read()
        if "<!--VERIFY-->" in t:
            pre = t.split("<!--VERIFY-->")[0]
            post = t.split("<!--/VERIFY-->", 1)[1]
            open(cp, "w", encoding="utf-8").write(pre + block + post)
            print("\nstamped into CHECKPOINT.md")

    if a.commit:
        msg = a.note or ("Impulse 2.3 checkpoint %s -- %s" % (stamp, "verified" if ok else "WIP, checks failing"))
        subprocess.run(["git", "add", "-A", "PCBs/Impulse_2.3_kicad"], cwd=ROOT)
        r = subprocess.run(["git", "commit", "-m", msg + "\n\nCo-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>"],
                           cwd=ROOT, capture_output=True, text=True)
        print(r.stdout + r.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
