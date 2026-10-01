"""ab_report.py <trial dir> [--json]: score a trial_ab.sh run (A-B-A sessions NN_<profile>/).

Run in the l4t-ml-gpio container with the workspace at /w:
  docker run --rm --user 1000:1000 -v ~/drive_logs/tools/isp_tuning:/w -w /w l4t-ml-gpio:latest \
      python3 ab_report.py trial_<stamp> --json
Per session and variant (unset, base, ee020, ee_default; 5 frames each), means over the valid frames: a frame
counts only if all four markers are found in both images and every metric is finite (a truncated JPEG, an
unusable border or a NaN metric drops that frame, with a note on stderr):
  LO under / over   dark rim / bright halo on the low-contrast grey edges, % of step (chartsharp, remap ESF)
  m.25              MTF at 0.25 cy/px of those edges (1 = no boost)
  Ystd              plane-fit luma std in grey patches 20-22 (noise; expanded limited range)
  dE                mean dE76 of the 18 colour patches, CSI vs UGREEN (correct CSI decode)
  paper L*          CSI / UGREEN
Then each candidate session is judged against the base-profile sessions just before and after it (those
with valid frames):
  sharpening at the ISP default (variant ee_default = ee-mode 1, strength -1: the table path an app that sets
  no ee property gets): under <= 20, over <= 8, m.25 <= 1.15, Ystd <= 1.10 x the same session's ee-mode=0
  (the 2026-10-01 review's criteria);
  colour unchanged: |dE(base variant) - dE(base variant of the neighbours)| <= 0.5;
  parse: no config-loader lines in the saved nvargus journals (NN_<profile>.nvargus.log) of the candidate and
  its neighbours. libnvscf skips a statement with an unknown key and logs it; a bad value makes the whole
  override load fail. A missing journal counts as unchecked = failed.
Informational: manual ee-strength 0.2 (variant ee020). A table-only override leaves it as on the base
profile (~22 % halo); one that disables the sharpen block (sharpness.v5.enable = FALSE) brings it to ~ee-off.
Verdicts are keyed by session (NN_<profile>), so a candidate listed twice keeps both. Sessions that did not
complete are named on the verdict line. --json writes report.json (strict JSON) into the trial directory.
"""
import json
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from chartcmp import measure
from chartsharp import analyse

VARIANTS = ("unset", "base", "ee020", "ee_default")
LIMITS = dict(under=20.0, over=8.0, m25=1.15, ystd_ratio=1.10, colour=0.5)
# libnvscf (R36.5.2) prints these via NvOsDebugPrintf when it skips or rejects a camera_overrides.isp statement
PARSE_MARKERS = ("NvCameraIspConfigFileLoad", "NvCameraIspLoadAllConfigFiles", "Failed to load all config files")


def parse_errors(step_dir):
    """Config-loader lines in the journal trial_ab.sh saved next to the session; None if there is none."""
    log = step_dir.parent / f"{step_dir.name}.nvargus.log"
    if not log.exists():
        return None
    return sorted({ln.split(": ", 1)[-1].strip() for ln in log.read_text(errors="replace").splitlines()
                   if any(m in ln for m in PARSE_MARKERS)})


def frame_metrics(d, name):
    try:
        a, b = measure(str(d / f"{name}.csi.jpg")), measure(str(d / f"{name}.ug.jpg"))
        if a is None or b is None or a[2] != "4m" or b[2] != "4m":
            return None
        s = analyse(str(d / f"{name}.csi.jpg"))
    except (OSError, ValueError) as e:      # truncated/corrupt JPEG; no usable LO border (empty concatenate)
        print(f"{d.name}/{name}: frame dropped ({type(e).__name__}: {e})", file=sys.stderr)
        return None
    if s is None:
        return None
    lo = s["lo"]
    row = dict(under=float(lo["metr"][2]), over=float(lo["metr"][1]), m25=float(lo["mtf"][2]),
               ystd=float(s["nz"]["sd"]), dE=float(np.linalg.norm(a[0][:18] - b[0][:18], axis=1).mean()),
               paperL=float(a[1][0]), ugL=float(b[1][0]))
    if not all(np.isfinite(v) for v in row.values()):   # esf_metrics gives NaN without a clean 50 % crossing
        print(f"{d.name}/{name}: frame dropped (non-finite metric)", file=sys.stderr)
        return None
    return row


def session(d):
    meta = json.loads((d / "session.json").read_text())
    out = dict(complete=bool(meta.get("complete")) and meta.get("isp_unchanged", True) is not False,
               isp_sha256=meta.get("isp_sha256", "")[:12], variants={}, frames={}, parse_errors=parse_errors(d))
    for v in VARIANTS:
        names = [c["name"] for c in meta.get("captures", []) if c["name"].rsplit("_", 1)[0] == v]
        rows = [r for r in (frame_metrics(d, n) for n in names) if r is not None]
        out["frames"][v] = [len(rows), len(names)]
        if rows:
            out["variants"][v] = {k: [float(np.mean([r[k] for r in rows])), float(np.std([r[k] for r in rows]))]
                                  for k in rows[0]} | {"n": len(rows), "of": len(names)}
    return out


def fmt(v, f):
    return "n/a" if v is None else format(v, f)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    t = pathlib.Path(sys.argv[1])
    steps = sorted(p for p in t.iterdir() if p.is_dir() and p.name[:2].isdigit() and (p / "session.json").exists())
    if not steps:
        sys.exit(f"no NN_<profile>/session.json in {t}")
    prof_of = {p.name: p.name.split("_", 1)[1] for p in steps}
    base_name = prof_of[steps[0].name]               # trial_ab.sh always starts with the base profile
    res = {p.name: session(p) for p in steps}
    print(f"{'session':24s} {'variant':10s} {'n':>4s} {'under':>6s} {'over':>6s} {'m.25':>5s} {'Ystd':>5s} {'dE':>5s} {'paperL':>11s}")
    for name, r in res.items():
        flag = "" if r["complete"] else "  (INCOMPLETE session)"
        for v in VARIANTS:
            got, of = r["frames"][v]
            if v in r["variants"]:
                m = r["variants"][v]
                print(f"{name:24s} {v:10s} {got}/{of} {m['under'][0]:6.1f} {m['over'][0]:6.1f} {m['m25'][0]:5.2f} "
                      f"{m['ystd'][0]:5.2f} {m['dE'][0]:5.2f} {m['paperL'][0]:5.1f}/{m['ugL'][0]:4.1f}{flag}")
            elif of:
                print(f"{name:24s} {v:10s} {got}/{of}  no valid frames{flag}")
    for name, r in res.items():
        if r["parse_errors"] is None:
            print(f"{name}: no {name}.nvargus.log, so the override parse is unchecked (counts as a failed 'parse' check)")
        elif r["parse_errors"]:
            print(f"{name}: camera_overrides.isp NOT applied as written:\n    " + "\n    ".join(r["parse_errors"]))
    print()
    verdicts = {}
    names = list(res)
    for i, name in enumerate(names):
        prof = prof_of[name]
        if prof == base_name:
            continue
        r = res[name]["variants"]
        adj = [names[j] for j in (i - 1, i + 1) if 0 <= j < len(names) and prof_of[names[j]] == base_name]
        nbn = [n for n in adj if "base" in res[n]["variants"]]
        nb = [res[n]["variants"] for n in nbn]
        missing = [f"{name} {v}" for v in ("base", "ee_default") if v not in r] + ([] if nb else ["a neighbouring base session"])
        if missing:
            verdicts[name] = dict(profile=prof, verdict="NO DATA", missing=missing)
            print(f"{name}: NO DATA (no valid frames for: {', '.join(missing)})"); continue
        e, b0 = r["ee_default"], r["base"]
        ratio = e["ystd"][0] / b0["ystd"][0]
        ref_dE = float(np.mean([x["base"]["dE"][0] for x in nb]))
        ref_ee = {k: float(np.mean(v)) if (v := [x["ee_default"][k][0] for x in nb if "ee_default" in x]) else None
                  for k in ("under", "over", "m25", "ystd")}
        used = [name] + nbn
        checks = dict(under=e["under"][0] <= LIMITS["under"], over=e["over"][0] <= LIMITS["over"],
                      m25=e["m25"][0] <= LIMITS["m25"], ystd=ratio <= LIMITS["ystd_ratio"],
                      colour=abs(b0["dE"][0] - ref_dE) <= LIMITS["colour"],
                      parse=all(res[n]["parse_errors"] == [] for n in used))
        ok = all(checks.values())
        incomplete = [n for n in used if not res[n]["complete"]]
        manual = ""
        if "ee020" in r and (m_nb := [x["ee020"]["over"][0] for x in nb if "ee020" in x]):
            manual = (f"   manual ee-strength 0.2: over {r['ee020']['over'][0]:.1f} (base profile {np.mean(m_nb):.1f},"
                      f" ee-off {b0['over'][0]:.1f})")
        verdicts[name] = dict(profile=prof, verdict="PASS" if ok else "FAIL", checks=checks, ee_default=e,
                              ystd_ratio=ratio, colour_dE=b0["dE"][0], colour_dE_neighbours=ref_dE,
                              base_profile_ee_default=ref_ee, neighbours=nbn, incomplete_sessions=incomplete,
                              ee020=r.get("ee020"))
        print(f"{name}: {'PASS' if ok else 'FAIL'}   ISP-default sharpening: under {e['under'][0]:.1f} (<= {LIMITS['under']:.0f};"
              f" base profile {fmt(ref_ee['under'], '.1f')}), over {e['over'][0]:.1f} (<= {LIMITS['over']:.0f};"
              f" {fmt(ref_ee['over'], '.1f')}), m.25 {e['m25'][0]:.2f} (<= {LIMITS['m25']}; {fmt(ref_ee['m25'], '.2f')}),"
              f" Ystd {ratio:.2f}x ee-off (<= {LIMITS['ystd_ratio']})   colour dE {b0['dE'][0]:.2f} vs neighbours"
              f" {ref_dE:.2f} (|diff| <= {LIMITS['colour']})" + manual
              + ("" if ok else "   failed: " + ", ".join(k for k, v in checks.items() if not v))
              + (f"   [INCOMPLETE session(s): {', '.join(incomplete)}]" if incomplete else ""))
    if "--json" in sys.argv[2:]:
        (t / "report.json").write_text(json.dumps(dict(sessions=res, verdicts=verdicts, limits=LIMITS), indent=2,
                                                  allow_nan=False))
        print(f"wrote {t / 'report.json'}")


if __name__ == "__main__":
    main()
