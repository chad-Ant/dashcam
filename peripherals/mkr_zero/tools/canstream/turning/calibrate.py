"""Time-weighted offline fits from externally labelled, complete manoeuvres."""
import dataclasses
import math
import statistics

from model import Profile, estimate


def integrate(rows, ref, profile):
    start, end = float(ref["start"]), float(ref["end"])
    if not all(math.isfinite(v) for v in (start, end)) or end - start < 5:
        raise ValueError("reference duration must be finite and >= 5 s")
    segment = int(ref["segment"])
    selected = [r for r in rows if int(r["segment"]) == segment and start <= float(r["time"]) <= end]
    if len(selected) < 20 or float(selected[0]["time"]) - start > .1 or end - float(selected[-1]["time"]) > .1:
        raise ValueError("reference lacks samples at its boundaries: " + ref["name"])
    # Boundaries must be actual sample times (<= 1 us), not silently trimmed.
    if abs(float(selected[0]["time"]) - start) > 1e-6 or abs(float(selected[-1]["time"]) - end) > 1e-6:
        raise ValueError("snap reference start/end to actual wheel-sample times")
    integral_mean = integral_diff = 0.0
    previous = None
    for row in selected:
        if row["status"] != "ok":
            raise ValueError("invalid sample inside reference: " + ref["name"])
        rl, rr = float(row["wheel_rl"]), float(row["wheel_rr"])
        if not all(math.isfinite(v) and 0 <= v <= 32767 for v in (rl, rr)):
            raise ValueError("invalid wheel counts")
        if estimate(rl, rr, profile)["status"] != "ok":
            raise ValueError("reference exceeds candidate envelope")
        point = float(row["time"]), (rl + rr) / 2, rr - rl
        if previous is not None:
            dt = point[0] - previous[0]
            if not 0 < dt <= profile.stale_s:
                raise ValueError("gap/duplicate/time step inside reference: " + ref["name"])
            integral_mean += dt * (point[1] + previous[1]) / 2
            integral_diff += dt * (point[2] + previous[2]) / 2
        previous = point
    return {"mean_integral": integral_mean, "diff_integral": integral_diff, "duration_s": end - start}


def reference_truth(ref):
    if ref["kind"] == "straight":
        distance = float(ref["distance_m"])
        yaw, radius = 0.0, None
    elif ref["kind"] in ("left", "right"):
        radius, laps = float(ref["radius_m"]), float(ref["laps"])
        if not math.isfinite(radius) or not 5 <= radius <= 100 or not math.isfinite(laps) or laps < 1 or laps != int(laps):
            raise ValueError("use measured rear-centre radius >= 5 m and whole laps")
        yaw = (1 if ref["kind"] == "left" else -1) * 360 * laps
        distance = 2 * math.pi * radius * laps
    else:
        raise ValueError("unknown reference kind")
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError("reference distance must be positive")
    return distance, yaw, radius


def fit(rows, refs, initial):
    if not refs or len(refs) > 100:
        raise ValueError("need 1..100 reference manoeuvres")
    names, intervals, items = set(), [], []
    for ref in refs:
        if not isinstance(ref.get("name"), str) or not ref["name"] or ref["name"] in names:
            raise ValueError("reference names must be unique")
        names.add(ref["name"])
        if ref["split"] not in ("fit", "validation"):
            raise ValueError("split must be fit or validation")
        interval = int(ref["segment"]), float(ref["start"]), float(ref["end"])
        if any(interval[0] == s and max(interval[1], a) < min(interval[2], b) for s, a, b in intervals):
            raise ValueError("reference windows overlap (fit/validation leakage)")
        intervals.append(interval)
        items.append((ref, integrate(rows, ref, initial), reference_truth(ref)))
    train = [item for item in items if item[0]["split"] == "fit"]
    for split in ("fit", "validation"):
        kinds = {r["kind"] for r, _, _ in items if r["split"] == split}
        if not {"straight", "left", "right"} <= kinds:
            raise ValueError("need separate straight, left and right runs in BOTH splits")
    straights = [(i, truth) for r, i, truth in train if r["kind"] == "straight"]
    epsilon = sum(i["diff_integral"] for i, _ in straights) / sum(i["mean_integral"] for i, _ in straights)
    # Each manoeuvre gets equal weight: long circles cannot dominate short ones.
    speed_scales = [truth[0] * 3.6 / i["mean_integral"] for _, i, truth in train]
    yaw_scales = []
    for ref, i, truth in train:
        if ref["kind"] == "straight":
            continue
        corrected = i["diff_integral"] - epsilon * i["mean_integral"]
        if corrected * truth[1] <= 0:
            raise ValueError("turn sign contradicts measured reference: " + ref["name"])
        yaw_scales.append(truth[1] / corrected)
    candidate = dataclasses.replace(initial, mismatch_ratio=epsilon,
                                    wheel_kmh_per_count=statistics.median(speed_scales),
                                    yaw_dps_per_count=statistics.median(yaw_scales))
    report = []
    for ref, i, truth in items:
        integrate(rows, ref, candidate)  # candidate must not invalidate its evidence
        distance = i["mean_integral"] * candidate.wheel_kmh_per_count / 3.6
        yaw = (i["diff_integral"] - candidate.mismatch_ratio * i["mean_integral"]) * candidate.yaw_dps_per_count
        predicted_radius = distance / abs(math.radians(yaw)) if abs(yaw) > .01 else None
        angle = math.degrees(math.atan(candidate.wheelbase_m / truth[2])) if truth[2] else 0
        predicted_angle = math.degrees(math.atan(candidate.wheelbase_m * math.radians(yaw) / distance))
        reference_angle = -angle if ref["kind"] == "right" else angle
        old_distance = i["mean_integral"] * initial.wheel_kmh_per_count / 3.6
        old_yaw = (i["diff_integral"] - initial.mismatch_ratio * i["mean_integral"]) * initial.yaw_dps_per_count
        report.append({"name": ref["name"], "split": ref["split"], "kind": ref["kind"],
                       "duration_s": i["duration_s"], "distance_error_pct": 100 * (distance / truth[0] - 1),
                       "yaw_integral_deg": yaw, "reference_yaw_deg": truth[1],
                       "mean_yaw_error_dps": (yaw - truth[1]) / i["duration_s"],
                       "radius_m": predicted_radius, "reference_radius_m": truth[2],
                       "angle_deg": predicted_angle, "reference_angle_deg": reference_angle,
                       "angle_error_deg": predicted_angle - reference_angle,
                       "baseline_distance_error_pct": 100 * (old_distance / truth[0] - 1),
                       "baseline_mean_yaw_error_dps": (old_yaw - truth[1]) / i["duration_s"]})
    return candidate, {"status": "CANDIDATE ONLY — inspect held-out errors; not auto-approved",
                       "fit_speed_scales": speed_scales, "fit_yaw_scales": yaw_scales, "runs": report}
