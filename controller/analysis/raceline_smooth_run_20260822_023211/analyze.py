#!/usr/bin/env python3
"""Compare the clean second lap of V3pro racelinev3 and raceline_smooth."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np


OUT = Path("/tmp/raceline_smooth_run_20260822_023211")
TRACK_LENGTH = 87.759715270

RUNS = {
    "V3pro original": {
        "telemetry": OUT / "baseline_v3pro_20260822_015637/hardware_stable_v3pro_20260822_015637.jsonl",
        "laps": OUT / "baseline_v3pro_20260822_015637/laps_stable_v3pro_20260822_015637.csv",
        "color": "#6b7280",
    },
    "V3pro raceline_smooth": {
        "telemetry": OUT / "hardware_stable_v3pro_raceline_smooth_20260822_023211.jsonl",
        "laps": OUT / "laps_stable_v3pro_raceline_smooth_20260822_023211.csv",
        "color": "#e11d48",
    },
}


def read_track(name):
    path = OUT / "tracks" / name / "raceline.csv"
    rows = list(csv.DictReader(path.open()))
    numeric = ["s_m", "x_m", "y_m", "psi_rad", "w_tr_right_m", "w_tr_left_m"]
    return {key: np.asarray([float(row[key]) for row in rows]) for key in numeric}


def read_run(spec):
    records = [json.loads(line) for line in spec["telemetry"].open()]
    laps = list(csv.DictReader(spec["laps"].open()))
    # Lap 1 contains launch/recovery. Lap 3 in the original run contains a stop/carry.
    # The interval between crossings 1 and 2 is the only clean, full, common lap.
    start = float(laps[0]["odom_crossing_stamp_s"])
    end = float(laps[1]["odom_crossing_stamp_s"])
    records = [row for row in records if start <= row["timestamp"] <= end]
    t = np.asarray([row["timestamp"] for row in records])
    dt = np.diff(t, prepend=t[0])
    if len(dt) > 1:
        dt[0] = np.median(dt[1:])
    diag = {key: np.asarray([row["diagnostic"][key] for row in records])
            for key in records[0]["diagnostic"]}
    steering = diag["published_steering"]
    steering_rate = np.zeros_like(steering)
    steering_rate[1:] = np.diff(steering) / np.diff(t)
    vx_pos = np.maximum(diag["vx"], 0.0)
    ds = np.zeros_like(t)
    ds[1:] = 0.5 * (vx_pos[1:] + vx_pos[:-1]) * np.diff(t)
    s_raw = np.cumsum(ds)
    s = s_raw * (TRACK_LENGTH / s_raw[-1])
    return {
        "records": records,
        "laps": laps,
        "lap": laps[1],
        "start": start,
        "end": end,
        "t": t,
        "t_rel": t - start,
        "dt": dt,
        "d": diag,
        "steering_rate": steering_rate,
        "s": s,
        "integrated_distance_m": float(s_raw[-1]),
    }


def intervals(run, mask, merge_gap=0.13):
    idx = np.flatnonzero(mask)
    if not len(idx):
        return []
    groups = []
    first = last = idx[0]
    for current in idx[1:]:
        if run["t"][current] - run["t"][last] <= merge_gap:
            last = current
        else:
            groups.append((first, last))
            first = last = current
    groups.append((first, last))
    return groups


def metrics(run):
    d = run["d"]
    dt = run["dt"]
    pp = d["candidate_active"] > 0.5
    risk = d["prediction_risk"] > 0.5
    slack = d["track_slack"] > 1e-3
    abs_contour = np.abs(d["contour_error"])
    abs_heading = np.abs(d["heading_error"])
    sr = np.abs(run["steering_rate"][1:])
    return {
        "lap_time_s": float(run["lap"]["lap_time_s"]),
        "lap_mean_speed_mps": float(run["lap"]["mean_speed_mps"]),
        "lap_max_speed_mps": float(run["lap"]["max_speed_mps"]),
        "contour_mae_m": float(np.mean(abs_contour)),
        "contour_p95_m": float(np.quantile(abs_contour, 0.95)),
        "contour_max_m": float(np.max(abs_contour)),
        "heading_mae_rad": float(np.mean(abs_heading)),
        "heading_p95_rad": float(np.quantile(abs_heading, 0.95)),
        "heading_max_rad": float(np.max(abs_heading)),
        "pp_active_s": float(np.sum(dt[pp])),
        "pp_active_fraction": float(np.sum(dt[pp]) / np.sum(dt)),
        "pp_events": len(intervals(run, pp)),
        "risk_active_s": float(np.sum(dt[risk])),
        "risk_fraction": float(np.sum(dt[risk]) / np.sum(dt)),
        "risk_events": len(intervals(run, risk)),
        "minimum_margin_q05_m": float(np.quantile(d["mpcc_minimum_margin"], 0.05)),
        "minimum_margin_min_m": float(np.min(d["mpcc_minimum_margin"])),
        "track_slack_over_1mm_fraction": float(np.sum(dt[slack]) / np.sum(dt)),
        "track_slack_max_m": float(np.max(d["track_slack"])),
        "steering_abs_p95_rad": float(np.quantile(np.abs(d["published_steering"]), 0.95)),
        "steering_rate_abs_mean_radps": float(np.mean(sr)),
        "steering_rate_abs_p95_radps": float(np.quantile(sr, 0.95)),
        "steering_rate_abs_max_radps": float(np.max(sr)),
        "solve_mean_ms": float(1000.0 * np.mean(d["solve_time"])),
        "solve_p95_ms": float(1000.0 * np.quantile(d["solve_time"], 0.95)),
        "solve_max_ms": float(1000.0 * np.max(d["solve_time"])),
        "solver_failure_samples": int(np.sum(d["solver_success"] < 0.5)),
        "control_rate_hz": float(1.0 / np.median(np.diff(run["t"]))),
        "integrated_distance_m": run["integrated_distance_m"],
    }


original_track = read_track("racelinev3")
smooth_track = read_track("raceline_smooth")
data = {name: read_run(spec) for name, spec in RUNS.items()}
summary = {name: metrics(run) for name, run in data.items()}

# Event table, explicitly separating PP from prediction-only warnings.
event_rows = []
for name, run in data.items():
    for event_type, mask in (
        ("PP/recovery", run["d"]["candidate_active"] > 0.5),
        ("prediction_risk", run["d"]["prediction_risk"] > 0.5),
        ("track_slack_gt_1mm", run["d"]["track_slack"] > 1e-3),
    ):
        for number, (a, b) in enumerate(intervals(run, mask), 1):
            reasons = sorted({int(v) for v in run["d"]["candidate_reason_code"][a:b + 1]})
            event_rows.append({
                "run": name,
                "event": event_type,
                "number": number,
                "start_s": float(run["t_rel"][a]),
                "end_s": float(run["t_rel"][b]),
                "duration_s": float(np.sum(run["dt"][a:b + 1])),
                "estimated_track_s_start_m": float(run["s"][a]),
                "estimated_track_s_end_m": float(run["s"][b]),
                "estimated_x_m": float(np.interp(np.mean(run["s"][a:b + 1]), smooth_track["s_m"], smooth_track["x_m"])),
                "estimated_y_m": float(np.interp(np.mean(run["s"][a:b + 1]), smooth_track["s_m"], smooth_track["y_m"])),
                "candidate_reason_codes": ";".join(map(str, reasons)),
                "max_abs_contour_error_m": float(np.max(np.abs(run["d"]["contour_error"][a:b + 1]))),
                "max_track_slack_m": float(np.max(run["d"]["track_slack"][a:b + 1])),
            })

with (OUT / "metrics.json").open("w") as handle:
    json.dump({"selection": "clean lap 2 only", "runs": summary}, handle, indent=2)
with (OUT / "events.csv").open("w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=list(event_rows[0]))
    writer.writeheader()
    writer.writerows(event_rows)


def boundary(track, side):
    if side == "left":
        width = track["w_tr_left_m"]
        return track["x_m"] - np.sin(track["psi_rad"]) * width, track["y_m"] + np.cos(track["psi_rad"]) * width
    width = track["w_tr_right_m"]
    return track["x_m"] + np.sin(track["psi_rad"]) * width, track["y_m"] - np.cos(track["psi_rad"]) * width


fig, axes = plt.subplots(3, 2, figsize=(16, 17), constrained_layout=True)
fig.suptitle("V3pro boundary smoothing: clean full-lap comparison", fontsize=18, fontweight="bold")

for ax, (name, run) in zip(axes[0], data.items()):
    lx, ly = boundary(smooth_track, "left")
    rx, ry = boundary(smooth_track, "right")
    ax.plot(lx, ly, color="#facc15", lw=1.5, label="smoothed safety boundary")
    ax.plot(rx, ry, color="#facc15", lw=1.5)
    s = run["s"]
    x = np.interp(s, smooth_track["s_m"], smooth_track["x_m"])
    y = np.interp(s, smooth_track["s_m"], smooth_track["y_m"])
    pts = np.column_stack((x, y))
    segments = np.stack((pts[:-1], pts[1:]), axis=1)
    lc = LineCollection(segments, cmap="viridis", norm=plt.Normalize(0.0, 0.45), linewidth=4)
    lc.set_array(np.abs(run["d"]["contour_error"][:-1]))
    ax.add_collection(lc)
    pp_groups = intervals(run, run["d"]["candidate_active"] > 0.5)
    for idx, (a, b) in enumerate(pp_groups):
        mid = (a + b) // 2
        ax.scatter(x[mid], y[mid], marker="X", s=150, color="#ef4444", edgecolor="white", zorder=5,
                   label="PP/recovery takeover" if idx == 0 else None)
    ax.set_title(f"{name}\ncolor = |contour error|, red X = actual PP/recovery")
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)
    ax.legend(loc="best", fontsize=8)
fig.colorbar(axes[0][1].collections[0], ax=axes[0], label="absolute contour error [m]", shrink=0.8)

ax = axes[1][0]
for name, run in data.items():
    ax.plot(100.0 * run["s"] / TRACK_LENGTH, np.abs(run["d"]["contour_error"]),
            color=RUNS[name]["color"], lw=1.3, label=name)
ax.axhline(0.30, color="#f59e0b", ls="--", lw=1, label="0.30 m reference")
ax.set(title="Tracking error versus estimated lap progress", xlabel="lap progress [%]", ylabel="|contour error| [m]")
ax.grid(alpha=0.25); ax.legend(fontsize=8)

ax = axes[1][1]
for name, run in data.items():
    ax.plot(100.0 * run["s"] / TRACK_LENGTH, run["d"]["mpcc_minimum_margin"],
            color=RUNS[name]["color"], lw=1.2, label=name)
ax.axhline(0.0, color="black", lw=1)
ax.set(title="Predicted minimum corridor margin", xlabel="lap progress [%]", ylabel="margin [m]")
ax.set_ylim(-0.25, 0.8); ax.grid(alpha=0.25); ax.legend(fontsize=8)

ax = axes[2][0]
ax.plot(original_track["s_m"], original_track["w_tr_left_m"], color="#9ca3af", lw=1, label="original left")
ax.plot(original_track["s_m"], original_track["w_tr_right_m"], color="#d1d5db", lw=1, label="original right")
ax.plot(smooth_track["s_m"], smooth_track["w_tr_left_m"], color="#2563eb", lw=1.8, label="smooth left")
ax.plot(smooth_track["s_m"], smooth_track["w_tr_right_m"], color="#e11d48", lw=1.8, label="smooth right")
ax.set(title="Boundary widths used by MPCC", xlabel="track s [m]", ylabel="width [m]")
ax.grid(alpha=0.25); ax.legend(ncol=2, fontsize=8)

ax = axes[2][1]
for name, run in data.items():
    ax.plot(100.0 * run["s"] / TRACK_LENGTH, run["d"]["vx"], color=RUNS[name]["color"], lw=1.2, label=f"{name} speed")
ax.set(title="Speed on the selected clean lap", xlabel="lap progress [%]", ylabel="vx [m/s]")
ax.grid(alpha=0.25); ax.legend(fontsize=8)

fig.savefig(OUT / "raceline_smooth_clean_lap_comparison.png", dpi=180)
plt.close(fig)

base = summary["V3pro original"]
smooth = summary["V3pro raceline_smooth"]
lines = [
    "# V3pro raceline_smooth run analysis",
    "",
    "Only the second complete lap is compared. The launch/recovery lap and later stop/carry data are excluded.",
    "No rosbag was produced for the smooth run; this analysis uses the complete controller JSONL telemetry and lap log.",
    "",
    "| Metric | Original V3pro | raceline_smooth | Change |",
    "|---|---:|---:|---:|",
]
rows = [
    ("Lap time [s]", "lap_time_s", False),
    ("Lap mean speed [m/s]", "lap_mean_speed_mps", True),
    ("Contour MAE [cm]", "contour_mae_m", False, 100),
    ("Contour P95 [cm]", "contour_p95_m", False, 100),
    ("Heading P95 [rad]", "heading_p95_rad", False),
    ("PP active [s]", "pp_active_s", False),
    ("Prediction-risk fraction [%]", "risk_fraction", False, 100),
    ("Track slack >1 mm [%]", "track_slack_over_1mm_fraction", False, 100),
    ("Steering-rate P95 [rad/s]", "steering_rate_abs_p95_radps", False),
    ("Solve P95 [ms]", "solve_p95_ms", False),
]
for row in rows:
    label, key, higher_better, *scale_arg = row
    scale = scale_arg[0] if scale_arg else 1.0
    a, b = base[key] * scale, smooth[key] * scale
    delta = 100.0 * (b - a) / a if a else 0.0
    lines.append(f"| {label} | {a:.3f} | {b:.3f} | {delta:+.1f}% |")
lines += [
    "",
    f"The remaining smooth-run PP event is at estimated s={next(row['estimated_track_s_start_m'] for row in event_rows if row['run']=='V3pro raceline_smooth' and row['event']=='PP/recovery'):.1f}–{next(row['estimated_track_s_end_m'] for row in event_rows if row['run']=='V3pro raceline_smooth' and row['event']=='PP/recovery'):.1f} m.",
]
(OUT / "REPORT.md").write_text("\n".join(lines) + "\n")

print(json.dumps(summary, indent=2))
print(f"Wrote {OUT / 'raceline_smooth_clean_lap_comparison.png'}")
