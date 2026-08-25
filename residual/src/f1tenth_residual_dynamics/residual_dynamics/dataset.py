"""Dataset loading, time-history features, and nominal residual targets."""

from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter


STATE_COLUMNS = ("vx", "vy", "yaw_rate")


def load_runs(paths):
    runs = []
    for path in paths:
        path = Path(path)
        if path.is_dir():
            files = sorted(path.glob("*.npz"))
        else:
            files = [path]
        for filename in files:
            with np.load(str(filename), allow_pickle=False) as data:
                run = {key: np.asarray(data[key]) for key in data.files}
            run["source_path"] = str(filename)
            runs.append(run)
    if not runs:
        raise ValueError("no NPZ datasets found")
    return runs


def zoh(times, values, query):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values)
    query = np.asarray(query, dtype=float)
    indices = np.searchsorted(times, query, side="right") - 1
    valid = indices >= 0
    indices = np.clip(indices, 0, max(len(times) - 1, 0))
    if values.ndim == 1:
        out = values[indices].astype(float)
        out[~valid] = np.nan
    else:
        out = values[indices].astype(float)
        out[~valid, :] = np.nan
    return out


def smooth_states(run, window):
    result = {key: value.copy() if isinstance(value, np.ndarray) else value
              for key, value in run.items()}
    count = len(run["t"])
    window = min(int(window), count if count % 2 else count - 1)
    if window >= 5:
        for key in STATE_COLUMNS:
            result[key] = savgol_filter(run[key], window, 2, mode="interp")
    return result


def feature_names(history_s, feature_set="history_v1"):
    names = [
        "bias", "vx", "vy", "yaw_rate", "delta",
        "speed_cmd", "steer_cmd", "speed_error", "steer_error",
        "vx_yaw_rate", "vy_yaw_rate", "abs_vx_vy", "abs_vx_yaw_rate",
        "vx_tan_delta", "vx_steer_cmd", "vx2_steer_cmd",
    ]
    if feature_set not in ("history_v1", "markov_v1"):
        raise ValueError("unknown feature set: %s" % feature_set)
    if feature_set == "markov_v1":
        return names
    for delay in history_s[1:]:
        tag = str(int(round(1000.0 * delay)))
        names.extend(("speed_cmd_%sms" % tag, "steer_cmd_%sms" % tag))
    return names


def make_features(vx, vy, yaw_rate, delta, command_history,
                  feature_set="history_v1"):
    command_history = np.asarray(command_history, dtype=float)
    speed_cmd, steer_cmd = command_history[0]
    values = [
        1.0, vx, vy, yaw_rate, delta, speed_cmd, steer_cmd,
        speed_cmd - vx, steer_cmd - delta,
        vx * yaw_rate, vy * yaw_rate, abs(vx) * vy, abs(vx) * yaw_rate,
        vx * np.tan(np.clip(delta, -0.7, 0.7)),
        vx * steer_cmd, vx * vx * steer_cmd,
    ]
    if feature_set not in ("history_v1", "markov_v1"):
        raise ValueError("unknown feature set: %s" % feature_set)
    if feature_set == "history_v1":
        values.extend(command_history[1:].reshape(-1).tolist())
    return np.asarray(values, dtype=float)


def command_histories(run, query_times, history_s):
    command_times = run.get("command_t", run["t"])
    commands = np.column_stack((run.get("command_speed", run["cmd_speed"]),
                                run.get("command_steer", run["cmd_steer"])))
    return np.stack([zoh(command_times, commands, query_times - delay)
                     for delay in history_s], axis=1)


def prepare_run(run, nominal_model, config):
    dataset_cfg = config["dataset"]
    run = smooth_states(run, dataset_cfg.get("smoothing_window", 7))
    times = np.asarray(run["t"], dtype=float)
    history_s = np.asarray(dataset_cfg["command_history_s"], dtype=float)
    feature_set = dataset_cfg.get("feature_set", "history_v1")
    histories = command_histories(run, times, history_s)
    steering = nominal_model.infer_steering(times, histories[:, 0, 1])

    features, targets, row_indices = [], [], []
    baseline_next, measured_next, dts = [], [], []
    for index in range(len(times) - 1):
        dt = times[index + 1] - times[index]
        if not dataset_cfg["minimum_dt_s"] <= dt <= dataset_cfg["maximum_dt_s"]:
            continue
        if run["vx"][index] < dataset_cfg["minimum_speed_mps"]:
            continue
        pose_step = np.hypot(run["x"][index + 1] - run["x"][index],
                             run["y"][index + 1] - run["y"][index])
        yaw_step = abs(np.arctan2(np.sin(run["yaw"][index + 1] - run["yaw"][index]),
                                  np.cos(run["yaw"][index + 1] - run["yaw"][index])))
        if (pose_step > dataset_cfg["maximum_pose_step_m"] or
                yaw_step > dataset_cfg["maximum_yaw_step_rad"] or
                not np.all(np.isfinite(histories[index]))):
            continue
        state = np.asarray([run["vx"][index], run["vy"][index],
                            run["yaw_rate"][index], steering[index]])
        command_times = run.get("command_t", times)
        delayed_speed = zoh(
            command_times, run.get("command_speed", run["cmd_speed"]),
            np.asarray([times[index] - nominal_model.speed_dead_time]))[0]
        delayed_steering = zoh(
            command_times, run.get("command_steer", run["cmd_steer"]),
            np.asarray([times[index] - nominal_model.steering_dead_time]))[0]
        command = np.asarray([delayed_speed, delayed_steering])
        if not np.all(np.isfinite(command)):
            continue
        predicted = nominal_model.step(state, command, dt)
        measured = np.asarray([run["vx"][index + 1], run["vy"][index + 1],
                               run["yaw_rate"][index + 1]])
        target = (measured - predicted[:3]) / dt
        features.append(make_features(*state, histories[index], feature_set))
        targets.append(target)
        baseline_next.append(predicted[:3])
        measured_next.append(measured)
        dts.append(dt)
        row_indices.append(index)
    if not features:
        raise ValueError("run %s has no valid dynamic samples" % run.get("source_path", ""))
    return {
        "features": np.asarray(features), "targets": np.asarray(targets),
        "baseline_next": np.asarray(baseline_next),
        "measured_next": np.asarray(measured_next), "dt": np.asarray(dts),
        "row_indices": np.asarray(row_indices), "run": run,
        "steering": steering, "histories": histories,
    }
