#!/usr/bin/env python3
"""Offline five-mode benchmark for delayed PointLIO state prediction."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict
from pathlib import Path

import numpy as np

from f1tenth_dynamic_mpcc.command_history_buffer import CommandHistoryBuffer, PublishedCommandSample
from f1tenth_dynamic_mpcc.config import load_yaml, package_root
from f1tenth_dynamic_mpcc.low_latency_state_predictor import LowLatencyVehicleStatePredictor
from f1tenth_dynamic_mpcc.track_model import wrap_angle
from f1tenth_dynamic_mpcc.vehicle_model import DynamicBicycleModel, VehicleParameters


MODES = ("RAW", "MODEL_ONLY", "MODEL_IMU", "MODEL_IMU_WHEEL", "REPROPAGATION")


def yaw(message) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def odom_state(message) -> np.ndarray:
    return np.asarray([
        message.pose.pose.position.x, message.pose.pose.position.y, yaw(message),
        message.twist.twist.linear.x, message.twist.twist.linear.y,
        message.twist.twist.angular.z, 0.0, 0.0, 0.0,
    ], dtype=float)


def interpolate_reference(stamps, states, stamp: float):
    index = int(np.searchsorted(stamps, stamp))
    if index == 0 or index >= len(stamps):
        return None
    t0, x0 = stamps[index-1], states[index-1]
    t1, x1 = stamps[index], states[index]
    if t1-t0 > 0.10:
        return None
    ratio = (stamp-t0)/(t1-t0)
    out = x0 + ratio*(x1-x0)
    out[2] = x0[2] + ratio*float(wrap_angle(x1[2]-x0[2]))
    return out


def predictor(params, commands, use_imu, use_wheel):
    result = LowLatencyVehicleStatePredictor(DynamicBicycleModel(params), commands, 0.01)
    result.use_imu = use_imu
    result.use_wheel = use_wheel
    return result


def metrics(errors):
    if not errors:
        return {"count": 0}
    values = np.asarray(errors)
    names = ("x", "y", "yaw", "vx", "vy", "yaw_rate")
    result = {"count": len(values)}
    for index, name in enumerate(names):
        absolute = np.abs(values[:, index])
        result[name] = {
            "rmse": float(np.sqrt(np.mean(values[:, index]**2))),
            "p95_abs": float(np.percentile(absolute, 95)),
            "max_abs": float(np.max(absolute)),
        }
    position = np.linalg.norm(values[:, :2], axis=1)
    result["position"] = {
        "rmse": float(np.sqrt(np.mean(position**2))),
        "p95": float(np.percentile(position, 95)),
        "max": float(np.max(position)),
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        import rosbag
    except ImportError as error:
        raise SystemExit(f"ROS rosbag Python module is required: {error}")

    root = package_root()
    controller = load_yaml(root/"config/controller.yaml")
    params = VehicleParameters.from_yaml(root/"config/vehicle.yaml", controller)
    max_reprop = float(controller["localization_prediction"]["max_repropagation_s"])
    all_errors = {mode: [] for mode in MODES}
    categories = {mode: {} for mode in MODES}
    ages, horizons = [], []

    for bag_path in args.bags:
        events=[]; references=[]
        with rosbag.Bag(str(Path(bag_path).expanduser())) as bag:
            for topic, message, bag_time in bag.read_messages(topics=[
                "/localization/vehicle_odom", "/livox/imu", "/tianracer/odom",
                "/tianracer/ackermann_cmd",
            ]):
                receive=bag_time.to_sec(); events.append((receive,topic,message))
                if topic=="/localization/vehicle_odom":
                    references.append((message.header.stamp.to_sec(), odom_state(message)))
        references.sort(key=lambda x:x[0]); events.sort(key=lambda x:x[0])
        reference_stamps = np.asarray([item[0] for item in references])
        reference_states = np.asarray([item[1] for item in references])
        histories={mode:CommandHistoryBuffer(3.0) for mode in MODES[1:]}
        predictors={
            "MODEL_ONLY": predictor(params,histories["MODEL_ONLY"],False,False),
            "MODEL_IMU": predictor(params,histories["MODEL_IMU"],True,False),
            "MODEL_IMU_WHEEL": predictor(params,histories["MODEL_IMU_WHEEL"],True,True),
            "REPROPAGATION": predictor(params,histories["REPROPAGATION"],True,True),
        }
        previous_cmd=(None,0.0)
        for receive,topic,message in events:
            if topic=="/tianracer/ackermann_cmd":
                previous_stamp,previous_steer=previous_cmd
                rate=0.0 if previous_stamp is None else (message.steering_angle-previous_steer)/max(receive-previous_stamp,1e-4)
                sample=PublishedCommandSample(receive,message.speed,message.steering_angle,0.0,rate)
                for history in histories.values(): history.push(sample)
                previous_cmd=(receive,message.steering_angle)
            elif topic=="/livox/imu":
                stamp=message.header.stamp.to_sec()
                for mode in ("MODEL_IMU","MODEL_IMU_WHEEL","REPROPAGATION"):
                    predictors[mode].push_imu(stamp,message.angular_velocity.z)
            elif topic=="/tianracer/odom":
                stamp=message.header.stamp.to_sec()
                for mode in ("MODEL_IMU_WHEEL","REPROPAGATION"):
                    predictors[mode].push_wheel(stamp,message.twist.twist.linear.x)
            elif topic=="/localization/vehicle_odom":
                source=message.header.stamp.to_sec(); age=receive-source
                if age<0 or age>max_reprop: continue
                measured=odom_state(message)
                truth=interpolate_reference(reference_stamps, reference_states, receive)
                if truth is None: continue
                ages.append(age)
                outputs={"RAW":measured}
                for mode in ("MODEL_ONLY","MODEL_IMU","MODEL_IMU_WHEEL"):
                    outputs[mode]=predictors[mode].predict_measurement_to_now(
                        measured,source,receive,max_reprop).state
                outputs["REPROPAGATION"]=predictors["REPROPAGATION"].correct_delayed_measurement_and_repropagate(
                    measured,source,receive,max_reprop).state
                horizons.append(age)
                speed=abs(truth[3]); turn=("left" if truth[5]>0.15 else "right" if truth[5]<-0.15 else "straight")
                band="0-1" if speed<1.0 else "1-2" if speed<2.0 else "2-3"
                for mode,state in outputs.items():
                    error=state[:6]-truth[:6]; error[2]=float(wrap_angle(error[2]))
                    all_errors[mode].append(error)
                    categories[mode].setdefault(turn,[]).append(error)
                    categories[mode].setdefault(band,[]).append(error)

    report={
        "bags":[str(Path(x).resolve()) for x in args.bags],
        "parameters":{"speed_gain":params.speed_gain,"speed_tau":params.speed_tau,"speed_dead_time":params.speed_dead_time},
        "pointlio_age_s": {"median":float(np.median(ages)),"p95":float(np.percentile(ages,95)),"max":float(np.max(ages))},
        "prediction_horizon_s":{"median":float(np.median(horizons)),"p95":float(np.percentile(horizons,95))},
        "modes":{mode:metrics(all_errors[mode]) for mode in MODES},
        "categories":{mode:{name:metrics(values) for name,values in groups.items()} for mode,groups in categories.items()},
    }
    raw=report["modes"]["RAW"]["position"]["rmse"]
    rep=report["modes"]["REPROPAGATION"]["position"]["rmse"]
    report["acceptance"]={
        "vx_rmse_below_0_15":report["modes"]["REPROPAGATION"]["vx"]["rmse"]<0.15,
        "position_improvement_fraction":float((raw-rep)/raw) if raw>0 else 0.0,
        "position_improvement_above_30_percent":bool(raw>0 and (raw-rep)/raw>0.30),
    }
    output=Path(args.output); output.parent.mkdir(parents=True,exist_ok=True)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+"\n")
    lines=["# Delay compensation replay validation","",f"Bags: {len(args.bags)}",""]
    lines += ["| Mode | Position RMSE | Yaw RMSE | vx RMSE | vy RMSE | r RMSE |","|---|---:|---:|---:|---:|---:|"]
    for mode in MODES:
        m=report['modes'][mode]
        lines.append(f"| {mode} | {m['position']['rmse']:.4f} | {m['yaw']['rmse']:.4f} | {m['vx']['rmse']:.4f} | {m['vy']['rmse']:.4f} | {m['yaw_rate']['rmse']:.4f} |")
    lines += ["",f"PointLIO age median/p95: {report['pointlio_age_s']['median']:.4f}/{report['pointlio_age_s']['p95']:.4f} s",
              f"Position improvement vs RAW: {100*report['acceptance']['position_improvement_fraction']:.1f}%",
              f"REPROPAGATION vx RMSE <0.15 m/s: {report['acceptance']['vx_rmse_below_0_15']}",""]
    output.with_suffix('.md').write_text("\n".join(lines))
    print(output.with_suffix('.md'))

if __name__ == "__main__": main()
