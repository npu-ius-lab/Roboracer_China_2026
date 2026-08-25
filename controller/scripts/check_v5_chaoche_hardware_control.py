#!/usr/bin/env python3
"""Static proof of the V5 Chaoche real-hardware command wiring.

This test only reads source/launch files.  It never initializes ROS, starts a
node, calls an enable service, or publishes an Ackermann command.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


CANDIDATE_TOPIC = "/v5_chaoche/mpcc/ackermann_cmd_stamped"
HARDWARE_TOPIC = "/tianracer/ackermann_cmd"
ENABLE_SERVICE = "/f1tenth_dynamic_mpcc_v5_chaoche/set_enabled"


def fail(message: str) -> None:
    raise AssertionError(message)


def params(node: ET.Element) -> dict[str, str]:
    return {
        item.attrib["name"]: item.attrib.get("value", "")
        for item in node.findall("param")
    }


def exactly_one(nodes: list[ET.Element], description: str) -> ET.Element:
    if len(nodes) != 1:
        fail(f"expected exactly one {description}, found {len(nodes)}")
    return nodes[0]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    args = parser.parse_args()
    root = args.workspace.expanduser().resolve()

    launch_path = root / "src/f1tenth_dynamic_mpcc/launch/hardware_mpcc_stable_v5_chaoche.launch"
    node_path = root / "src/f1tenth_dynamic_mpcc/src/mpcc_node_auto_relaunch_chaoche.cpp"
    supervisor_path = root / "src/f1tenth_dynamic_mpcc/scripts/automatic_relaunch_supervisor.py"
    start_path = root / "scripts/start_mpcc_hardware_stable_v5_chaoche.sh"
    recorder_path = root / "scripts/record_v5_chaoche_bag.sh"
    manager_config_path = root / "avoidance/v5_chaoche/config/reference_manager.yaml"
    avoidance_start_path = root / "avoidance/v5_chaoche/scripts/start_v5_chaoche_avoidance.sh"
    planner_path = root / "avoidance/v5_chaoche/scripts/overtake_planner.py"
    fusion_path = root / "avoidance/v5_chaoche/scripts/static_target_fusion.py"

    for path in (
        launch_path,
        node_path,
        supervisor_path,
        start_path,
        recorder_path,
        manager_config_path,
        avoidance_start_path,
        planner_path,
        fusion_path,
    ):
        if not path.is_file():
            fail(f"missing required control file: {path}")

    launch = ET.parse(launch_path).getroot()
    arguments = {item.attrib["name"]: item.attrib.get("default", "") for item in launch.findall("arg")}
    if arguments.get("allow_real_hardware") != "false":
        fail("real hardware must remain locked by default")
    if arguments.get("hardware_command_topic") != HARDWARE_TOPIC:
        fail("hardware command topic default is not the TianRacer actuator topic")
    if arguments.get("local_reference_topic") != "/v5_chaoche/local_reference":
        fail("V5 Chaoche local reference is not exposed to the controller")

    controller = exactly_one(
        [node for node in launch.findall("node") if node.attrib.get("type") == "mpcc_node_auto_relaunch_chaoche_cpp"],
        "V5 Chaoche MPCC controller",
    )
    controller_params = params(controller)
    if controller_params.get("command_topic") != CANDIDATE_TOPIC:
        fail("V5 Chaoche MPCC is not publishing its isolated command candidate")
    if controller_params.get("start_enabled") != "false":
        fail("MPCC must start disabled and be armed only by the supervisor")
    if controller_params.get("local_reference_topic") != "$(arg local_reference_topic)":
        fail("local overtake reference is not wired into V5 Chaoche MPCC")

    supervisors = [
        node for node in launch.findall("node")
        if node.attrib.get("type") == "automatic_relaunch_supervisor.py"
    ]
    supervisor = exactly_one(supervisors, "guarded hardware supervisor")
    if supervisor.attrib.get("if") != "$(arg allow_real_hardware)":
        fail("hardware supervisor is not protected by allow_real_hardware")
    supervisor_params = params(supervisor)
    expected = {
        "allow_real_hardware": "true",
        "enable_service": ENABLE_SERVICE,
        "mpcc_command_topic": CANDIDATE_TOPIC,
        "hardware_command_topic": "$(arg hardware_command_topic)",
    }
    for key, value in expected.items():
        if supervisor_params.get(key) != value:
            fail(f"supervisor parameter {key} is not wired to {value}")

    hardware_consumers = []
    for node in launch.findall("node"):
        if params(node).get("hardware_command_topic") == "$(arg hardware_command_topic)":
            hardware_consumers.append(node.attrib.get("name", node.attrib.get("type", "unknown")))
    if hardware_consumers != [supervisor.attrib.get("name")]:
        fail(f"hardware topic must have one launch owner, got {hardware_consumers}")

    controller_source = node_path.read_text(encoding="utf-8")
    supervisor_source = supervisor_path.read_text(encoding="utf-8")
    start_source = start_path.read_text(encoding="utf-8")
    recorder_source = recorder_path.read_text(encoding="utf-8")
    manager_config_source = manager_config_path.read_text(encoding="utf-8")
    avoidance_start_source = avoidance_start_path.read_text(encoding="utf-8")
    planner_source = planner_path.read_text(encoding="utf-8")
    fusion_source = fusion_path.read_text(encoding="utf-8")
    source_contracts = {
        "controller advertises AckermannDriveStamped candidate": (
            "advertise<ackermann_msgs::AckermannDriveStamped>(command_topic_,1)" in controller_source
        ),
        "controller exposes enable service": (
            'advertiseService("set_enabled"' in controller_source
        ),
        "supervisor refuses an unarmed hardware launch": (
            'allow_real_hardware must be true' in supervisor_source
        ),
        "supervisor owns AckermannDrive hardware publisher": (
            "self.hardware_pub = rospy.Publisher(" in supervisor_source
            and "AckermannDrive," in supervisor_source
        ),
        "start script defaults to real-hardware mode": (
            'ACTION="${1:---hardware}"' in start_source
            and 'V5_CHAOCHE_ALLOW_REAL_HARDWARE:-YES' in start_source
            and 'ACTION" == "--hardware"' in start_source
        ),
        "locked path refresh checks the whole path, not its expired first pose": (
            "nearest_path_distance_sq" in controller_source
            and "local path has no point near the vehicle" in controller_source
            and "local path starts too far from the vehicle" not in controller_source
        ),
        "avoidance status timeout covers the detector loss hold": (
            "status_timeout_s: 1.20" in manager_config_source
        ),
        "FOLLOW speed cap reaches MPCC without a local path": (
            'local_state=="FOLLOW"' in controller_source
            and 'local_state=="ABORT"&&!local_mpcc_active' in controller_source
        ),
        "isolated planner includes stable overtake authorization": (
            "candidate_stability_s" in planner_source
            and "minimum_dynamic_ego_speed_mps" in planner_source
        ),
        "isolated launcher fuses static obstacles before planning": (
            "static_target_fusion.py" in avoidance_start_source
            and "/v5_chaoche/perception/targets_dynamic" in avoidance_start_source
            and 'python3 "$OVERTAKE_PLANNER"' in avoidance_start_source
            and 'overtake_shadow_node.py' not in avoidance_start_source
        ),
        "static fusion has no actuator interface": (
            "AckermannDrive" not in fusion_source
            and "/tianracer/ackermann_cmd" not in fusion_source
            and "command_authority" in fusion_source
        ),
        "bag recorder monitors the namespaced V5 Chaoche supervisor": (
            "/automatic_relaunch_supervisor_v5_chaoche/state" in recorder_source
            and "  /automatic_relaunch_supervisor/state" not in recorder_source
        ),
        "bag recorder captures dynamic/static fusion evidence": (
            "/v5_chaoche/perception/targets_dynamic" in recorder_source
            and "/v5_chaoche/perception/fusion_status" in recorder_source
            and "/v5_chaoche/perception/static_obstacle_markers" in recorder_source
        ),
    }
    missing = [name for name, valid in source_contracts.items() if not valid]
    if missing:
        fail("source control contract failed: " + "; ".join(missing))

    print("[OK] local reference -> V5 Chaoche MPCC")
    print(f"[OK] V5 Chaoche MPCC -> {CANDIDATE_TOPIC}")
    print(f"[OK] guarded supervisor -> {HARDWARE_TOPIC}")
    print("[OK] MPCC node starts disabled; supervisor owns the enable handoff")
    print("[OK] no-argument wrapper mode resolves to real hardware by request")
    print("[OK] locked local path can refresh after its first pose falls behind")
    print("[OK] status loss hold and supervisor-state recording are consistent")
    print("[OK] FOLLOW cap, stable pass authorization, and static fusion are isolated to V5 Chaoche")
    print("[OK] static inspection only; zero ROS publishers were created")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as error:
        print(f"[FAIL] {error}", file=sys.stderr)
        raise SystemExit(1)
