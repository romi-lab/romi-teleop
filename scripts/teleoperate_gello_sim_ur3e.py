"""Teleoperate the MuJoCo UR3e simulation with GELLO."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import rerun as rr
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.scripts.lerobot_teleoperate import teleop_loop
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import init_rerun

from lerobot_camera_mujoco import MujocoCameraConfig
from lerobot_robot_sim_ur3e import SimUR3EConfig
from lerobot_teleoperator_gello import GelloConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Teleoperate MuJoCo UR3e simulation with a GELLO leader.")
    parser.add_argument("--teleop-port", default="/dev/ttyUSB0", help="GELLO Dynamixel serial port.")
    parser.add_argument("--teleop-id", default="gello", help="LeRobot id for the GELLO teleoperator.")
    parser.add_argument("--robot-id", default="sim_ur3e", help="LeRobot id for the simulated robot.")
    parser.add_argument("--fps", type=int, default=30, help="Control loop frequency.")
    parser.add_argument("--teleop-time-s", type=float, default=None, help="Optional teleoperation duration.")
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=Path("calibration"),
        help="Root directory for LeRobot calibration files.",
    )
    parser.add_argument("--display-data", action="store_true", help="Log observations and actions to Rerun.")
    parser.add_argument("--no-cameras", action="store_true", help="Disable MuJoCo camera observations.")
    parser.add_argument("--no-eye-in-hand", action="store_true", help="Disable the wrist eye-in-hand camera.")
    parser.add_argument("--camera-width", type=int, default=640, help="MuJoCo camera image width.")
    parser.add_argument("--camera-height", type=int, default=480, help="MuJoCo camera image height.")
    parser.add_argument("--no-viewer", action="store_true", help="Do not open the MuJoCo viewer window.")
    parser.add_argument("--collision-debug", action="store_true", help="Enable UR3 self-collision debug logging.")
    parser.add_argument("--command-substeps", type=int, default=6, help="MuJoCo substeps for ordinary arm commands.")
    parser.add_argument(
        "--gripper-command-substeps",
        type=int,
        default=120,
        help="MuJoCo substeps when the Robotiq gripper target changes.",
    )
    return parser.parse_args()


def make_mujoco_cameras(args: argparse.Namespace) -> dict[str, MujocoCameraConfig]:
    cameras = {
        "agentview": MujocoCameraConfig(
            camera="agentview",
            width=args.camera_width,
            height=args.camera_height,
            fps=args.fps,
        )
    }
    if not args.no_eye_in_hand:
        cameras["eye_in_hand"] = MujocoCameraConfig(
            camera="eye_in_hand",
            width=args.camera_width,
            height=args.camera_height,
            fps=args.fps,
        )
    return cameras


def main() -> None:
    args = parse_args()
    init_logging()
    logging.info("Starting GELLO <-> simulated UR3e teleoperation")

    register_third_party_plugins()

    robot_cfg = SimUR3EConfig(
        id=args.robot_id,
        calibration_dir=args.calibration_dir / "robots" / "sim_ur3e",
        cameras={} if args.no_cameras else make_mujoco_cameras(args),
        collision_debug=args.collision_debug,
        show_viewer=not args.no_viewer,
        command_substeps=args.command_substeps,
        gripper_command_substeps=args.gripper_command_substeps,
    )
    teleop_cfg = GelloConfig(
        port=args.teleop_port,
        id=args.teleop_id,
        calibration_dir=args.calibration_dir / "teleoperators" / "gello",
    )

    robot = make_robot_from_config(robot_cfg)
    teleop = make_teleoperator_from_config(teleop_cfg)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    if args.display_data:
        init_rerun(session_name="gello_sim_ur3e")

    try:
        robot.connect()
        teleop.connect()
        teleop_loop(
            teleop=teleop,
            robot=robot,
            fps=args.fps,
            display_data=args.display_data,
            duration=args.teleop_time_s,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
        )
    except KeyboardInterrupt:
        logging.info("Teleoperation interrupted by user")
    finally:
        if args.display_data:
            rr.rerun_shutdown()
        if teleop.is_connected:
            teleop.disconnect()
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
