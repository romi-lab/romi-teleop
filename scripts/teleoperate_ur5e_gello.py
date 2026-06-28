"""Teleoperate a physical UR5e with GELLO and an OnRobot RG2 gripper."""

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

from lerobot_robot_ur5e import UR5EConfig
from lerobot_teleoperator_gello import GelloConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Teleoperate a physical UR5e with a GELLO leader.")
    parser.add_argument("--robot-ip", default="192.168.0.10", help="UR5e controller IP address.")
    parser.add_argument("--robot-id", default="ur5e", help="LeRobot id for the UR5e robot.")
    parser.add_argument("--teleop-port", default="/dev/ttyUSB0", help="GELLO Dynamixel serial port.")
    parser.add_argument("--teleop-id", default="gello", help="LeRobot id for the GELLO teleoperator.")
    parser.add_argument("--fps", type=int, default=20, help="Control loop frequency.")
    parser.add_argument("--teleop-time-s", type=float, default=None, help="Optional teleoperation duration.")
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=Path("calibration"),
        help="Root directory for LeRobot calibration files.",
    )
    parser.add_argument("--display-data", action="store_true", help="Log observations and actions to Rerun.")
    parser.add_argument("--no-gripper", action="store_true", help="Do not connect or control the OnRobot RG2.")
    parser.add_argument("--no-reset", action="store_true", help="Do not move UR5e to configured start joints first.")
    parser.add_argument("--gripper-port", type=int, default=41414, help="OnRobot XML-RPC port.")
    parser.add_argument("--gripper-rg-id", type=int, default=0, help="RG2 id used by the onRobot library.")
    parser.add_argument("--gripper-open-width-mm", type=float, default=100.0, help="RG2 open width in millimeters.")
    parser.add_argument("--gripper-closed-width-mm", type=float, default=0.0, help="RG2 closed width in millimeters.")
    parser.add_argument("--gripper-force-n", type=float, default=40.0, help="RG2 grip force in newtons.")
    parser.add_argument(
        "--max-joint-delta",
        type=float,
        default=0.1,
        help="Maximum commanded joint change per control step in radians. Use 0 to disable.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    init_logging()
    logging.info("Starting UR5e <-> GELLO teleoperation")

    register_third_party_plugins()

    robot_cfg = UR5EConfig(
        ip=args.robot_ip,
        id=args.robot_id,
        calibration_dir=args.calibration_dir / "robots" / "ur5e",
        cameras={},
        with_gripper=not args.no_gripper,
        gripper_port=args.gripper_port,
        gripper_rg_id=args.gripper_rg_id,
        gripper_open_width_mm=args.gripper_open_width_mm,
        gripper_closed_width_mm=args.gripper_closed_width_mm,
        gripper_force_n=args.gripper_force_n,
        max_joint_delta_per_step=args.max_joint_delta if args.max_joint_delta > 0 else None,
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
        init_rerun(session_name="ur5e_gello_teleoperation")

    try:
        robot.connect()
        if not args.no_reset:
            logging.info("Moving UR5e to start joints: %s", [round(value, 4) for value in robot_cfg.start_joints])
            robot.move_to_start_joints(wait=True)
            robot.reset_gripper(0.1)

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
