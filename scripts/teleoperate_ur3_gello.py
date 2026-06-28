"""Teleoperate a physical UR3 with GELLO and two RealSense cameras."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import rerun as rr
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.scripts.lerobot_teleoperate import teleop_loop
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import init_rerun

from lerobot_robot_ur3 import UR3Config
from lerobot_teleoperator_gello import GelloConfig


D455_SERIAL = "239222303378"
D435_SERIAL = "317222074788"

# right: 158.132.172.214
# left: 158.132.172.193

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Teleoperate a physical UR3 with a GELLO leader.")
    parser.add_argument("--robot-ip", required=True, help="UR3 controller IP address.")
    parser.add_argument("--robot-id", default="ur3", help="LeRobot id for the UR3 robot.")
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
    parser.add_argument("--no-cameras", action="store_true", help="Disable RealSense cameras.")
    parser.add_argument("--no-gripper", action="store_true", help="Do not connect or control the Robotiq gripper.")
    parser.add_argument("--no-reset", action="store_true", help="Do not move UR3 to configured start joints first.")
    parser.add_argument("--gripper-speed", type=int, default=200, help="Robotiq gripper speed command, 0..255.")
    parser.add_argument("--gripper-force", type=int, default=40, help="Robotiq gripper force command, 0..255.")
    parser.add_argument(
        "--max-joint-delta",
        type=float,
        default=0.1,
        help="Maximum commanded joint change per control step in radians. Use 0 to disable.",
    )
    return parser.parse_args()


def make_realsense_cameras() -> dict[str, RealSenseCameraConfig]:
    return {
        "d455": RealSenseCameraConfig(
            serial_number_or_name=D455_SERIAL,
            width=1280,
            height=720,
            fps=30,
        ),
        "d435": RealSenseCameraConfig(
            serial_number_or_name=D435_SERIAL,
            width=640,
            height=480,
            fps=30,
        ),
    }


def main() -> None:
    args = parse_args()
    init_logging()
    logging.info("Starting UR3 <-> GELLO teleoperation")

    register_third_party_plugins()

    robot_cfg = UR3Config(
        ip=args.robot_ip,
        id=args.robot_id,
        calibration_dir=args.calibration_dir / "robots" / "ur3",
        cameras={} if args.no_cameras else make_realsense_cameras(),
        with_gripper=not args.no_gripper,
        gripper_auto_calibrate=False,
        gripper_speed=args.gripper_speed,
        gripper_force=args.gripper_force,
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
        init_rerun(session_name="ur3_gello_teleoperation")

    try:
        robot.connect()
        if not args.no_reset:
            logging.info("Moving UR3 to start joints: %s", [round(value, 4) for value in robot_cfg.start_joints])
            robot.move_to_start_joints(wait=True)

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
