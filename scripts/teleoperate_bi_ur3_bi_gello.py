"""Teleoperate two physical UR3 arms with two GELLO leaders."""

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

from lerobot_robot_bi_ur3 import (
    BiUR3Config,
    LEFT_UR3_IP,
    LEFT_UR3_START_JOINTS,
    RIGHT_UR3_IP,
    RIGHT_UR3_START_JOINTS,
)
from lerobot_robot_ur3 import UR3Config
from lerobot_teleoperator_bi_gello import BiGelloConfig
from lerobot_teleoperator_gello import GelloConfig


D455_SERIAL = "239222303378"
D435_SERIAL = "317222074788"

LEFT_GELLO_CALIBRATION_POSITION = [-1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708]
RIGHT_GELLO_CALIBRATION_POSITION = [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 1.5708]
GELLO_JOINT_SIGNS = [1, 1, -1, 1, 1, 1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Teleoperate two physical UR3 arms with two GELLO leaders.")
    parser.add_argument("--left-robot-ip", default=LEFT_UR3_IP, help="Left UR3 controller IP address.")
    parser.add_argument("--right-robot-ip", default=RIGHT_UR3_IP, help="Right UR3 controller IP address.")
    parser.add_argument("--robot-id", default="bi_ur3", help="LeRobot id for the bimanual UR3 robot.")
    parser.add_argument("--left-teleop-port", default="/dev/ttyUSB0", help="Left GELLO Dynamixel serial port.")
    parser.add_argument("--right-teleop-port", default="/dev/ttyUSB1", help="Right GELLO Dynamixel serial port.")
    parser.add_argument("--teleop-id", default="bi_gello", help="LeRobot id for the bimanual GELLO teleoperator.")
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
    parser.add_argument("--no-gripper", action="store_true", help="Do not connect or control Robotiq grippers.")
    parser.add_argument("--no-reset", action="store_true", help="Do not move UR3 arms to configured start joints first.")
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


def make_ur3_config(
    *,
    side: str,
    ip: str,
    calibration_dir: Path,
    start_joints: tuple[float, ...],
    cameras: dict[str, RealSenseCameraConfig],
    with_gripper: bool,
    gripper_speed: int,
    gripper_force: int,
    max_joint_delta: float,
) -> UR3Config:
    return UR3Config(
        ip=ip,
        id=f"{side}_ur3",
        calibration_dir=calibration_dir,
        cameras=cameras,
        start_joints=start_joints,
        with_gripper=with_gripper,
        gripper_auto_calibrate=False,
        gripper_speed=gripper_speed,
        gripper_force=gripper_force,
        max_joint_delta_per_step=max_joint_delta if max_joint_delta > 0 else None,
    )


def make_gello_config(
    *,
    side: str,
    port: str,
    calibration_dir: Path,
    calibration_position: list[float],
) -> GelloConfig:
    return GelloConfig(
        port=port,
        id=f"{side}_gello",
        calibration_dir=calibration_dir,
        calibration_position=calibration_position,
        joint_signs=GELLO_JOINT_SIGNS,
    )


def main() -> None:
    args = parse_args()
    init_logging()
    logging.info("Starting BiUR3 <-> BiGello teleoperation")

    register_third_party_plugins()

    cameras = {} if args.no_cameras else make_realsense_cameras()
    robot_calibration_dir = args.calibration_dir / "robots" / "bi_ur3"
    teleop_calibration_dir = args.calibration_dir / "teleoperators" / "bi_gello"

    robot_cfg = BiUR3Config(
        id=args.robot_id,
        calibration_dir=robot_calibration_dir,
        left_arm_config=make_ur3_config(
            side="left",
            ip=args.left_robot_ip,
            calibration_dir=robot_calibration_dir / "left",
            start_joints=LEFT_UR3_START_JOINTS,
            cameras=cameras,
            with_gripper=not args.no_gripper,
            gripper_speed=args.gripper_speed,
            gripper_force=args.gripper_force,
            max_joint_delta=args.max_joint_delta,
        ),
        right_arm_config=make_ur3_config(
            side="right",
            ip=args.right_robot_ip,
            calibration_dir=robot_calibration_dir / "right",
            start_joints=RIGHT_UR3_START_JOINTS,
            cameras={},
            with_gripper=not args.no_gripper,
            gripper_speed=args.gripper_speed,
            gripper_force=args.gripper_force,
            max_joint_delta=args.max_joint_delta,
        ),
    )
    teleop_cfg = BiGelloConfig(
        id=args.teleop_id,
        calibration_dir=teleop_calibration_dir,
        left_arm_config=make_gello_config(
            side="left",
            port=args.left_teleop_port,
            calibration_dir=teleop_calibration_dir / "left",
            calibration_position=LEFT_GELLO_CALIBRATION_POSITION,
        ),
        right_arm_config=make_gello_config(
            side="right",
            port=args.right_teleop_port,
            calibration_dir=teleop_calibration_dir / "right",
            calibration_position=RIGHT_GELLO_CALIBRATION_POSITION,
        ),
    )

    robot = make_robot_from_config(robot_cfg)
    teleop = make_teleoperator_from_config(teleop_cfg)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    if args.display_data:
        init_rerun(session_name="bi_ur3_bi_gello_teleoperation")

    try:
        robot.connect()
        if not args.no_reset:
            logging.info("Moving left UR3 to start joints: %s", [round(value, 4) for value in LEFT_UR3_START_JOINTS])
            logging.info("Moving right UR3 to start joints: %s", [round(value, 4) for value in RIGHT_UR3_START_JOINTS])
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
