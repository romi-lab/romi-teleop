"""Teleoperate two physical UR5e arms with two GELLO leaders."""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

import rerun as rr
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.scripts.lerobot_teleoperate import teleop_loop
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging
from lerobot.utils.visualization_utils import init_rerun

from lerobot_robot_bi_ur5e import (
    BiUR5EConfig,
    LEFT_UR5E_GELLO_CALIBRATION_POSITION,
    LEFT_UR5E_IP,
    LEFT_UR5E_START_JOINTS,
    RIGHT_UR5E_GELLO_CALIBRATION_POSITION,
    RIGHT_UR5E_IP,
    RIGHT_UR5E_START_JOINTS,
)
from lerobot_robot_ur5e import UR5EConfig
from lerobot_teleoperator_bi_gello import BiGelloConfig
from lerobot_teleoperator_gello import GelloConfig


GELLO_JOINT_SIGNS = [1, 1, -1, 1, 1, 1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Teleoperate two physical UR5e arms with two GELLO leaders.")
    parser.add_argument("--left-robot-ip", default=LEFT_UR5E_IP, help="Left UR5e controller IP address.")
    parser.add_argument("--right-robot-ip", default=RIGHT_UR5E_IP, help="Right UR5e controller IP address.")
    parser.add_argument("--robot-id", default="bi_ur5e", help="LeRobot id for the bimanual UR5e robot.")
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
    parser.add_argument("--no-gripper", action="store_true", help="Do not connect or control OnRobot RG2 grippers.")
    parser.add_argument("--no-reset", action="store_true", help="Do not move UR5e arms to configured start joints first.")
    parser.add_argument("--left-gripper-port", type=int, default=41414, help="Left OnRobot XML-RPC port.")
    parser.add_argument("--right-gripper-port", type=int, default=41414, help="Right OnRobot XML-RPC port.")
    parser.add_argument("--left-gripper-rg-id", type=int, default=0, help="Left RG2 id used by the onRobot library.")
    parser.add_argument("--right-gripper-rg-id", type=int, default=0, help="Right RG2 id used by the onRobot library.")
    parser.add_argument("--gripper-open-width-mm", type=float, default=100.0, help="RG2 open width in millimeters.")
    parser.add_argument("--gripper-closed-width-mm", type=float, default=0.0, help="RG2 closed width in millimeters.")
    parser.add_argument("--gripper-force-n", type=float, default=40.0, help="RG2 grip force in newtons.")
    parser.add_argument(
        "--gripper-only",
        choices=("left", "right", "both"),
        default=None,
        help="Only send a gripper command for diagnostics; do not connect GELLO or move arm joints.",
    )
    parser.add_argument(
        "--gripper-position",
        type=float,
        default=1.0,
        help="Normalized diagnostic gripper command for --gripper-only: 0.0=open, 1.0=closed.",
    )
    parser.add_argument("--debug-gripper-actions", action="store_true", help="Log left/right gripper actions in teleop.")
    parser.add_argument("--debug-gripper-period-s", type=float, default=0.5, help="Debug gripper log period.")
    parser.add_argument(
        "--max-joint-delta",
        type=float,
        default=0.1,
        help="Maximum commanded joint change per control step in radians. Use 0 to disable.",
    )
    return parser.parse_args()


def make_ur5e_config(
    *,
    side: str,
    ip: str,
    calibration_dir: Path,
    start_joints: tuple[float, ...],
    with_gripper: bool,
    gripper_port: int,
    gripper_rg_id: int,
    gripper_open_width_mm: float,
    gripper_closed_width_mm: float,
    gripper_force_n: float,
    max_joint_delta: float,
) -> UR5EConfig:
    return UR5EConfig(
        ip=ip,
        id=f"{side}_ur5e",
        calibration_dir=calibration_dir,
        cameras={},
        start_joints=start_joints,
        with_gripper=with_gripper,
        gripper_port=gripper_port,
        gripper_rg_id=gripper_rg_id,
        gripper_open_width_mm=gripper_open_width_mm,
        gripper_closed_width_mm=gripper_closed_width_mm,
        gripper_force_n=gripper_force_n,
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
        calibration_position=calibration_position.copy(),
        joint_signs=GELLO_JOINT_SIGNS,
    )


def main() -> None:
    args = parse_args()
    init_logging()
    logging.info("Starting BiUR5E <-> BiGello teleoperation")

    register_third_party_plugins()

    robot_calibration_dir = args.calibration_dir / "robots" / "bi_ur5e"
    teleop_calibration_dir = args.calibration_dir / "teleoperators" / "bi_gello_ur5e"

    robot_cfg = BiUR5EConfig(
        id=args.robot_id,
        calibration_dir=robot_calibration_dir,
        left_arm_config=make_ur5e_config(
            side="left",
            ip=args.left_robot_ip,
            calibration_dir=robot_calibration_dir / "left",
            start_joints=LEFT_UR5E_START_JOINTS,
            with_gripper=not args.no_gripper,
            gripper_port=args.left_gripper_port,
            gripper_rg_id=args.left_gripper_rg_id,
            gripper_open_width_mm=args.gripper_open_width_mm,
            gripper_closed_width_mm=args.gripper_closed_width_mm,
            gripper_force_n=args.gripper_force_n,
            max_joint_delta=args.max_joint_delta,
        ),
        right_arm_config=make_ur5e_config(
            side="right",
            ip=args.right_robot_ip,
            calibration_dir=robot_calibration_dir / "right",
            start_joints=RIGHT_UR5E_START_JOINTS,
            with_gripper=not args.no_gripper,
            gripper_port=args.right_gripper_port,
            gripper_rg_id=args.right_gripper_rg_id,
            gripper_open_width_mm=args.gripper_open_width_mm,
            gripper_closed_width_mm=args.gripper_closed_width_mm,
            gripper_force_n=args.gripper_force_n,
            max_joint_delta=args.max_joint_delta,
        ),
    )
    logging.info(
        "Left UR5e: ip=%s gripper=%s:%s rg_id=%s",
        robot_cfg.left_arm_config.ip,
        robot_cfg.left_arm_config.ip,
        robot_cfg.left_arm_config.gripper_port,
        robot_cfg.left_arm_config.gripper_rg_id,
    )
    logging.info(
        "Right UR5e: ip=%s gripper=%s:%s rg_id=%s",
        robot_cfg.right_arm_config.ip,
        robot_cfg.right_arm_config.ip,
        robot_cfg.right_arm_config.gripper_port,
        robot_cfg.right_arm_config.gripper_rg_id,
    )
    teleop_cfg = BiGelloConfig(
        id=args.teleop_id,
        calibration_dir=teleop_calibration_dir,
        left_arm_config=make_gello_config(
            side="left",
            port=args.left_teleop_port,
            calibration_dir=teleop_calibration_dir / "left",
            calibration_position=LEFT_UR5E_GELLO_CALIBRATION_POSITION,
        ),
        right_arm_config=make_gello_config(
            side="right",
            port=args.right_teleop_port,
            calibration_dir=teleop_calibration_dir / "right",
            calibration_position=RIGHT_UR5E_GELLO_CALIBRATION_POSITION,
        ),
    )

    robot = make_robot_from_config(robot_cfg)
    teleop = make_teleoperator_from_config(teleop_cfg)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    if args.debug_gripper_actions:
        base_teleop_action_processor = teleop_action_processor
        base_robot_action_processor = robot_action_processor
        last_debug_log_s = 0.0

        def teleop_action_processor(input_data):
            action = base_teleop_action_processor(input_data)
            return action

        def robot_action_processor(input_data):
            nonlocal last_debug_log_s
            action = base_robot_action_processor(input_data)
            now_s = time.monotonic()
            if now_s - last_debug_log_s >= args.debug_gripper_period_s:
                last_debug_log_s = now_s
                logging.info(
                    "gripper action: left=%s right=%s keys=%s",
                    action.get("left_gripper"),
                    action.get("right_gripper"),
                    sorted(key for key in action if key.endswith("gripper")),
                )
            return action

    if args.display_data:
        init_rerun(session_name="bi_ur5e_bi_gello_teleoperation")

    try:
        robot.connect()
        if args.gripper_only is not None:
            if args.gripper_only in ("left", "both"):
                logging.info("Sending left RG2 diagnostic command %.3f", args.gripper_position)
                robot.left_arm.reset_gripper(args.gripper_position)
            if args.gripper_only in ("right", "both"):
                logging.info("Sending right RG2 diagnostic command %.3f", args.gripper_position)
                robot.right_arm.reset_gripper(args.gripper_position)
            logging.info("Gripper diagnostic complete.")
            return

        if not args.no_reset:
            logging.info("Moving left UR5e to start joints: %s", [round(value, 4) for value in LEFT_UR5E_START_JOINTS])
            logging.info("Moving right UR5e to start joints: %s", [round(value, 4) for value in RIGHT_UR5E_START_JOINTS])
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
