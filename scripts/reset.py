"""Reset a physical UR3 with Robotiq 2F-85 to the configured start joints."""

from __future__ import annotations

import argparse
import logging
import socket
from pathlib import Path

from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging
from lerobot_robot_bi_ur3 import (
    BiUR3,
    BiUR3Config,
    LEFT_UR3_IP,
    LEFT_UR3_START_JOINTS,
    RIGHT_UR3_IP,
    RIGHT_UR3_START_JOINTS,
)
from lerobot_robot_ur3 import UR3, UR3Config

RTDE_CONTROL_PORT = 30004
ROBOTIQ_PORT = 63352


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reset one or two physical UR3 arms to their start joints.")
    parser.add_argument("--ip", default=None, help="Single UR3 controller IP override.")
    parser.add_argument("--left-ip", default=LEFT_UR3_IP, help="Left UR3 controller IP address.")
    parser.add_argument("--right-ip", default=RIGHT_UR3_IP, help="Right UR3 controller IP address.")
    parser.add_argument(
        "--arm",
        choices=("left", "right", "both"),
        default=None,
        help="Arm to reset. Defaults to right for single-arm mode and both for --bi.",
    )
    parser.add_argument("--bi", action="store_true", help="Reset the bimanual UR3 robot.")
    parser.add_argument("--id", default="ur3", help="LeRobot robot id.")
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=Path("calibration"),
        help="Root directory for LeRobot calibration files.",
    )
    parser.add_argument("--speed", type=float, default=0.3, help="moveJ speed for reset.")
    parser.add_argument("--acceleration", type=float, default=0.2, help="moveJ acceleration for reset.")
    parser.add_argument("--no-gripper", action="store_true", help="Do not connect or activate the Robotiq gripper.")
    parser.add_argument("--skip-port-check", action="store_true", help="Skip TCP port checks before connecting.")
    return parser.parse_args()


def _check_tcp_port(host: str, port: int, timeout_s: float = 2.0) -> None:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return
    except OSError as exc:
        raise RuntimeError(
            f"Cannot connect to {host}:{port}. Check the robot IP, network route, and that the UR controller "
            f"has the corresponding service enabled."
        ) from exc


def _arm_defaults(side: str) -> tuple[str, tuple[float, ...]]:
    if side == "left":
        return LEFT_UR3_IP, LEFT_UR3_START_JOINTS
    if side == "right":
        return RIGHT_UR3_IP, RIGHT_UR3_START_JOINTS
    raise ValueError(f"Expected side to be 'left' or 'right', got {side}.")


def _make_single_arm_config(args: argparse.Namespace, side: str, ip: str | None = None) -> UR3Config:
    default_ip, start_joints = _arm_defaults(side)
    return UR3Config(
        ip=ip if ip is not None else default_ip,
        id=f"{args.id}_{side}" if args.bi else args.id,
        calibration_dir=args.calibration_dir / "robots" / "bi_ur3" / side,
        start_joints=start_joints,
        reset_speed=args.speed,
        reset_acceleration=args.acceleration,
        with_gripper=not args.no_gripper,
        cameras={},
    )


def _check_robot_ports(args: argparse.Namespace, ip: str) -> None:
    if args.skip_port_check:
        return

    _check_tcp_port(ip, RTDE_CONTROL_PORT)
    if not args.no_gripper:
        _check_tcp_port(ip, ROBOTIQ_PORT)


def _reset_single_arm(config: UR3Config) -> None:
    robot = UR3(config)
    try:
        robot.connect()
        logging.info(
            "Moving UR3 %s (%s) to start joints: %s",
            config.id,
            config.ip,
            [round(value, 4) for value in config.start_joints],
        )
        robot.move_to_start_joints(wait=True)
        logging.info("UR3 %s reset complete.", config.id)
    finally:
        if robot.is_connected:
            robot.disconnect()


def _reset_both_arms(args: argparse.Namespace) -> None:
    left_config = _make_single_arm_config(args, "left", args.left_ip)
    right_config = _make_single_arm_config(args, "right", args.right_ip)
    robot = BiUR3(
        BiUR3Config(
            id=args.id,
            calibration_dir=args.calibration_dir / "robots" / "bi_ur3",
            left_arm_config=left_config,
            right_arm_config=right_config,
        )
    )

    try:
        robot.connect()
        logging.info(
            "Moving left UR3 (%s) to start joints: %s",
            left_config.ip,
            [round(v, 4) for v in left_config.start_joints],
        )
        logging.info(
            "Moving right UR3 (%s) to start joints: %s",
            right_config.ip,
            [round(v, 4) for v in right_config.start_joints],
        )
        robot.move_to_start_joints(wait=True)
        logging.info("Bimanual UR3 reset complete.")
    finally:
        if robot.is_connected:
            robot.disconnect()


def main() -> None:
    args = parse_args()
    init_logging()
    register_third_party_plugins()

    arm = args.arm
    if arm is None:
        if args.bi:
            arm = "both"
        elif args.ip == args.left_ip:
            arm = "left"
        else:
            arm = "right"

    if args.bi and arm == "both":
        _check_robot_ports(args, args.left_ip)
        _check_robot_ports(args, args.right_ip)
        print("TCP port checks passed. Connecting to both robots...")
        _reset_both_arms(args)
        return

    if arm == "both":
        raise ValueError("Use --bi with --arm both, or choose --arm left/right for single-arm reset.")

    ip = args.ip if args.ip is not None else (args.left_ip if arm == "left" else args.right_ip)
    _check_robot_ports(args, ip)
    print("TCP port checks passed. Connecting to the robot...")
    _reset_single_arm(_make_single_arm_config(args, arm, ip))


if __name__ == "__main__":
    main()





