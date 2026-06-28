"""Reset a physical UR5e with an OnRobot gripper to the configured start joints."""

from __future__ import annotations

import argparse
import logging
import socket
import subprocess
from pathlib import Path

import numpy as np
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging
from lerobot_robot_ur5e import UR5E, UR5EConfig

RTDE_CONTROL_PORT = 30004
RTDE_UR_CAP_PORT = 30004
ONROBOT_RPC_PORT = 41414
# UR5E_START_JOINTS_DEG = (-90.0, -90.0, -90.0, -90.0, 90.0, 0.0)
UR5E_START_JOINTS_DEG = (90.0, -90.0, 90.0, -90.0, -90.0, 0.0)
UR5E_START_JOINTS_RAD = tuple(np.deg2rad(UR5E_START_JOINTS_DEG).tolist())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reset a physical UR5e arm to its start joints.")
    parser.add_argument("--ip", default="192.168.0.10", help="UR5e controller IP address.")
    parser.add_argument("--id", default="ur5e", help="LeRobot robot id.")
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=Path("calibration"),
        help="Root directory for LeRobot calibration files.",
    )
    parser.add_argument("--speed", type=float, default=0.3, help="moveJ speed for reset.")
    parser.add_argument("--acceleration", type=float, default=0.2, help="moveJ acceleration for reset.")
    parser.add_argument(
        "--rtde-port",
        type=int,
        default=RTDE_CONTROL_PORT,
        help="RTDE receive/control port to check. UR controllers normally use 30004.",
    )
    parser.add_argument(
        "--rtde-ur-cap-port",
        type=int,
        default=RTDE_UR_CAP_PORT,
        help="ur_rtde script upload port. This setup uses 30004.",
    )
    parser.add_argument(
        "--gripper-port",
        type=int,
        default=ONROBOT_RPC_PORT,
        help="OnRobot XML-RPC port used by the onRobot Python library.",
    )
    parser.add_argument(
        "--gripper-reset-position",
        type=float,
        default=0.0,
        help="Normalized OnRobot reset position: 0.0=open, 1.0=closed.",
    )
    parser.add_argument("--gripper-rg-id", type=int, default=0, help="RG2 id used by the onRobot library.")
    parser.add_argument("--gripper-open-width-mm", type=float, default=100.0, help="RG2 open width in millimeters.")
    parser.add_argument("--gripper-closed-width-mm", type=float, default=0.0, help="RG2 closed width in millimeters.")
    parser.add_argument("--gripper-force-n", type=float, default=40.0, help="RG2 grip force in newtons.")
    parser.add_argument("--gripper-only", action="store_true", help="Only reset the OnRobot gripper; do not move UR5e.")
    parser.add_argument("--no-gripper", action="store_true", help="Do not configure the OnRobot gripper interface.")
    parser.add_argument("--skip-port-check", action="store_true", help="Skip TCP port checks before connecting.")
    return parser.parse_args()


def _check_tcp_port(host: str, port: int, timeout_s: float = 2.0) -> None:
    local_ips = _get_local_ipv4_addresses()
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return
    except OSError as exc:
        local_ip_hint = ""
        if host in local_ips:
            local_ip_hint = (
                f" {host} is one of this computer's local IPv4 addresses; use the UR controller IP instead, "
                "and keep the computer on a different address in the same subnet."
            )
        raise RuntimeError(
            f"Cannot connect to {host}:{port}. Check the robot IP, network route, and that the UR controller "
            f"has the corresponding service enabled.{local_ip_hint}"
        ) from exc


def _get_local_ipv4_addresses() -> set[str]:
    try:
        output = subprocess.check_output(["hostname", "-I"], text=True, timeout=1.0)
    except Exception:
        return set()
    return {value for value in output.split() if "." in value}


def _check_robot_ports(args: argparse.Namespace) -> None:
    if args.skip_port_check:
        return

    _check_tcp_port(args.ip, args.rtde_port)
    _check_tcp_port(args.ip, args.rtde_ur_cap_port)
    if not args.no_gripper:
        _check_tcp_port(args.ip, args.gripper_port)


def _make_config(args: argparse.Namespace) -> UR5EConfig:
    return UR5EConfig(
        ip=args.ip,
        id=args.id,
        calibration_dir=args.calibration_dir / "robots" / "ur5e",
        start_joints=UR5E_START_JOINTS_RAD,
        reset_speed=args.speed,
        reset_acceleration=args.acceleration,
        rtde_ur_cap_port=args.rtde_ur_cap_port,
        with_gripper=not args.no_gripper,
        gripper_port=args.gripper_port,
        gripper_rg_id=args.gripper_rg_id,
        gripper_open_width_mm=args.gripper_open_width_mm,
        gripper_closed_width_mm=args.gripper_closed_width_mm,
        gripper_force_n=args.gripper_force_n,
        cameras={},
    )


def reset_ur5e(config: UR5EConfig, gripper_reset_position: float = 0.0, gripper_only: bool = False) -> None:
    robot = UR5E(config)
    try:
        robot.connect()
        if robot.gripper is not None:
            logging.info("Resetting OnRobot gripper to %.3f (0.0=open, 1.0=closed).", gripper_reset_position)
            sent_gripper = robot.reset_gripper(gripper_reset_position)
            logging.info("OnRobot gripper reset command sent: %.3f.", sent_gripper)

        if gripper_only:
            logging.info("Gripper-only reset complete.")
            return

        logging.info(
            "Moving UR5e %s (%s) to start joints deg=%s rad=%s",
            config.id,
            config.ip,
            [round(value, 2) for value in UR5E_START_JOINTS_DEG],
            [round(value, 4) for value in config.start_joints],
        )
        robot.move_to_start_joints(wait=True)
        logging.info("UR5e %s reset complete.", config.id)
    finally:
        if robot.is_connected:
            robot.disconnect()


def main() -> None:
    args = parse_args()
    init_logging()
    register_third_party_plugins()

    _check_robot_ports(args)
    print("TCP port checks passed. Connecting to the UR5e...")
    reset_ur5e(
        _make_config(args),
        gripper_reset_position=args.gripper_reset_position,
        gripper_only=args.gripper_only,
    )


if __name__ == "__main__":
    main()
