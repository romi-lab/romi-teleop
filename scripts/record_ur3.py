"""Record UR3 demonstrations with GELLO and two RealSense cameras."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.scripts.lerobot_record import DatasetRecordConfig, RecordConfig, record
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot_robot_ur3 import UR3Config
from lerobot_teleoperator_gello import GelloConfig


D455_SERIAL = "239222303378"
D435_SERIAL = "317222074788"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record UR3 demonstrations controlled by GELLO.")
    parser.add_argument("--robot-ip", required=True, help="UR3 controller IP address.")
    parser.add_argument("--robot-id", default="ur3", help="LeRobot id for the UR3 robot.")
    parser.add_argument("--teleop-port", default="/dev/ttyUSB0", help="GELLO Dynamixel serial port.")
    parser.add_argument("--teleop-id", default="gello", help="LeRobot id for the GELLO teleoperator.")
    parser.add_argument("--repo-id", required=True, help="Dataset repo id, e.g. songhao/ur3-demo.")
    parser.add_argument("--single-task", required=True, help="Task description stored with every frame.")
    parser.add_argument("--root", type=Path, default=None, help="Local dataset root directory.")
    parser.add_argument("--fps", type=int, default=20, help="Recording/control frequency.")
    parser.add_argument("--episode-time-s", type=float, default=60.0, help="Seconds per recorded episode.")
    parser.add_argument("--reset-time-s", type=float, default=10.0, help="Seconds between episodes for reset.")
    parser.add_argument("--num-episodes", type=int, default=10, help="Number of episodes to record.")
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=Path("calibration"),
        help="Root directory for LeRobot calibration files.",
    )
    parser.add_argument(
        "--display-data",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Log observations and actions to Rerun.",
    )
    parser.add_argument("--no-cameras", action="store_true", help="Disable RealSense cameras.")
    parser.add_argument("--no-gripper", action="store_true", help="Do not connect or control the Robotiq gripper.")
    parser.add_argument("--gripper-speed", type=int, default=160, help="Robotiq gripper speed command, 0..255.")
    parser.add_argument("--gripper-force", type=int, default=255, help="Robotiq gripper force command, 0..255.")
    parser.add_argument(
        "--max-joint-delta",
        type=float,
        default=0.1,
        help="Maximum commanded joint change per control step in radians. Use 0 to disable.",
    )
    parser.add_argument("--video", action=argparse.BooleanOptionalAction, default=True, help="Encode images as videos.")
    parser.add_argument(
        "--streaming-encoding",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Encode video frames during recording instead of saving PNGs first.",
    )
    parser.add_argument("--encoder-threads", type=int, default=2, help="Number of threads per streaming encoder.")
    parser.add_argument("--resume", action="store_true", help="Resume recording into an existing dataset.")
    parser.add_argument("--push-to-hub", action="store_true", help="Push dataset to Hugging Face Hub when done.")
    parser.add_argument("--private", action="store_true", help="Push as a private Hub dataset.")
    parser.add_argument("--play-sounds", action=argparse.BooleanOptionalAction, default=True, help="Enable spoken prompts.")
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
    logging.info("Recording UR3 demonstrations with GELLO")
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
    dataset_cfg = DatasetRecordConfig(
        repo_id=args.repo_id,
        single_task=args.single_task,
        root=args.root,
        fps=args.fps,
        episode_time_s=args.episode_time_s,
        reset_time_s=args.reset_time_s,
        num_episodes=args.num_episodes,
        video=args.video,
        streaming_encoding=args.streaming_encoding,
        encoder_threads=args.encoder_threads,
        push_to_hub=args.push_to_hub,
        private=args.private,
    )

    cfg = RecordConfig(
        robot=robot_cfg,
        teleop=teleop_cfg,
        dataset=dataset_cfg,
        display_data=args.display_data,
        play_sounds=args.play_sounds,
        resume=args.resume,
    )
    record(cfg)


if __name__ == "__main__":
    main()
