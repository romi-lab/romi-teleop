"""Record bimanual UR3 demonstrations with two GELLO leaders."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from lerobot.scripts.lerobot_record import DatasetRecordConfig, RecordConfig, record
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot_robot_bi_ur3 import (
    BiUR3Config,
    LEFT_UR3_IP,
    LEFT_UR3_START_JOINTS,
    RIGHT_UR3_IP,
    RIGHT_UR3_START_JOINTS,
)
from lerobot_teleoperator_bi_gello import BiGelloConfig

from teleoperate_bi_ur3_bi_gello import (
    LEFT_GELLO_CALIBRATION_POSITION,
    RIGHT_GELLO_CALIBRATION_POSITION,
    make_gello_config,
    make_realsense_cameras,
    make_ur3_config,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record bimanual UR3 demonstrations controlled by two GELLO leaders.")
    parser.add_argument("--left-robot-ip", default=LEFT_UR3_IP, help="Left UR3 controller IP address.")
    parser.add_argument("--right-robot-ip", default=RIGHT_UR3_IP, help="Right UR3 controller IP address.")
    parser.add_argument("--robot-id", default="bi_ur3", help="LeRobot id for the bimanual UR3 robot.")
    parser.add_argument("--left-teleop-port", default="/dev/ttyUSB0", help="Left GELLO Dynamixel serial port.")
    parser.add_argument("--right-teleop-port", default="/dev/ttyUSB1", help="Right GELLO Dynamixel serial port.")
    parser.add_argument("--teleop-id", default="bi_gello", help="LeRobot id for the bimanual GELLO teleoperator.")
    parser.add_argument("--repo-id", required=True, help="Dataset repo id, e.g. songhao/bi-ur3-demo.")
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
    parser.add_argument("--no-gripper", action="store_true", help="Do not connect or control Robotiq grippers.")
    parser.add_argument("--gripper-speed", type=int, default=200, help="Robotiq gripper speed command, 0..255.")
    parser.add_argument("--gripper-force", type=int, default=40, help="Robotiq gripper force command, 0..255.")
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


def main() -> None:
    args = parse_args()
    logging.info("Recording BiUR3 demonstrations with BiGello")
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
