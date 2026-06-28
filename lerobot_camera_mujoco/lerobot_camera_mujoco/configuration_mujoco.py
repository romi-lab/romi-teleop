"""Configuration for MuJoCo cameras used by LeRobot plugins."""

from dataclasses import dataclass

from lerobot.cameras.configs import CameraConfig


@CameraConfig.register_subclass("mujoco")
@dataclass
class MujocoCameraConfig(CameraConfig):
    camera: str | int | None = None
    use_depth: bool = False

    def __post_init__(self) -> None:
        if self.fps is None:
            self.fps = 60
        if self.width is None:
            self.width = 128
        if self.height is None:
            self.height = 128
