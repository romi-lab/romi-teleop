"""LeRobot camera implementation backed by a MuJoCo renderer."""

from threading import Lock
from typing import Any

import cv2
import mujoco
import numpy as np
from lerobot.cameras.camera import Camera
from lerobot.utils.errors import DeviceNotConnectedError

from .configuration_mujoco import MujocoCameraConfig


class MujocoCamera(Camera):
    def __init__(self, config: MujocoCameraConfig):
        super().__init__(config)
        self.config = config
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._camera: int | str | None = config.camera
        self._renderer: mujoco.Renderer | None = None
        self._frame_lock = Lock()
        self._latest_image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self._latest_depth = np.zeros((self.height, self.width, 1), dtype=np.float32)
        self._connected = False

    def __repr__(self) -> str:
        return f"MujocoCamera(camera={self._camera}, width={self.width}, height={self.height})"

    @property
    def is_connected(self) -> bool:
        return self._connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        return []

    def bind(self, model: mujoco.MjModel, data: mujoco.MjData, camera: int | str) -> None:
        self._model = model
        self._data = data
        self._camera = camera

    def connect(self, warmup: bool = True) -> None:
        if self._model is None or self._data is None or self._camera is None:
            raise DeviceNotConnectedError(f"{self} is not bound to a MuJoCo model/data.")
        self._ensure_renderer()
        self._connected = True
        if warmup:
            self.render()

    def _ensure_renderer(self) -> mujoco.Renderer:
        if self._model is None:
            raise DeviceNotConnectedError(f"{self} is not bound to a MuJoCo model.")
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self._model, height=self.height, width=self.width)
        return self._renderer

    def render(self, data: mujoco.MjData | None = None) -> None:
        if data is not None:
            self._data = data
        if self._data is None:
            raise DeviceNotConnectedError(f"{self} is not bound to MuJoCo data.")

        renderer = self._ensure_renderer()
        renderer.disable_depth_rendering()
        renderer.update_scene(self._data, camera=self._camera)
        image = renderer.render().copy()

        renderer.enable_depth_rendering()
        renderer.update_scene(self._data, camera=self._camera)
        depth = renderer.render().copy()[:, :, None].astype(np.float32)
        renderer.disable_depth_rendering()

        with self._frame_lock:
            self._latest_image = image
            self._latest_depth = depth

    def read(self) -> np.ndarray:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        with self._frame_lock:
            return self._latest_image.copy()

    def read_depth(self) -> np.ndarray:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        with self._frame_lock:
            return self._latest_depth.copy()

    def read_resized(self, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
        image = self.read()
        depth = self.read_depth()
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
        depth = cv2.resize(depth[:, :, 0], (width, height), interpolation=cv2.INTER_LINEAR)[:, :, None]
        return image, depth

    def async_read(self, timeout_ms: float = 200) -> np.ndarray:
        return self.read_latest()

    def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
        self.render()
        return self.read()

    def disconnect(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
        self._renderer = None
        self._connected = False
