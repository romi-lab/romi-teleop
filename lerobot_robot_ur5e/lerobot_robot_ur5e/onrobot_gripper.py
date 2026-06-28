"""OnRobot RG2 gripper control using the ``onRobot`` Python package."""

from __future__ import annotations

import logging
import time
from contextlib import contextmanager
from typing import Iterator

from onRobot import gripper as onrobot_gripper
from onRobot.gripper import RG2

logger = logging.getLogger(__name__)


class OnRobotGripper:
    """Thin adapter from LeRobot normalized gripper actions to ``onRobot.RG2``."""

    def __init__(
        self,
        rg_id: int = 0,
        open_width_mm: float = 100.0,
        closed_width_mm: float = 0.0,
        force_n: float = 40.0,
        min_command_delta: float = 0.01,
        min_command_period_s: float = 0.05,
        verify_on_connect: bool = True,
    ) -> None:
        self.rg_id = int(rg_id)
        self.open_width_mm = float(open_width_mm)
        self.closed_width_mm = float(closed_width_mm)
        self.force_n = float(force_n)
        self.min_command_delta = float(min_command_delta)
        self.min_command_period_s = float(min_command_period_s)
        self.verify_on_connect = bool(verify_on_connect)

        self.hostname: str | None = None
        self.port: int | None = None
        self.socket_timeout_s = 2.0
        self._rg2: RG2 | None = None
        self._last_action: float | None = None
        self._last_command_time_s = 0.0

    @property
    def is_connected(self) -> bool:
        return self.hostname is not None and self.port is not None and self._rg2 is not None

    @property
    def rpc_url(self) -> str:
        if self.hostname is None or self.port is None:
            raise RuntimeError("OnRobot gripper is not connected.")
        return f"http://{self.hostname}:{self.port}"

    def connect(self, hostname: str, port: int, socket_timeout_s: float = 2.0) -> None:
        self.hostname = hostname
        self.port = int(port)
        self.socket_timeout_s = float(socket_timeout_s)
        self._rg2 = RG2(self.rg_id)
        if self.verify_on_connect:
            rpc_url = self.rpc_url
            try:
                action = self.get_current_position()
            except Exception as exc:
                self.disconnect()
                raise RuntimeError(f"Failed to verify OnRobot RG2 XML-RPC endpoint at {rpc_url}.") from exc
            self._last_action = action
            logger.info("Verified OnRobot RG2 at %s, current normalized position %.3f.", rpc_url, action)

    def disconnect(self) -> None:
        self.hostname = None
        self.port = None
        self._rg2 = None

    def activate(self) -> None:
        pass

    def get_current_position(self) -> float:
        if self._rg2 is None:
            return 0.0 if self._last_action is None else self._last_action

        with self._patched_onrobot_post():
            width_mm = float(self._rg2.get_rg_width())

        width_span = self.closed_width_mm - self.open_width_mm
        if abs(width_span) < 1e-9:
            return 0.0
        action = (width_mm - self.open_width_mm) / width_span
        return max(0.0, min(action, 1.0))

    def move(self, action: float) -> tuple[bool, float]:
        if not self.is_connected or self._rg2 is None:
            raise RuntimeError("OnRobot gripper is not connected.")

        clipped_action = max(0.0, min(float(action), 1.0))
        now_s = time.monotonic()
        if self._last_action is not None:
            delta = abs(clipped_action - self._last_action)
            elapsed_s = now_s - self._last_command_time_s
            if delta < self.min_command_delta or elapsed_s < self.min_command_period_s:
                return True, self._last_action

        width_mm = self.open_width_mm + clipped_action * (self.closed_width_mm - self.open_width_mm)
        logger.info(
            "OnRobot RG2 command url=%s action=%.3f width_mm=%.3f force_n=%.3f rg_id=%s",
            self.rpc_url,
            clipped_action,
            width_mm,
            self.force_n,
            self.rg_id,
        )

        try:
            with self._patched_onrobot_post():
                ok = bool(self._rg2.rg_grip(target_width=width_mm, target_force=self.force_n))
        except Exception:
            logger.exception("Failed to send OnRobot RG2 command via %s.", self.rpc_url)
            return False, 0.0 if self._last_action is None else self._last_action

        if ok:
            self._last_action = clipped_action
            self._last_command_time_s = now_s
        return ok, clipped_action

    @contextmanager
    def _patched_onrobot_post(self) -> Iterator[None]:
        """Redirect the installed onRobot package's hardcoded XML-RPC URL.

        onRobot 0.1.0 hardcodes ``http://192.168.0.99:41414`` inside
        ``RG2.rg_grip`` and ``RG2.get_rg_width``. Keep using the package API,
        but route its HTTP request to the configured robot address.
        """

        original_post = onrobot_gripper.requests.post

        def redirected_post(_url: str, *args, **kwargs):
            kwargs.setdefault("timeout", self.socket_timeout_s)
            return original_post(self.rpc_url, *args, **kwargs)

        onrobot_gripper.requests.post = redirected_post
        try:
            yield
        finally:
            onrobot_gripper.requests.post = original_post
