"""Configuration dataclass for the GELLO teleoperator plugin.

Defines GelloConfig with serial port settings, calibration position, joint signs,
and optional smoothing/async parameters.
"""

from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("gello")
@dataclass
class GelloConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str = "/dev/ttyUSB0"
    baudrate: int = 57_600
    # right arm
    # calibration_position: list[float] = field(default_factory=lambda: [1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 1.5708])
    # left arm
    # calibration_position: list[float] = field(default_factory=lambda: [-1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 1.5708])

    # right arm ur5e start joints: [-90, -90, -90, -90, 90, 0]
    # calibration_position: list[float] = field(
    #     default_factory=lambda: [
    #         -1.5708,
    #         -1.5708,
    #         -1.5708,
    #         -1.5708,
    #         1.5708,
    #         0,
    #     ]
    # )


    # left arm ur5e start joints: [90, -90, 90, -90, -90, 0]
    calibration_position: list[float] = field(
        default_factory=lambda: [
            1.5708,
            -1.5708,
            1.5708,
            -1.5708,
            -1.5708,
            0,
        ]
    )

    joint_signs: list[int] = field(default_factory=lambda: [1, 1, -1, 1, 1, 1])
    
    gripper_travel_counts: int = 575

    # Smoothing factor for Exponential Moving Average (EMA).
    # Range [0, 1]. 1 means no smoothing (instant update), 0 means no update (freeze).
    # Lower values smooth out jitter but add latency.现在只有最后一个关节偏了90度，方向是对的，why？
    smoothing: float = 0.85
    # Whether to run device reading in a background thread.
    # This helps when USB communication is slow (e.g. long cables).
    use_async: bool = True
