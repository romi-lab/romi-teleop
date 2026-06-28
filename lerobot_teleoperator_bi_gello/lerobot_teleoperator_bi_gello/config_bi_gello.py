"""Configuration dataclass for the bimanual GELLO teleoperator plugin."""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot_teleoperator_gello import GelloConfig


@TeleoperatorConfig.register_subclass("bi_gello")
@dataclass(kw_only=True)
class BiGelloConfig(TeleoperatorConfig):
    """Configuration for two GELLO leaders controlled as one bimanual teleoperator."""

    left_arm_config: GelloConfig
    right_arm_config: GelloConfig
