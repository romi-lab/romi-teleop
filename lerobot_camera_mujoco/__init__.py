from .lerobot_camera_mujoco import MujocoCameraConfig

__all__ = ["MujocoCamera", "MujocoCameraConfig"]


def __getattr__(name: str):
    if name == "MujocoCamera":
        from .lerobot_camera_mujoco import MujocoCamera

        return MujocoCamera
    raise AttributeError(name)
