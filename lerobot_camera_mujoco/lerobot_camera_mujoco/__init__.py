from .configuration_mujoco import MujocoCameraConfig

__all__ = ["MujocoCamera", "MujocoCameraConfig"]


def __getattr__(name: str):
    if name == "MujocoCamera":
        from .mujoco_camera import MujocoCamera

        return MujocoCamera
    raise AttributeError(name)
