from .collision_checker import CollisionCheckResult, CollisionChecker

__all__ = ["CollisionCheckResult", "CollisionChecker", "UR3SelfCollisionChecker"]


def __getattr__(name: str):
    if name == "UR3SelfCollisionChecker":
        from .ur3_self_collision import UR3SelfCollisionChecker

        return UR3SelfCollisionChecker
    raise AttributeError(name)

__all__ = [
    "CollisionCheckResult",
    "CollisionChecker",
    "UR3SelfCollisionChecker",
]
