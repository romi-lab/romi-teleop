# lerobot_teleoperator_bi_gello

LeRobot teleoperator plugin that wraps two existing `lerobot_teleoperator_gello.Gello` leaders as one bimanual teleoperator.

Actions are prefixed with `left_` and `right_`, for example:

- `left_joint_0` ... `left_joint_5`, `left_gripper`
- `right_joint_0` ... `right_joint_5`, `right_gripper`

The package intentionally reuses the working single-arm GELLO implementation instead of duplicating Dynamixel logic.
