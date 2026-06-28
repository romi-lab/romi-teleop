# lerobot_robot_bi_ur5e

LeRobot robot plugin that wraps two existing `lerobot_robot_ur5e.UR5E` robots as a single bimanual robot.

Actions and observations are prefixed with `left_` and `right_`, for example:

- `left_joint_0` ... `left_joint_5`, `left_gripper`
- `right_joint_0` ... `right_joint_5`, `right_gripper`

The package reuses the single-arm UR5e implementation, including RTDE control
and OnRobot RG2 control through the `onRobot` Python package.

Default arm assignment:

- Left UR5e: `192.168.0.10`, start joints `[90, -90, 90, -90, -90, 0]`
- Right UR5e: `192.168.0.11`, start joints `[-90, -90, -90, -90, 90, 0]`

The same left/right joint poses are exported as GELLO calibration-position
constants so bimanual teleoperation can use matching reference poses.
