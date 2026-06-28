# lerobot_robot_bi_ur3

LeRobot robot plugin that wraps two existing `lerobot_robot_ur3.UR3` robots as a single bimanual robot.

Actions and observations are prefixed with `left_` and `right_`, for example:

- `left_joint_0` ... `left_joint_5`, `left_gripper`
- `right_joint_0` ... `right_joint_5`, `right_gripper`

The package intentionally reuses the working single-arm UR3 implementation instead of duplicating RTDE or Robotiq logic.

Default arm assignment:

- Left UR3: `158.132.172.193`, calibration under `calibration/robots/bi_ur3/left`
- Right UR3: `158.132.172.214`, calibration under `calibration/robots/bi_ur3/right`

If a top-level `BiUR3Config(calibration_dir=...)` is provided, it is split into `left/` and `right/` subdirectories automatically.

You can still override any single-arm setting directly:

```python
from lerobot_robot_bi_ur3 import BiUR3Config
from lerobot_robot_ur3 import UR3Config

robot_cfg = BiUR3Config(
    left_arm_config=UR3Config(ip="158.132.172.193", calibration_dir="calibration/robots/left_ur3"),
    right_arm_config=UR3Config(ip="158.132.172.214", calibration_dir="calibration/robots/right_ur3"),
)
```
