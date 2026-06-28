# lerobot_robot_ur5e

LeRobot robot plugin for a Universal Robots UR5e arm with an OnRobot gripper.

The LeRobot robot type is `ur5e`.

The gripper action is normalized as `0.0 = open` and `1.0 = closed`. By default
the package controls the RG2 through the `onRobot` Python package over the
OnRobot XML-RPC endpoint on port `41414`.

Install locally with:

```bash
uv pip install -e ./lerobot_robot_ur5e
```

Reset the robot and open the RG2:

```bash
python scripts/reset_ur5e.py --ip 192.168.0.10
```

Run GELLO teleoperation:

```bash
python scripts/teleoperate.py --robot-ip 192.168.0.10 --gello-port /dev/ttyUSB0
```
