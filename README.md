# magpie_control
breakout of submodule for ur5 and gripper control using the MAGPIE interface.

# Installation
To install `magpie_control` in a python environment, run the following commands once your environment is active:

```bash
git clone https://github.com/correlllab/magpie_control.git #clone this repository
cd magpie_control #enter the repo
pip install . --user #install magpie_control
```

Test out the functionality!

```bash
python3.10
import magpie_control

```

## ROS2 Gripper Node: Run and Control

If you see:

```bash
ros2 run magpie_control gripper_node
No executable found
```

the package is usually not built/sourced in your ROS2 workspace yet (or you are in a different shell that is not sourced).

If you see:

```bash
ModuleNotFoundError: No module named 'magpie_msgs'
```

`magpie_msgs` was not available in the current shell environment. Build both packages and source the workspace-wide setup file (not only a single package local setup).

### 1. Build and source from workspace root

From your ROS2 workspace root (example: `~/ws_ctrl`):

```bash
cd ~/ws_ctrl
colcon build --packages-select magpie_msgs magpie_control
source install/setup.bash
```

Optional sanity check:

```bash
ros2 pkg executables magpie_control
```

You should see at least:

- `magpie_control gripper_node`
- `magpie_control ft_sensor_node`
- `magpie_control tactile_sensor_node`

### 2. Run the gripper node

```bash
ros2 run magpie_control gripper_node
```

You can override parameters at startup:

```bash
ros2 run magpie_control gripper_node --ros-args \
	-p auto_detect_port:=true \
	-p port:=/dev/ttyUSB0 \
	-p default_speed:=100 \
	-p default_torque:=200
```

### 3. Control the gripper with ROS services

Open gripper:

```bash
ros2 service call /gripper/open std_srvs/srv/Trigger "{}"
```

Close gripper:

```bash
ros2 service call /gripper/close std_srvs/srv/Trigger "{}"
```

Set aperture/position (meters):

```bash
ros2 service call /gripper/set_position magpie_msgs/srv/SetGripperPosition "{position: 0.040}"
```

Set force limit (N):

```bash
ros2 service call /gripper/set_force magpie_msgs/srv/SetGripperForce "{max_force: 8.0}"
```

Calibrate:

```bash
ros2 service call /gripper/calibrate std_srvs/srv/Trigger "{}"
```

Monitor state:

```bash
ros2 topic echo /gripper/state
```

### 4. Common ROS2 troubleshooting

- Make sure your ROS distro is sourced before workspace setup:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_ctrl/install/setup.bash
```

- Avoid sourcing only one package setup (for example `install/magpie_control/local_setup.bash`) when running `gripper_node`; that can omit runtime dependencies such as `magpie_msgs`.

- Verify package visibility:

```bash
ros2 pkg list | grep magpie_control
```

- If executables are still missing, rebuild cleanly:

```bash
cd ~/ws_ctrl
rm -rf build/magpie_control install/magpie_control log
colcon build --packages-select magpie_control
source install/setup.bash
```

## Bring Up Fresh Dynamixel Motors

This repo includes a small CLI utility to find and configure new AX-12 motors.

1. Scan for motors on likely serial ports:

```bash
python -m magpie_control.dxl_setup scan --id-max 30
```

2. If two brand-new motors are attached at once (both default to ID 1), unplug one first.
Leave one motor as ID 1, then plug in the other motor by itself and change it to ID 2:

```bash
python -m magpie_control.dxl_setup set-id --port /dev/ttyACM0 --baud 1000000 --current-id 1 --new-id 2
```

3. Optional: set baudrate (e.g., to keep everything at 1,000,000):

```bash
python -m magpie_control.dxl_setup set-baud --port /dev/ttyACM0 --baud 57600 --id 2 --new-baud 1000000
```

4. Rescan and verify both IDs are visible at 1,000,000:

```bash
python -m magpie_control.dxl_setup scan --ports /dev/ttyACM0 --bauds 1000000 --id-max 10
```
