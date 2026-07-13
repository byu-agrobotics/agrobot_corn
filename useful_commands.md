# Agrobot ROS2 Startup Commands

Here are the commands you need to run your various nodes and launch files. The workspace sourcing command has been included with each one for easy copy-pasting.

## Launch Files

### Mini Launch
Starts the `tof_sensor` node and the `mini_nav` (mini_nav_state_machine) node simultaneously.

```bash
cd ~/ros2_ws
colcon build --packages-select agrobot_nav
source ~/ros2_ws/install/setup.bash
ros2 launch agrobot_nav mini_agrobot_launch.py
```

## Individual Nodes

### 3. Keyboard Input Node
Publishes your keyboard commands (like 'w' to start or 'q' to stop).
```bash
source ~/ros2_ws/install/setup.bash
ros2 run agrobot_nav input
```

If you want to run the nodes individually instead of using the launch file, use the following commands:

### 1. Nav State Machine
Runs the autonomous navigation state machine.
```bash
source ~/ros2_ws/install/setup.bash
ros2 run agrobot_nav nav
```

### 2. Mini Nav State Machine
Runs the mini version of the navigation state machine.
```bash
source ~/ros2_ws/install/setup.bash
ros2 run agrobot_nav mini_nav
```

### 3. WASD Drive Control Node
If you want to bypass the autonomous state machine and manually drive using the keyboard input, run the `drive_control` node:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run agrobot_nav drive_control
```
Debugging:
If