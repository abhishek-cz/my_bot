# NavBot LiDAR Simulation — Startup Guide

## What This Does

The simulation launches a differential-drive robot in Gazebo with a 2D LiDAR sensor.
The robot automatically drives forward and turns away from obstacles detected by the LiDAR.
RViz shows the robot model and the live laser scan in real time.

---

## Prerequisites

Open a terminal and make sure your workspace is sourced:

```bash
cd ~/dev_ws
source install/setup.bash
```

---

## Step-by-Step: Start the Simulation

### Step 1 — Build the workspace (only needed after code changes)

```bash
cd ~/dev_ws
colcon build
source install/setup.bash
```

### Step 2 — Launch everything

```bash
cd ~/dev_ws
source install/setup.bash
ros2 launch my_bot launch_sim.launch.py
```

This single command starts:
- **Gazebo** with the world file (obstacles included)
- **robot_state_publisher** (publishes the robot model and TF tree)
- **ros_gz_bridge** (bridges LiDAR scan, odometry, cmd_vel, clock between Gazebo and ROS 2)
- **lidar_avoider** node (reads `/scan`, publishes `/cmd_vel` to avoid obstacles)
- **RViz** (visualises the robot and the laser scan)

---

## Step 3 — Verify the LiDAR is Working

In a **second terminal**:

```bash
cd ~/dev_ws
source install/setup.bash

# Check the /scan topic is publishing
ros2 topic hz /scan

# See raw scan data (Ctrl-C to stop)
ros2 topic echo /scan --once

# Check the avoider is sending velocity commands
ros2 topic echo /cmd_vel
```

Expected output from `ros2 topic hz /scan`:
```
average rate: 10.000
```

Expected output from the avoider log (visible in the launch terminal):
```
[lidar_avoider]: clear (2.50 m) — forward
[lidar_avoider]: obstacle at 0.55 m — turning right
```

---

## Step 4 — View the LiDAR Scan in RViz

RViz opens automatically. If the laser scan is not visible:

1. Click **Add** → **By topic** → `/scan` → **LaserScan** → OK
2. Set **Fixed Frame** to `base_link` (top-left Global Options panel)
3. The red/white dots around the robot are the live obstacle scan

---

## Verify Individual Components

```bash
# List all active topics
ros2 topic list

# Check which nodes are running
ros2 node list

# Inspect the scan topic publisher/subscriber details
ros2 topic info /scan -v

# Check the bridge is running
ros2 node info /ros_gz_bridge
```

---

## Stop the Simulation

Press **Ctrl-C** in the launch terminal. All nodes (Gazebo, RViz, bridge, avoider) shut down together.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `/scan` not listed in `ros2 topic list` | Bridge not started | Check launch terminal for bridge errors |
| `ros2 topic hz /scan` shows 0 Hz | Sensors plugin not loaded | Rebuild: `colcon build && source install/setup.bash` |
| Robot not moving | lidar_avoider not running | Check `ros2 node list` for `/lidar_avoider` |
| RViz opens empty | Fixed Frame wrong | Set Fixed Frame to `base_link` |
| Gazebo opens but robot missing | Spawn failed | Check launch terminal for spawn errors |

---

## Key Topics

| Topic | Direction | Description |
|---|---|---|
| `/scan` | Gazebo → ROS 2 | 2D LiDAR laser scan |
| `/cmd_vel` | ROS 2 → Gazebo | Velocity commands from the avoider |
| `/odom` | Gazebo → ROS 2 | Wheel odometry |
| `/tf` | Gazebo → ROS 2 | Transform tree |
| `/clock` | Gazebo → ROS 2 | Simulation time |
| `/robot_description` | ROS 2 internal | URDF model for RViz and RSP |
