# Project Progress Report: my_bot ROS2 Simulation

## Project Overview

- Project name: `my_bot`
- Workspace root: `/home/navbot/dev_ws`
- Package location: `/home/navbot/dev_ws/src/my_bot`
- Purpose: ROS2 robot simulation package with a 3-wheeled autorickshaw-style robot model, Gazebo/ROS-Gazebo bridge integration, and robot state publishing.
- Simulation target: ROS2 (Humble/Iron) + `ros_gz_sim` + Gazebo world support.

## Contents of the Report

- Project structure and locations
- Package metadata and dependencies
- Robot description and model structure
- Inertia calculations and geometry
- Launch procedures and runtime configuration
- Build and setup commands
- Execution steps and verification

---

## 1. Workspace Structure

Root workspace directories:

- `build/` : build outputs produced by `colcon build`
- `install/` : installation outputs and sourced setup files
- `log/` : build logs and runtime logs
- `src/` : source code root for ROS2 packages

Package source path:

- `src/my_bot/`
    - `CMakeLists.txt`
    - `package.xml`
    - `launch/`
    - `description/`
    - `config/`
    - `worlds/`

The requested report file is created at the workspace root:

- `PROJECT_PROGRESS_REPORT.md`

---

## 2. Package Metadata

### `src/my_bot/package.xml`

- Package name: `my_bot`
- Version: `0.0.0`
- Build system: `ament_cmake`
- Maintainer: `MY NAME <my_email@email.com>`
- Declared dependencies:
  - `ament_cmake`
  - `ament_lint_auto` (test only)
  - `ament_lint_common` (test only)

### `src/my_bot/CMakeLists.txt`

- Uses `cmake_minimum_required(VERSION 3.5)`
- Defaults C standard to C99 and C++ standard to C++14
- Adds strict compiler warnings for GCC/Clang (`-Wall -Wextra -Wpedantic`)
- Finds `ament_cmake` and, when testing, `ament_lint_auto`
- Installs package resource directories:
  - `config`
  - `description`
  - `launch`
  - `worlds`
- Declares `ament_package()` to finalize package build behavior

---

## 3. Robot Description and Model Design

### Package layout used by robot description

- `src/my_bot/description/robot.urdf.xacro`
- `src/my_bot/description/robot_core.xacro`
- `src/my_bot/description/gazebo_control.xacro`
- `src/my_bot/description/lidar.xacro`
- `src/my_bot/description/inertial_macros.xacro`

The robot description is generated from a top-level Xacro file that includes the robot core, Gazebo control plugins, and a laser sensor definition.

### Top-level robot description

- `robot.urdf.xacro` includes:
  - `robot_core.xacro`
  - `gazebo_control.xacro`
  - `lidar.xacro`

This produces a complete URDF with links, joints, inertial values, and Gazebo plugin configuration.

### Robot structural hierarchy

- `base_link` : root link of the robot
- `chassis` : fixed to `base_link`
- `left_wheel` : continuous joint attached to `base_link`
- `right_wheel` : continuous joint attached to `base_link`
- `caster_wheel` : fixed joint attached to `chassis`
- `laser_frame` : fixed joint attached to `chassis`

### Motors and drive configuration

- Left wheel joint: `left_wheel_joint`
- Right wheel joint: `right_wheel_joint`
- DiffDrive plugin uses these joints
- ROS command input topic: `cmd_vel`
- Odometry output topic: `odom`
- TF topic: `tf`
- `robot_state_publisher` publishes `robot_description`

---

## 4. Geometry, Dimensions, and Inertia Calculations

### Chassis

- Type: box
- Size: `0.3 x 0.3 x 0.15` meters
- Mass: `0.5 kg`
- Visual and collision origin: `xyz="0.15 0 0.075"`
- Chassis joint origin relative to `base_link`: `xyz="-0.1 0 0"`

Calculation note:

- A box's inertia about its center of mass:
  - `Ixx = (1/12) * mass * (y^2 + z^2)`
  - `Iyy = (1/12) * mass * (x^2 + z^2)`
  - `Izz = (1/12) * mass * (x^2 + y^2)`

For the chassis with mass `0.5 kg` and dimensions `x=0.3`, `y=0.3`, `z=0.15`:

- `Ixx = (1/12) * 0.5 * (0.3^2 + 0.15^2)`
  - = `0.0416667 * (0.09 + 0.0225)`
  - = `0.0416667 * 0.1125`
  - = `0.0046875 kg·m^2`
- `Iyy = (1/12) * 0.5 * (0.3^2 + 0.15^2)`
  - same as `Ixx` = `0.0046875 kg·m^2`
- `Izz = (1/12) * 0.5 * (0.3^2 + 0.3^2)`
  - = `0.0416667 * 0.18`
  - = `0.0075 kg·m^2`

### Left and Right Wheels

- Type: cylinders
- Radius: `0.05 m`
- Length: `0.04 m`
- Mass: `0.1 kg`
- Joint origins:
  - Left wheel: `xyz="0 0.175 0" rpy="-${pi/2} 0 0"`
  - Right wheel: `xyz="0 -0.175 0" rpy="${pi/2} 0 0"`
- Wheel separation for drive plugin: `0.35 m`

Calculation note for cylinders:

- Cylinder inertia formula for axis along cylinder length:
  - `Ixx = Iyy = (1/12) * mass * (3*radius^2 + length^2)`
  - `Izz = (1/2) * mass * radius^2`

For mass `0.1 kg`, `radius = 0.05 m`, and `length = 0.04 m`:

- `Ixx = Iyy = (1/12) * 0.1 * (3*0.05^2 + 0.04^2)`
  - = `0.0083333 * (3*0.0025 + 0.0016)`
  - = `0.0083333 * (0.0075 + 0.0016)`
  - = `0.0083333 * 0.0091`
  - = `7.5833e-05 kg·m^2`
- `Izz = (1/2) * 0.1 * 0.05^2`
  - = `0.05 * 0.0025`
  - = `0.000125 kg·m^2`

### Caster Wheel

- Type: sphere
- Radius: `0.05 m`
- Mass: `0.1 kg`
- Origin relative to `chassis`: `xyz="0.24 0 0"`
- Joint type: `fixed`

Calculation note for sphere inertia:

- `Ixx = Iyy = Izz = (2/5) * mass * radius^2`

For mass `0.1 kg`, `radius = 0.05 m`:

- `I = 0.4 * 0.1 * 0.0025`
- `I = 0.0001 kg·m^2`

### Laser Sensor Mount

- Laser joint: `laser_joint`
- Parent: `chassis`
- Child: `laser_frame`
- Origin: `xyz="0.1 0 0.175"`
- Laser model: ray sensor, 360 samples, range `0.3` to `12` meters, update rate `10 Hz`
- Output topic: `scan`

---

## 5. Launch Files and Runtime Flow

### Primary launch file: `src/my_bot/launch/launch_sim.launch.py`

This launch file performs:

1. Include the `rsp.launch.py` launch description from the same package.
2. Declare a `world` launch argument and default to `worlds/simple_world.world`.
3. Include `ros_gz_sim` Gazebo launch file with `gz_args`.
4. Spawn the entity from the `robot_description` topic.
5. Launch the ROS-Gazebo bridge using `config/gz_bridge.yaml`.

### Robot state publisher launch file: `src/my_bot/launch/rsp.launch.py`

This launch file performs:

1. Loads the URDF Xacro file `description/robot.urdf.xacro`.
2. Processes Xacro to XML using `xacro.process_file(...)`.
3. Starts `robot_state_publisher` with parameters:
   - `robot_description`
   - `use_sim_time`

### Gazebo bridge configuration: `src/my_bot/config/gz_bridge.yaml`

Mapping between Gazebo and ROS topics:

- `clock` : `rosgraph_msgs/msg/Clock`
- `cmd_vel` : `geometry_msgs/msg/Twist`
- `odom` : `nav_msgs/msg/Odometry`
- `tf` : `tf2_msgs/msg/TFMessage`
- `joint_states` : `sensor_msgs/msg/JointState`

This ensures ROS topics are bridged correctly for simulation control and feedback.

### World file: `src/my_bot/worlds/simple_world.world`

World description includes:

- Sun directional light
- Ground plane of size `100 x 100`
- Static boxes and cylinders placed in the world for obstacles
- Simple environment for testing robot motion

---

## 6. Build, Setup, and Execution Instructions

### Prerequisites

- ROS2 Humble or Iron installed
- `ros_gz_sim` and `ros_gz_bridge` packages installed
- `xacro` installed
- Ament/CMake build environment available

### Recommended shell setup

Before building the package, source the ROS2 installation in your shell. Example:

```bash
source /opt/ros/humble/setup.bash
```

If using Iron replace `humble` with `iron`.

### Build the workspace

From the workspace root `/home/navbot/dev_ws`:

```bash
colcon build --symlink-install
```

If build artifacts already exist and you need a clean rebuild:

```bash
rm -rf build install log
colcon build --symlink-install
```

### Source the workspace

```bash
source install/setup.bash
```

### Run the simulation

From the workspace root after sourcing:

```bash
ros2 launch my_bot launch_sim.launch.py
```

This will:

- Launch `ros_gz_sim` with the world file
- Start `robot_state_publisher`
- Spawn the robot in Gazebo
- Start the ROS-Gazebo bridge

### Test robot control

After launch, send velocity commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1" --once
```

Verify telemetry:

```bash
ros2 topic echo /odom
ros2 topic echo /tf
ros2 topic echo /joint_states
ros2 topic echo /scan
```

---

## 7. Project Status and Work Completed

### Completed items

- Robot model structure implemented with URDF/Xacro
- Chassis, wheels, caster wheel, and laser frame defined
- Inertial macros added for sphere, cylinder, and box geometry
- Gazebo diffusion drive plugin configured for `cmd_vel`, `odom`, `tf`
- ROS-Gazebo bridge configured using YAML topic mapping
- Launch files created for simulation startup and robot state publishing
- World file included for testing with static objects

### Known limitations and future improvements

- The package metadata still contains placeholder description and license values and should be updated.
- The `launch_sim.launch.py` uses package name `my_bot` hard-coded; if package is renamed, update the package string.
- The rear caster wheel is fixed and does not simulate actual steering behavior.
- Laser sensor configuration uses a generic plugin and may require ROS2-native sensor plugin adjustments.

---

## 8. Detailed Calculations and Physics Notes

### Wheel separation and motion

- Left wheel position: `y = +0.175 m`
- Right wheel position: `y = -0.175 m`
- Total wheel separation: `0.175 m + 0.175 m = 0.35 m`
- This separation is used directly in the DiffDrive plugin: `wheel_separation = 0.35`

### Wheel kinematics

For differential drive, the linear velocity `v` and angular velocity `omega` relate to wheel velocities `vl` and `vr` by:

- `v = (vr + vl) / 2`
- `omega = (vr - vl) / wheel_separation`

Where `wheel_separation = 0.35 m`.

### Inertia formulas used in macros

- Box inertia: `Ixx = (1/12) * m * (y^2 + z^2)`
- Cylinder inertia: `Ixx = Iyy = (1/12) * m * (3*r^2 + l^2)`, `Izz = (1/2) * m * r^2`
- Sphere inertia: `I = (2/5) * m * r^2`

These calculations produce realistic inertial values for the Gazebo physics engine.

---

## 9. How to Understand Project Structure Quickly

1. Start at `src/my_bot/launch/launch_sim.launch.py` to see the runtime launch flow.
2. Open `src/my_bot/launch/rsp.launch.py` to understand how `robot_state_publisher` is launched.
3. Inspect `src/my_bot/description/robot.urdf.xacro` to see the top-level robot description.
4. Read `src/my_bot/description/robot_core.xacro` for link and joint definitions.
5. Review `src/my_bot/description/gazebo_control.xacro` for simulation plugin settings.
6. Review `src/my_bot/config/gz_bridge.yaml` to understand ROS/Gazebo topic mappings.
7. Check `src/my_bot/worlds/simple_world.world` for the simulation environment content.

---

## 10. Additional Notes

- There is currently no `README.md` at the workspace root; the project uses package-level documentation inside `src/my_bot/README.md`.
- This report file is intentionally created outside the `src/` folder, at the workspace root, to meet the requirement.
- If the package name changes, update both `package.xml` and `launch_sim.launch.py` accordingly.

---

## 11. File Location

- Progress report file created at: `/home/navbot/dev_ws/PROJECT_PROGRESS_REPORT.md`

