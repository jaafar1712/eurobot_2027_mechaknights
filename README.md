# Eurobot 2027 — MechaKnights

ROS 2 software for the MechaKnights robot in [Eurobot 2027](https://www.eurobot.org/), "The Legend of Camelot". The robot has 100 seconds to collect cardboard **stones** from quarries on the table and build a castle with them (walls, towers and a gate). It also has to raise its Grail and finish the match back in its start area.

New to the project? Start with:
1. [docs/RULES_2027.md](docs/RULES_2027.md): a one-page summary of the game, the table and the points.
2. [WSL2_GPU_DEVELOPMENT_GUIDE.md](WSL2_GPU_DEVELOPMENT_GUIDE.md): if you work on Windows.
3. [Building](#building) and [Running the simulation](#running-the-simulation) below.

---

## Table of Contents

- [Project status](#project-status)
- [Architecture](#architecture)
- [Packages](#packages)
- [Simulation world](#simulation-world)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Running the simulation](#running-the-simulation)
- [Configuration](#configuration)
- [Branching strategy](#branching-strategy)

---

## Project status

This repository started from the team's Eurobot 2026 robot, whose game was collecting small crates. The parts of that robot that don't depend on the game have been kept. The game-specific parts still have to be adapted to stones.

| Part | Status |
| --- | --- |
| Gazebo arena (2027 table, playmat, 30 stones) | ✅ Ready |
| Robot model: differential drive, lidar, camera, 2-finger gripper | ✅ Works. The gripper was sized for the 2026 crates |
| ROS ↔ Gazebo bridges, odometry, `cmd_vel` | ✅ Works |
| SLAM (slam_toolbox) and Nav2 | ✅ Works. The lidar sees the table borders, see [Simulation world](#simulation-world) |
| ArUco perception | ⚠️ Still looks for 2026 crate markers (36/47). Stones use marker **13** |
| Task manager (match strategy) | ⚠️ Still runs the 2026 crate mission |
| `eurobot_interfaces` messages | ⚠️ Still named `Crate…` / `Pantry…` |

The "Crate", "pantry" and "nest" names in the code come from the 2026 game. They will be renamed as each part is adapted.

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│          Task Manager (mission state machine)   │
└────────┬──────────────┬──────────────┬──────────┘
         │              │              │
   ┌─────▼──────┐ ┌─────▼──────┐ ┌─────▼──────────┐
   │ Perception │ │ Navigation │ │  Manipulation  │
   │ camera +   │ │ SLAM + Nav2│ │ gripper state  │
   │ ArUco      │ │            │ │ machine        │
   └────────────┘ └────────────┘ └────────────────┘
              ▲               ▲               ▲
              └─── ROS ↔ Gazebo bridges ──────┘
                   (simulated robot + arena)
```

**Main ROS 2 topics:**

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `Twist` | Robot velocity commands |
| `/odom` | `Odometry` | Wheel odometry |
| `/scan` | `LaserScan` | 360° lidar |
| `/camera`, `/camera_info` | `Image`, `CameraInfo` | Front camera, 1280×960 |
| `/map` | `OccupancyGrid` | SLAM map |
| `/gripper/command` | `String` | `open` / `close` |
| `/gripper/state` | `String` | Current gripper state |
| `/gripper/left/contact`, `/gripper/right/contact` | `Contacts` | Finger contact sensors |
| `/crate/detections` | `CrateDetectionArray` | ArUco detections (2026 naming) |
| `/task_manager/state` | `String` | Current mission state |

---

## Packages

All packages are in `src/`.

### `mechaknights_2027`
Simulation and launch files: the robot model, the Gazebo arena and RViz config.

- `launch/full_system.launch.py`: launches Gazebo, bridges, SLAM, Nav2, perception, RViz and teleop.
- `launch/arena.launch.py`: launches only Gazebo, the robot and RViz.
- `worlds/arena_world.sdf`: the 2027 arena (see [Simulation world](#simulation-world)).
- `models/simple_robot/`: the simulated robot (SDF for Gazebo, URDF for `robot_state_publisher`).
- `models/eurobot_table/`, `models/eurobot_stone/`: the official table and stone.
- `urdf/`, `meshes/`: CAD-exported robot description.

### `eurobot_navigation`
SLAM, path planning and the mission logic.

- `launch/online_async_launch.py` and `config/mapper_params_online_async.yaml`: SLAM with `slam_toolbox`.
- `config/nav2_params.yaml`: Nav2, using a DWA local planner.
- `task_manager.py`: the mission state machine, with the match timer and the return home before time runs out.
- `scan_frame_fixer.py`: fixes the lidar frame id so SLAM accepts the scans.
- `teleop_keyboard.py`: drive the robot from the keyboard.
- `launch/bridges.launch.py`: the ROS ↔ Gazebo topic bridges.

### `eurobot_perception`
Camera-based detection.

- `aruco_crate_perception.py`: ArUco detection (`DICT_4X4_50`) with temporal filtering and outlier rejection. Still configured for the 2026 markers.
- `pantry_detection.py`: 2026 drop-zone detection.

### `eurobot_manipulation`
- `gripper_controller.py`: 20 Hz gripper state machine (`initializing → idle → opening → closing → gripping`). It stops closing when both finger contact sensors touch something.

### `eurobot_interfaces`
Custom ROS 2 messages: `CrateDetection(Array)` and `PantryDetection(Array)`.

---

## Simulation world

`src/mechaknights_2027/worlds/arena_world.sdf`

- **Frame**: the origin is the front-left (yellow) corner of the table. X runs along the 3 m side towards blue, Y points away from the audience, and the table surface is at z = 0.
- **Table**: 3000 × 2000 mm official playmat, with 70 mm high borders.
- **Stones**: 30 stones in 10 quarries of 3 stacked stones. Each stone is 320 × 110 × 110 mm, 90 g, friction μ = 0.5, with ArUco ID 13 on its 4 long faces. A printable label for the real stones is in [assets/aruco/](assets/aruco/).
- **Robot**: spawns in the yellow throne room at (0.25, 1.0), facing blue.

> **Lidar height:** the table borders are only 70 mm tall, so the simulated lidar is mounted about 55 mm above the table to see them. At that height some beams hit the robot's own wheels. `scan_frame_fixer` drops every reading closer than 0.20 m (`self_filter_range` parameter) before publishing `/scan_fixed`, which SLAM and Nav2 use. If the real robot's lidar ends up at a different height, update `lidar_link` in both `model.sdf` and `simple_robot.urdf`.

---

## Prerequisites

- Ubuntu 22.04 (native, or WSL2 on Windows — see the [WSL2 guide](WSL2_GPU_DEVELOPMENT_GUIDE.md))
- ROS 2 Humble
- Gazebo Ignition **Fortress**, the version that ships with `ros-humble-ros-gz`
- Python 3.10

```bash
sudo apt install ros-humble-ros-gz ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control \
  ros-humble-image-view ros-humble-cv-bridge ros-humble-tf-transformations xterm
```

---

## Building

```bash
# The repository itself is the colcon workspace (packages are in src/)
git clone https://github.com/jaafar1712/eurobot_2027_mechaknights.git ~/mechaknights_ws
cd ~/mechaknights_ws

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

On WSL, keep the workspace in the Linux home folder (`~`) and not under `/mnt/c/`, because builds there are much slower.

To rebuild a single package: `colcon build --packages-select eurobot_navigation`

---

## Running the simulation

```bash
# Everything: Gazebo, SLAM, Nav2, perception, RViz, teleop
ros2 launch mechaknights_2027 full_system.launch.py

# Only the arena and the robot
ros2 launch mechaknights_2027 arena.launch.py
```

`full_system.launch.py` starts things in this order:

| Time | Component |
|------|-----------|
| t=0s | Gazebo, robot_state_publisher, RViz, ROS ↔ Gazebo bridges |
| t=5s | Scan frame fixer, ArUco perception |
| t=6s | ArUco debug image view |
| t=7s | SLAM (slam_toolbox) |
| t=10s | Nav2 |
| t=13s | Keyboard teleop (opens in an xterm window) |

**Useful commands:**

```bash
# Drive the robot from the keyboard
ros2 run eurobot_navigation teleop_keyboard

# Open / close the gripper
ros2 topic pub /gripper/command std_msgs/String "data: 'open'" --once
ros2 topic pub /gripper/command std_msgs/String "data: 'close'" --once

# Run the arena without any GUI (fast check that the world loads)
ign gazebo -s -r $(ros2 pkg prefix mechaknights_2027)/share/mechaknights_2027/worlds/arena_world.sdf
```

The launch files force software rendering (`LIBGL_ALWAYS_SOFTWARE=1`) because Gazebo's renderer crashes on some WSL GPU drivers. If you run `ign gazebo` by hand on WSL and it crashes with an `Ogre::UnimplementedException`, export the same variable first.

---

## Configuration

### Task manager parameters

| Parameter | Default | Description |
|---|---|---|
| `team_color` | `yellow` | Team side (`yellow` or `blue`) |
| `match_duration` | `100.0` | Match length in seconds |
| `nest_return_time` | `10.0` | Seconds before the end when the robot heads back to its start area |
| `alignment_distance` | `0.50` | Approach distance before fine alignment (m) |
| `alignment_tolerance` | `0.04` | Fine alignment tolerance (m) |
| `angle_tolerance` | `8.0` | Angular alignment tolerance (degrees) |

### Nav2
`src/eurobot_navigation/config/nav2_params.yaml`: local planner, costmap inflation and recovery behaviours.

---

## Branching strategy

```
main          ← stable, tested on the real robot
└── dev       ← integration branch (default)
    ├── feature/your-feature
    └── fix/your-fix
```

- Create every `feature/` or `fix/` branch from `dev`.
- Open a Pull Request into `dev`.
- `dev` gets merged into `main` after it has been tested on the physical robot.

---

> Built with ROS 2 Humble · Gazebo Fortress · Nav2 · slam_toolbox · OpenCV
