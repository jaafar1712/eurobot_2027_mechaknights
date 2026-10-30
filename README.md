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
| Stone perception (ArUco 13) | ✅ Works. Camera error is 1–2 mm at 0.6 m and about 10 mm at 1.4 m. It reports stack level and top height (see [Checking accuracy](#checking-accuracy)) |
| Localisation after driving | ⚠️ 4–8 cm and up to 7° off after a few moves. This is now the main error source |
| Task manager (match strategy) | ⚠️ Still runs the 2026 crate mission |
| `eurobot_interfaces` messages | ✅ `StoneDetection(Array)` added. The 2026 `Crate…` / `Pantry…` ones are still there for the old task manager |

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
| `/stones/detections` | `StoneDetectionArray` | Stones seen by the camera, in the `table` frame |
| `/stones/debug` | `Image` | Camera image with the detections drawn on it |
| `/stones/markers` | `MarkerArray` | Detected stones for RViz |
| `/task_manager/state` | `String` | Current mission state |

---

## Packages

All packages are in `src/`.

### `mechaknights_2027`
Simulation and launch files: the robot model, the Gazebo arena and RViz config.

- `launch/full_system.launch.py`: launches Gazebo, bridges, SLAM, Nav2, perception, RViz and teleop.
- `launch/arena.launch.py`: launches only Gazebo, the robot and RViz.
- `launch/robot_spawn.launch.py`: spawns the robot for `team_color` and publishes the `table` frame. Both launch files above include it.
- `worlds/arena_world.sdf`: the 2027 arena (see [Simulation world](#simulation-world)).
- `models/simple_robot/`: the simulated robot (SDF for Gazebo, URDF for `robot_state_publisher`).
- `models/eurobot_table/`, `models/eurobot_stone/`: the official table and stone.
- `urdf/`, `meshes/`: CAD-exported robot description.

### `eurobot_navigation`
SLAM, path planning and the mission logic.

- `launch/online_async_launch.py` and `config/mapper_params_online_async.yaml`: SLAM with `slam_toolbox`.
- `config/nav2_params.yaml`: Nav2, using a DWA local planner.
- `task_manager.py`: the mission state machine, with the match timer and the return home before time runs out.
- `field.py`: the table layout from the rules, with yellow/blue mirroring helpers.
- `field_markers.py`: draws the table (outline, throne rooms, stables, quarries) in RViz.
- `table_localizer.py`: corrects the robot position with the fixed table markers (see [Localisation with the table markers](#localisation-with-the-table-markers)).
- `scan_frame_fixer.py`: fixes the lidar frame id so SLAM accepts the scans.
- `teleop_keyboard.py`: drive the robot from the keyboard.
- `launch/bridges.launch.py`: the ROS ↔ Gazebo topic bridges.

### `eurobot_perception`
Camera-based detection.

- `stone_perception.py`: finds stones from their ArUco marker (ID 13, 80 mm). For each stone it reports the centre position (x, y, z) in the `table` frame, its yaw, its stack level, its top-face height (`top_z`, for a gripper coming from above), whether it is the top of its stack, and its distance and bearing from the robot. The camera mounting comes from TF (the URDF), not from parameters.
- `stone_accuracy.py`: simulation-only check that compares the detections with Gazebo's ground truth (see [Checking accuracy](#checking-accuracy)).
- `aruco_crate_perception.py`, `pantry_detection.py`: 2026 crate and drop-zone detection, kept for reference.

### `eurobot_manipulation`
- `gripper_controller.py`: 20 Hz gripper state machine (`initializing → idle → opening → closing → gripping`). It stops closing when both finger contact sensors touch something.

### `eurobot_interfaces`
Custom ROS 2 messages: `StoneDetection(Array)` for 2027, plus the 2026 `CrateDetection(Array)` and `PantryDetection(Array)`.

---

## Simulation world

`src/mechaknights_2027/worlds/arena_world.sdf`

- **Frame**: the origin is the front-left (yellow) corner of the table. X runs along the 3 m side towards blue, Y points away from the audience, and the table surface is at z = 0.
- **Table**: 3000 × 2000 mm official playmat, with 70 mm high borders.
- **Stones**: 30 stones in 10 quarries of 3 stacked stones. Each stone is 320 × 110 × 110 mm, 90 g, friction μ = 0.5, with ArUco ID 13 on its 4 long faces. A printable label for the real stones is in [assets/aruco/](assets/aruco/).
- **Robot**: not part of the world file. The launch files spawn it in the throne room of the chosen team: yellow (0.25, 1.0) facing +X, or blue (2.75, 1.0) facing −X.

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
ros2 launch mechaknights_2027 full_system.launch.py team_color:=yellow

# Only the arena and the robot
ros2 launch mechaknights_2027 arena.launch.py team_color:=blue
```

**Choose the team colour before starting.** At a match the colour is given at the last moment, and the robot has to work for both. `team_color` (`yellow` by default, or `blue`) does three things:

- spawns the robot in that team's throne room, facing the opponent,
- places the `table` frame so positions match the rule book,
- passes the colour to the nodes that need it, such as perception.

### Coordinate frames

```text
table ──(table_localizer)──> map ──(SLAM)──> odom ──(wheel odometry)──> base_footprint ──> base_link
```

- **`table`** is the rule-book frame: origin at the yellow front-left corner, X towards blue, Y away from the audience. **Use it for anything on the table**: goals, quarries, zones. RViz uses it as its fixed frame, so the view looks like the real table.
- `map` is SLAM's frame. Its origin is wherever the robot started, so it differs for yellow and blue. Avoid hard-coding positions in it.
- `base_footprint` is the robot's centre on the table surface (z = 0); `base_link` is the body, 40 mm above it.

### Localisation with the table markers

SLAM drifts as the robot drives (several cm and degrees after a few moves). The playmat has 4 fixed ArUco markers, IDs 20–23, at (0.6, 1.4), (2.4, 1.4), (0.6, 0.6) and (2.4, 0.6). Whenever the camera sees one closer than 1.3 m, `table_localizer` measures the robot's true position from it, to about 5 mm and 0.5°, and gradually corrects `table → map`. Nav2 works in `map`, so it never sees a jump.

The strategy should therefore **look at a table marker now and then**, for example before an accurate pickup or a build. `marker_localization:=false` switches the correction off, for comparison. Each accepted fix is logged and published on `/localization/marker_pose`.
- Nav2 accepts goals in `table` directly: `header.frame_id: table`.

### Table layout in code

Every position from the rules (throne rooms, stables, quarries, stone size and its ArUco ID) is in [`eurobot_navigation/field.py`](src/eurobot_navigation/eurobot_navigation/field.py). Team-owned zones are written once for yellow and mirrored for blue (the table is symmetric about x = 1.5 m):

```python
from eurobot_navigation import field

x, y, yaw = field.start_pose(team)                  # our throne room
home = field.own(team, field.YELLOW_THRONE_ROOM)    # our zone, either colour
for qx, qy, qyaw in field.QUARRIES: ...             # stone quarries at the start
```

Don't type table coordinates anywhere else. Add them to `field.py`, and `field_markers` will also draw them in RViz.

`full_system.launch.py` starts things in this order:

| Time | Component |
|------|-----------|
| t=0s | Gazebo, robot_state_publisher, RViz, ROS ↔ Gazebo bridges |
| t=5s | Scan frame fixer, stone perception |
| t=6s | Camera preview with the detected stones (`/stones/debug`) |
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

### Checking accuracy

In simulation, Gazebo knows exactly where every stone and the robot are, so positions can be measured instead of judged by eye. With the simulation running:

```bash
ros2 run eurobot_perception stone_accuracy --ros-args -p use_sim_time:=true
```

Every few seconds it prints each visible stone with three errors:

- **camera**: stone position relative to the robot, measured vs true. This is the perception error.
- **total**: stone position on the table, reported vs true. This is what the strategy actually uses: camera error plus localisation error.
- **height**: stone centre height (z), reported vs true. Check this when grabbing from above.

It also prints the robot's **localisation error** (estimated vs true pose), plus running mean and max values. When the total error is large, these numbers show whether to work on perception or on localisation.

On the real table there is no ground truth. Instead, place stones at known positions (measured with a tape from the table corner, or the quarry marks on the mat) and compare them with `/stones/detections`.

---

## Configuration

### Task manager parameters

| Parameter | Default | Description |
|---|---|---|
| `team_color` | `yellow` | Team side (`yellow` or `blue`). Set it with the launch argument, not here |
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
