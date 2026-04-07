---
name: r2-sim-patterns
description: Coding patterns for R2 robot simulation (ROS 2 + Gazebo, Robocon 2026)
version: 1.0.0
source: local-git-analysis
analyzed_commits: 6
---

# R2 Sim Patterns

## Project Overview

ROS 2 Python package (`ament_python` build type) for simulating the R2 robot in Gazebo for Robocon 2026. The robot uses mecanum wheels, a rocker-bogie suspension, a lift mechanism, and a gripper. The arena is the "Meihua Forest" (梅花桩) challenge.

## Commit Conventions

This project uses **short imperative descriptions** without conventional commit prefixes:

- `update r2 model`
- `using A* algorithm for r2`
- `add r1 gazebo`
- `add spearhead action`
- `update world`

No consistent prefix convention yet. Consider adopting `feat:` / `fix:` / `chore:` for clarity.

## Code Architecture

```
r2_sim/
├── config/                  # RViz and ros_gz_bridge YAML configs
│   ├── r2_robot.rviz
│   └── ros_gz_bridge.yaml
├── launch/                  # ROS 2 launch files (Python)
│   ├── auto.launch.py       # Full autonomous stack
│   ├── r2_gazebo.launch.py  # Gazebo simulation only
│   ├── r2_rviz.launch.py    # RViz visualization only
│   ├── r2_teleop.launch.py  # Teleoperation mode
│   └── sim.launch.py        # Minimal sim launch
├── meshes/                  # STL mesh files for robot links
├── r2_sim/                  # Python ROS 2 nodes
│   ├── astar_planner.py     # A* path planning (426 lines)
│   ├── kfs_collector.py     # KFS collection logic (521 lines)
│   ├── navigator.py         # Navigation node (156 lines)
│   └── r1_sim.py            # R1 opponent simulation (189 lines)
├── scripts/                 # Utility scripts (mesh generators, layout tools)
│   ├── apply_kfs_layout.py  # Apply KFS positions from YAML
│   └── generate_wheel_bracket.py  # Procedural STL generation
├── urdf/                    # Robot description (XACRO)
│   └── r2_robot.urdf.xacro  # Main robot model
├── worlds/                  # Gazebo SDF world files
│   └── meihua_forest.sdf    # Competition arena
├── package.xml              # ROS 2 package manifest
└── setup.py                 # ament_python setup
```

## Key Patterns

### 1. Procedural Mesh Generation

STL meshes are generated programmatically via Python scripts in `scripts/` rather than using CAD software. This allows version-controlled, parameterized geometry.

**Pattern:** Create a Python script that uses `numpy-stl` or raw triangle generation to produce `.STL` files, then reference them in URDF/XACRO.

```python
# scripts/generate_*.py → meshes/*.STL
# Then in URDF: <mesh filename="package://r2_sim/meshes/part.STL"/>
```

### 2. XACRO Robot Description

The robot model uses XACRO macros for reusable components. Previously split into `urdf/components/*.xacro` files (chassis, tower, wheel), now consolidated into `r2_robot.urdf.xacro`.

### 3. Launch File Composition

Launch files are Python-based and compose different operational modes:
- **auto**: Full autonomous stack (planner + navigator + collector)
- **gazebo**: Simulation environment only
- **teleop**: Manual control for testing
- **rviz**: Visualization only

### 4. ROS 2 Node Pattern

Nodes follow the standard ROS 2 Python pattern:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Publishers, subscribers, timers, services

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

Entry points registered in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'navigator = r2_sim.navigator:main',
        'kfs_collector = r2_sim.kfs_collector:main',
    ],
},
```

### 5. Gazebo-ROS Bridge

Topics bridged between Gazebo and ROS 2 are configured in `config/ros_gz_bridge.yaml`. Common bridges:
- `/cmd_vel` (Twist) — velocity commands
- `/odom` (Odometry) — robot odometry
- `/joint_states` (JointState) — joint positions
- `/clock` (Clock) — simulation time

### 6. World and Model Organization

Custom Gazebo models live under `worlds/models/` with standard structure:
```
worlds/models/<model_name>/
├── model.config
├── model.sdf (or model.slf)
└── meshes/
    └── <model>.stl
```

## Workflows

### Adding a New Robot Component

1. Create mesh: add Python generator in `scripts/generate_*.py` or place `.STL` in `meshes/`
2. Define URDF: add link + joint in `urdf/r2_robot.urdf.xacro`
3. Bridge topics: update `config/ros_gz_bridge.yaml` if new sensors/actuators
4. Update `setup.py`: ensure `data_files` globs include new directories

### Adding a New ROS 2 Node

1. Create `r2_sim/<node_name>.py` with `Node` subclass and `main()` function
2. Register in `setup.py` under `console_scripts`
3. Add to relevant launch file(s) in `launch/`

### Modifying the Arena

1. Edit `worlds/meihua_forest.sdf`
2. Add custom models under `worlds/models/`
3. Update `config/kfs_layout.yaml` for KFS positions
4. Run `apply_kfs_layout.py` to apply changes

## Hot Files

Files that change most frequently (co-change often):
- `r2_sim/kfs_collector.py` — core game logic, changes with every strategy update
- `launch/auto.launch.py` — updated whenever nodes are added/reconfigured
- `worlds/meihua_forest.sdf` — arena layout iterations

## Dependencies

- **ROS 2** (ament_python build)
- **Gazebo** via `ros_gz_sim` and `ros_gz_bridge`
- **xacro** for URDF processing
- **robot_state_publisher** for TF broadcasting
- Python: `numpy-stl` for mesh generation scripts

## Testing

Currently minimal — only default ament linting tests (`flake8`, `pep257`, `copyright`). No unit or integration tests for ROS 2 nodes yet.
