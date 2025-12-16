# Sliding Mode Controller for Crazyflie 2.0 Quadrotor

C++ implementation of boundary layer-based sliding mode control for trajectory tracking in Gazebo simulation with **custom trajectory support**.

## Overview

This ROS package implements a robust sliding mode controller (SMC) for the Crazyflie 2.0 quadrotor. The controller tracks quintic polynomial trajectories through multiple waypoints with high precision and supports arbitrary user-defined trajectories via ROS parameters.

**Key Features:**
- ✅ Boundary layer SMC for altitude and attitude control
- ✅ Quintic polynomial trajectory generation with zero velocity/acceleration at waypoints
- ✅ **Custom trajectory support** via ROS parameters or programmatic API
- ✅ Gyroscopic coupling compensation
- ✅ Native C++ implementation for real-time performance
- ✅ Trajectory visualization and analysis tools
- ✅ Production-ready with validated performance (mean error <0.02m)

## Requirements

- ROS Noetic (Ubuntu 20.04)
- Gazebo 11
- CrazyS simulation package
- Eigen3
- C++14 compiler

## Package Structure

```
smc_quadrotor_cpp/
├── include/smc_quadrotor/
│   ├── quadrotor_control.h
│   └── trajectory_generator.h       # Custom trajectory API
├── src/
│   ├── quadrotor_control.cpp        # ROS node with parameter loading
│   ├── trajectory_generator.cpp
│   └── quadrotor_control_node.cpp
├── launch/
│   ├── quadrotor_control.launch            # Default trajectory
│   └── custom_trajectory_example.launch    # Figure-8 example (validated)
├── config/
│   └── smc_controller.yaml
├── visualize_trajectory.py          # Trajectory analysis tool
├── README.md
└── TRAJECTORY_GUIDE.md
```

## Installation

```bash
# Clone CrazyS dependencies
cd ~/catkin_ws/src
git clone https://github.com/gsilano/CrazyS.git
git clone https://github.com/gsilano/mav_comm.git

# Clone this package
git clone <repository-url> smc_quadrotor_cpp

# Build
cd ~/catkin_ws
catkin build smc_quadrotor_cpp
source devel/setup.bash
```

## Usage

### Default Trajectory
```bash
# Launch simulation with default square trajectory
roslaunch smc_quadrotor_cpp quadrotor_control.launch
```

The default trajectory executes through 5 waypoints over 65 seconds:
```
(0,0,0) → (0,0,1) → (1,0,1) → (1,1,1) → (0,1,1) → (0,0,1)
```

### Custom Trajectories

#### Method 1: Load from File (Recommended for Large Trajectories)

For trajectories with many waypoints, use CSV or YAML files:

**CSV Format (`trajectories/square_simple.csv`):**
```csv
# Format: x, y, z, time
# time = duration (seconds) to reach this waypoint from previous one
x, y, z, time
0.0, 0.0, 0.0, 5.0
0.0, 0.0, 1.0, 10.0
1.0, 0.0, 1.0, 15.0
1.0, 1.0, 1.0, 15.0
0.0, 0.0, 0.5, 20.0
```

**YAML Format (`trajectories/lawnmower.yaml`):**
```yaml
# time = duration (seconds) to reach this waypoint from previous one
waypoints:
  - {x: 0.0, y: 0.0, z: 0.0, time: 5.0}
  - {x: 0.0, y: 0.0, z: 1.5, time: 10.0}
  - {x: 2.0, y: 0.0, z: 1.5, time: 15.0}
  # ... more waypoints (16 total for lawnmower pattern)
```

**Launch with file:**
```bash
# Use included spiral trajectory (20 waypoints)
roslaunch smc_quadrotor_cpp trajectory_from_file.launch

# Or specify custom file
roslaunch smc_quadrotor_cpp trajectory_from_file.launch trajectory_file:=/path/to/your/trajectory.csv

# Test with simple square
roslaunch smc_quadrotor_cpp trajectory_from_file.launch trajectory_file:='$(find smc_quadrotor_cpp)/trajectories/square_simple.csv'
```

**In your launch file:**
```xml
<node name="quadrotor_control" pkg="smc_quadrotor_cpp" type="quadrotor_control_node" output="screen">
    <param name="trajectory_file" value="$(find smc_quadrotor_cpp)/trajectories/spiral.csv"/>
</node>
```

#### Method 2: ROS Parameters (Simple Trajectories)

For small trajectories, use inline parameters:

```xml
<launch>
    <!-- Start Gazebo with Crazyflie -->
    <include file="$(find rotors_gazebo)/launch/crazyflie2_without_controller.launch"/>
    
    <!-- SMC Controller with custom trajectory -->
    <node name="quadrotor_control" pkg="smc_quadrotor_cpp" type="quadrotor_control_node" output="screen">
        <!-- Define trajectory waypoints (must have same length) -->
        <rosparam param="trajectory/waypoints_x">[0.0, 0.0, 1.5, 2.0, 1.5, 0.5, 0.0, 0.0, 0.0]</rosparam>
        <rosparam param="trajectory/waypoints_y">[0.0, 0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0]</rosparam>
        <rosparam param="trajectory/waypoints_z">[0.0, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.0]</rosparam>
        
        <!-- Time to spend on each segment (length = waypoints-1) -->
        <rosparam param="trajectory/time_segments">[5.0, 20.0, 15.0, 15.0, 15.0, 15.0, 15.0, 5.0]</rosparam>
    </node>
</launch>
```

**Validated Example: Figure-8 Trajectory**
```bash
roslaunch smc_quadrotor_cpp custom_trajectory_example.launch
```

Performance metrics from Gazebo testing:
- **9 waypoints**, 8 segments, **105 seconds** total mission time
- Max velocity: 0.561 m/s, Max acceleration: 0.344 m/s²
- Mean tracking error: **0.023 m**, Max error: 0.215 m
- Safe for Crazyflie 2.0: no motor saturation, smooth execution

#### Method 3: Programmatic API

For trajectories generated algorithmically or from external sources:

```cpp
#include <smc_quadrotor/trajectory_generator.h>

// Option A: Load from file
auto traj_gen = std::make_shared<TrajectoryGenerator>(false);
if (!traj_gen->loadFromFile("/path/to/trajectory.csv")) {
    ROS_ERROR("Failed to load trajectory file");
}

// Option B: Set programmatically
std::vector<Eigen::Vector3d> waypoints = {
    Eigen::Vector3d(0.0, 0.0, 0.0),    // Start at ground
    Eigen::Vector3d(0.0, 0.0, 1.5),    // Rise to altitude
    Eigen::Vector3d(2.0, 0.0, 1.5),    // Move forward
    Eigen::Vector3d(2.0, 2.0, 1.5),    // Move right
    Eigen::Vector3d(0.0, 0.0, 1.0)     // Return and descend
};

std::vector<double> time_segments = {5.0, 10.0, 10.0, 15.0};

if (!traj_gen->setTrajectory(waypoints, time_segments)) {
    ROS_ERROR("Failed to set trajectory");
}
```

**Trajectory Requirements:**
- Minimum **2 waypoints** (start and end)
- `time_segments.size()` **must equal** `waypoints.size() - 1`
- All time segments must be **positive**
- Validated at runtime with detailed error messages

**See `TRAJECTORY_GUIDE.md` for:**
- Detailed file format specifications
- More trajectory examples (vertical line, triangle, circle)
- Performance tuning guidelines
- Troubleshooting common issues
- Python script for generating trajectories

### Trajectory Visualization

Analyze trajectory before flight:

```bash
./visualize_trajectory.py \
    --waypoints_x "0.0,0.0,1.5,2.0,1.5,0.5,0.0,0.0,0.0" \
    --waypoints_y "0.0,0.5,1.0,0.5,0.0,-0.5,-1.0,-0.5,0.0" \
    --waypoints_z "0.0,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.0" \
    --time_segments "5.0,20.0,15.0,15.0,15.0,15.0,15.0,5.0" \
    --output trajectory_preview.png
```

Generates 6-plot analysis with position, velocity, acceleration profiles and statistics.

### Included Example Trajectories

| File | Waypoints | Duration | Description |
|------|-----------|----------|-------------|
| `square_simple.csv` | 5 | 65s | Basic square at 1m altitude |
| `spiral.csv` | 20 | 175s | 3D spiral ascent/descent |
| `lawnmower.yaml` | 16 | 160s | Coverage scanning pattern |

## Package Structure

```
smc_quadrotor_cpp/
├── include/smc_quadrotor/
│   ├── quadrotor_control.h
│   └── trajectory_generator.h       # Custom trajectory API with loadFromFile()
├── src/
│   ├── quadrotor_control.cpp        # ROS node with file/parameter loading
│   ├── trajectory_generator.cpp
│   └── quadrotor_control_node.cpp
├── launch/
│   ├── quadrotor_control.launch            # Default trajectory
│   ├── custom_trajectory_example.launch    # Figure-8 (9 waypoints)
│   └── trajectory_from_file.launch         # File-based loading
├── trajectories/
│   ├── square_simple.csv                   # 5 waypoints
│   ├── spiral.csv                          # 20 waypoints
│   └── lawnmower.yaml                      # 16 waypoints
├── config/
│   └── smc_controller.yaml
├── visualize_trajectory.py          # Trajectory analysis tool
├── README.md
└── TRAJECTORY_GUIDE.md
```

## Controller Parameters

### Physical (Crazyflie 2.0)
- Mass: 27g
- Arm length: 46mm
- Inertia: Ix=Iy=16.57×10⁻⁶, Iz=29.26×10⁻⁶ kg·m²

### Control Gains
- Position: Kp=90, Kd=10
- Altitude: λz=7, ηz=10
- Roll: λφ=12, ηφ=120
- Pitch: λθ=12, ηθ=120
- Yaw: λψ=8, ηψ=10

## Performance

### Default Trajectory
- Mean error: 0.018 m
- Max error: 0.091 m
- RMSE: 0.030 m
- Waypoint precision: <1mm

### Custom Trajectory (Figure-8 Example)
- Mission time: 105 seconds, 8.12 m total distance
- Mean tracking error: 0.023 m, Max error: 0.215 m
- Max velocity: 0.561 m/s, Max acceleration: 0.344 m/s²
- No motor saturation, smooth execution throughout

## Documentation

- **TRAJECTORY_GUIDE.md** - Complete trajectory design and loading reference
- **visualize_trajectory.py --help** - Trajectory visualization tool

## References

1. [CrazyS Simulator](https://github.com/gsilano/CrazyS)
2. Slotine & Li (1991). Applied Nonlinear Control.

## License

MIT
