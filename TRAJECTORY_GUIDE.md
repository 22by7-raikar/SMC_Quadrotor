# Trajectory Design and Loading Guide

Complete guide for creating and loading custom trajectories for the SMC Quadrotor controller.

## Table of Contents
1. [Loading Methods](#loading-methods)
2. [File Formats](#file-formats)
3. [ROS Parameters](#ros-parameters)
4. [Programmatic API](#programmatic-api)
5. [Example Trajectories](#example-trajectories)
6. [Validation Rules](#validation-rules)
7. [Tips and Best Practices](#tips-and-best-practices)
8. [Troubleshooting](#troubleshooting)

---

## Loading Methods

The controller supports three methods with the following priority:

1. **File-based** (via `trajectory_file` parameter) - **Recommended for large trajectories**
2. **ROS Parameters** (via `trajectory/waypoints_*`) - Good for simple trajectories
3. **Default** - Built-in square pattern as fallback

### When to Use Each Method

| Method | Best For | Waypoint Limit | Pros |
|--------|----------|----------------|------|
| **File (CSV/YAML)** | 10+ waypoints | Unlimited | Clean launch files, scriptable, version control friendly |
| **ROS Parameters** | <10 waypoints | ~10-15 | Quick testing, no extra files |
| **Programmatic API** | Generated trajectories | Unlimited | Dynamic generation, integration with planners |

---

## File Formats

### CSV Format

Simple comma-separated format, ideal for script generation:

```csv
# Comments start with #
# Format: x, y, z, time
x, y, z, time
0.0, 0.0, 0.0, 5.0
0.0, 0.0, 1.0, 10.0
1.0, 0.0, 1.0, 15.0
1.0, 1.0, 1.0, 15.0
0.0, 0.0, 0.5, 20.0
```

**Field Definitions:**
- **x, y, z**: Waypoint position in meters
- **time**: Duration (seconds) to reach THIS waypoint from PREVIOUS one
  - First waypoint: initial delay/takeoff time
  - Subsequent: travel time between waypoints

**Usage:**
```bash
# Command line
roslaunch smc_quadrotor_cpp trajectory_from_file.launch trajectory_file:=/path/to/trajectory.csv

# In launch file
<param name="trajectory_file" value="$(find smc_quadrotor_cpp)/trajectories/spiral.csv"/>
```

### YAML Format

More readable, better for mission planning and documentation:

```yaml
# Mission trajectory with inline comments
waypoints:
  - {x: 0.0, y: 0.0, z: 0.0, time: 5.0}   # Ground start
  - {x: 0.0, y: 0.0, z: 1.5, time: 10.0}  # Takeoff to altitude
  - {x: 2.0, y: 0.0, z: 1.5, time: 15.0}  # Transit to survey area
  - {x: 2.0, y: 2.0, z: 1.5, time: 15.0}  # Survey pattern
  - {x: 0.0, y: 0.0, z: 0.5, time: 20.0}  # Return and descend
```

**Same field definitions as CSV.**

**Usage:**
```bash
roslaunch smc_quadrotor_cpp trajectory_from_file.launch trajectory_file:=/path/to/mission.yaml
```

### Generate Trajectories Programmatically

**Python example - Circular trajectory:**
```python
#!/usr/bin/env python3
import numpy as np
import csv

radius = 2.0
altitude = 1.5
n_points = 20

with open('circle.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['x', 'y', 'z', 'time'])
    
    # Takeoff
    writer.writerow([0.0, 0.0, 0.0, 5.0])
    writer.writerow([0.0, 0.0, altitude, 10.0])
    
    # Circle waypoints
    for i in range(n_points):
        theta = 2 * np.pi * i / n_points
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        writer.writerow([x, y, altitude, 8.0])
    
    # Landing
    writer.writerow([0.0, 0.0, 0.5, 15.0])

print(f"Generated circle.csv with {n_points+3} waypoints")
```

---

## ROS Parameters

For simple trajectories (typically <10 waypoints), use ROS parameters directly in launch files:

```xml
<launch>
    <include file="$(find rotors_gazebo)/launch/crazyflie2_without_controller.launch"/>
    
    <node name="quadrotor_control" pkg="smc_quadrotor_cpp" type="quadrotor_control_node" output="screen">
        <!-- Define trajectory -->
        <rosparam param="trajectory/waypoints_x">[0.0, 0.0, 1.5, 2.0, 1.5, 0.0]</rosparam>
        <rosparam param="trajectory/waypoints_y">[0.0, 0.5, 1.0, 0.5, 0.0, 0.0]</rosparam>
        <rosparam param="trajectory/waypoints_z">[0.0, 1.5, 1.5, 1.5, 1.5, 1.0]</rosparam>
        <rosparam param="trajectory/time_segments">[5.0, 15.0, 15.0, 15.0, 15.0]</rosparam>
    </node>
</launch>
```

**Format:**
- Three arrays (x, y, z coordinates) - **must have same length**
- One time array - **length must equal waypoints - 1**

---

## Programmatic API

For dynamic trajectory generation or integration with path planners:

### Option 1: Load from File
```cpp
#include <smc_quadrotor/trajectory_generator.h>

auto traj_gen = std::make_shared<TrajectoryGenerator>(false);
if (!traj_gen->loadFromFile("/path/to/trajectory.csv")) {
    ROS_ERROR("Failed to load trajectory file");
}
```

### Option 2: Set Directly
```cpp
#include <smc_quadrotor/trajectory_generator.h>

// Create generator without default trajectory
auto traj_gen = std::make_shared<TrajectoryGenerator>(false);

// Define waypoints
std::vector<Eigen::Vector3d> waypoints = {
    Eigen::Vector3d(0.0, 0.0, 0.0),    // Ground start
    Eigen::Vector3d(0.0, 0.0, 1.5),    // Takeoff
    Eigen::Vector3d(2.0, 0.0, 1.5),    // Move forward
    Eigen::Vector3d(2.0, 2.0, 1.5),    // Move right
    Eigen::Vector3d(0.0, 0.0, 1.0)     // Return and descend
};

// Define time segments (one less than waypoints)
std::vector<double> time_segments = {5.0, 10.0, 15.0, 20.0};

// Set trajectory with automatic validation
if (!traj_gen->setTrajectory(waypoints, time_segments)) {
    ROS_ERROR("Invalid trajectory parameters");
}
```

---

## Example Trajectories

### Included Examples

| File | Format | Waypoints | Duration | Description |
|------|--------|-----------|----------|-------------|
| `square_simple.csv` | CSV | 5 | 65s | Basic square pattern for testing |
| `spiral.csv` | CSV | 20 | 175s | 3D spiral with ascending/descending motion |
| `lawnmower.yaml` | YAML | 16 | 160s | Area coverage pattern for surveying |

### Additional Patterns (ROS Parameters)

**Vertical Line:**
```xml
<rosparam param="trajectory/waypoints_x">[0.0, 0.0, 0.0]</rosparam>
<rosparam param="trajectory/waypoints_y">[0.0, 0.0, 0.0]</rosparam>
<rosparam param="trajectory/waypoints_z">[0.0, 2.0, 0.0]</rosparam>
<rosparam param="trajectory/time_segments">[10.0, 10.0]</rosparam>
```

**Triangle:**
```xml
<rosparam param="trajectory/waypoints_x">[0.0, 0.0, 1.0, 0.5, 0.0]</rosparam>
<rosparam param="trajectory/waypoints_y">[0.0, 0.0, 0.0, 0.866, 0.0]</rosparam>
<rosparam param="trajectory/waypoints_z">[0.0, 1.5, 1.5, 1.5, 1.5]</rosparam>
<rosparam param="trajectory/time_segments">[5.0, 8.0, 8.0, 8.0]</rosparam>
```

**Custom Trajectory Example:**
```bash
roslaunch smc_quadrotor_cpp custom_trajectory_example.launch
```

---

## Validation Rules

All trajectories are validated before execution:

### Valid Trajectory Requirements
- **Minimum 2 waypoints** (start and end)
- **Array lengths match**: `waypoints_x.size() == waypoints_y.size() == waypoints_z.size()`
- **Correct time segments**: `time_segments.size() == waypoints.size() - 1`
- **Positive times**: All time values > 0
- **File format**: `.csv` or `.yaml`/`.yml` extensions only

### ❌ Common Validation Errors

| Error Message | Cause | Solution |
|---------------|-------|----------|
| "Less than 2 waypoints" | Not enough waypoints | Add more waypoints (minimum 2) |
| "Invalid waypoint dimensions" | Mismatched array lengths | Ensure x, y, z arrays have same length |
| "time_segments.size() != waypoints.size()-1" | Wrong number of time values | Fix time array size |
| "All time segments must be positive" | Zero or negative time | Use positive time values only |
| "Could not open file" | File doesn't exist | Check file path |
| "Unsupported file format" | Wrong extension | Use .csv or .yaml |

### Controller Behavior on Invalid Trajectory
If validation fails, the controller:
1. Logs detailed error message
2. Falls back to default trajectory
3. Continues operation (does not crash)

---

## Tips and Best Practices

### Trajectory Design

1. **Start at origin (0,0,0)** - Matches Gazebo spawn position
2. **Allow adequate takeoff time** - First segment should be 5-10 seconds
3. **Keep velocities reasonable** - Distance/time < 0.5 m/s for Crazyflie 2.0
4. **Avoid sharp turns** - Add intermediate waypoints for smooth curves
5. **Stay within workspace** - Recommended: ±5m in x,y and 0-3m in z
6. **End at stable hover** - Final waypoint should allow 10+ seconds for settling

### Performance Guidelines (Crazyflie 2.0)

| Parameter | Conservative | Moderate | Aggressive | Notes |
|-----------|-------------|----------|------------|-------|
| Max velocity | 0.3 m/s | 0.5 m/s | 0.8 m/s | Higher → tracking errors |
| Max acceleration | 0.2 m/s² | 0.4 m/s² | 0.6 m/s² | Limited by motor saturation |
| Min segment time | 10s | 5s | 3s | Longer → better tracking |
| Turn radius | 1.0m | 0.5m | 0.3m | Smaller → needs slower speed |

### Implementation Details

The trajectory generator uses **quintic (5th order) polynomials** with these boundary conditions:
- **Position**: Matches exactly at waypoints
- **Velocity**: Zero at each waypoint (smooth stops)
- **Acceleration**: Zero at each waypoint (jerk-free motion)

This ensures smooth motion without sudden accelerations that could destabilize the quadrotor.

### File Format Best Practices

**CSV:**
- Use for script-generated trajectories
- Easy to parse with Python, MATLAB, etc.
- Good for large datasets (100+ waypoints)

**YAML:**
- Use for hand-crafted missions
- Better for version control (readable diffs)
- Easy to add comments and documentation
- Good for complex missions with different phases

### Testing New Trajectories

**Always visualize before flying:**
```bash
./visualize_trajectory.py \
    --waypoints_x "0.0,0.0,1.5,2.0,1.5,0.0" \
    --waypoints_y "0.0,0.5,1.0,0.5,0.0,0.0" \
    --waypoints_z "0.0,1.5,1.5,1.5,1.5,1.0" \
    --time_segments "5.0,15.0,15.0,15.0,15.0" \
    --output preview.png
```

**Check key metrics:**
- Total distance and time
- Maximum velocity and acceleration
- Sharp turns or direction changes
- Altitude changes vs horizontal motion

---

## Troubleshooting

### Controller Issues

**Problem: Controller falls back to default trajectory**

Possible causes and solutions:
- Check parameter names: `trajectory/waypoints_x` not `trajectories/...`
- Verify all arrays have same length
- Ensure time segments count = waypoints - 1
- Check all time values are positive
- Look for validation error messages in console

**Problem: Quadrotor oscillates or becomes unstable**

Solutions:
- Reduce speeds (increase time segments)
- Add intermediate waypoints for smooth turns
- Check for extreme altitude changes
- Verify workspace limits aren't exceeded
- Lower controller gains if oscillations persist

**Problem: Tracking errors are large**

Solutions:
- Increase time segments (give more time per segment)
- Simplify trajectory (remove sharp turns)
- Check for realistic velocity/acceleration requirements
- Verify controller gains are properly tuned

### File Loading Issues

**Problem: File not found**

```bash
# Use absolute path
trajectory_file:=/home/user/trajectories/my_trajectory.csv

# Or use ROS find
trajectory_file:='$(find smc_quadrotor_cpp)/trajectories/my_trajectory.csv'
```

**Problem: Parse errors in CSV**

- Check for exactly 4 comma-separated values per line
- Verify no extra commas or spaces
- Ensure header line (if present) starts with letter
- Check for blank lines (they're ignored, but verify)

**Problem: Parse errors in YAML**

- Verify proper YAML syntax (indentation matters!)
- Check for matching braces: `{x: ..., y: ..., z: ..., time: ...}`
- Ensure `waypoints:` line exists
- Each waypoint must start with `- {`

### Performance Issues

**Problem: Mission takes too long**

- Review time segments, reduce if possible
- Check for unnecessarily slow segments
- Verify times make sense for distances

**Problem: Mission is too aggressive**

- Calculate velocity for each segment: `distance / time`
- Target: <0.5 m/s for reliable tracking
- Increase time segments proportionally
- Consider splitting long segments with intermediate waypoints

---

## Quick Reference

### Load Priority
```
trajectory_file parameter  →  ROS parameters  →  Default
    (highest priority)                            (fallback)
```

### File Extensions
- `.csv` - CSV format
- `.yaml` or `.yml` - YAML format

### Required Parameters (ROS method)
- `trajectory/waypoints_x` - Array of X coordinates
- `trajectory/waypoints_y` - Array of Y coordinates  
- `trajectory/waypoints_z` - Array of Z coordinates
- `trajectory/time_segments` - Array of segment durations

### Required Parameters (File method)
- `trajectory_file` - Path to CSV or YAML file

### Validation Requirements
- ≥2 waypoints
- Equal length coordinate arrays
- time_segments.size() = waypoints.size() - 1
- All times > 0

---

## See Also

- **README.md** - Main package documentation
- **visualize_trajectory.py --help** - Visualization tool usage
- **launch/trajectory_from_file.launch** - File loading example
- **launch/custom_trajectory_example.launch** - Custom trajectory example
- **trajectories/** - Example trajectory files
