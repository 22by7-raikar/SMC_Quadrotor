#!/usr/bin/env python3
"""
Trajectory Visualization and Analysis Tool
Analyzes quintic polynomial trajectories for the SMC Quadrotor controller
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

def quintic_coeffs(p0, p1, T):
    """
    Compute quintic polynomial coefficients for a trajectory segment.
    
    Args:
        p0: Start position
        p1: End position
        T: Time duration
        
    Returns:
        Coefficients [a0, a1, a2, a3, a4, a5] for polynomial:
        s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    """
    A = np.array([
        [1, 0,   0,      0,        0,         0],
        [0, 1,   0,      0,        0,         0],
        [0, 0,   2,      0,        0,         0],
        [1, T,   T**2,   T**3,     T**4,      T**5],
        [0, 1,   2*T,    3*T**2,   4*T**3,    5*T**4],
        [0, 0,   2,      6*T,      12*T**2,   20*T**3]
    ])
    b = np.array([p0, 0, 0, p1, 0, 0])
    return np.linalg.solve(A, b)

def generate_trajectory(waypoints_x, waypoints_y, waypoints_z, time_segments):
    """
    Generate complete trajectory from waypoints and time segments.
    
    Args:
        waypoints_x, waypoints_y, waypoints_z: Lists of waypoint coordinates
        time_segments: List of time durations for each segment
        
    Returns:
        time_points, x_traj, y_traj, z_traj: Arrays of trajectory points
    """
    time_points = []
    x_traj, y_traj, z_traj = [], [], []
    t_current = 0
    
    for i in range(len(time_segments)):
        p0 = np.array([waypoints_x[i], waypoints_y[i], waypoints_z[i]])
        p1 = np.array([waypoints_x[i+1], waypoints_y[i+1], waypoints_z[i+1]])
        T = time_segments[i]
        
        # Compute coefficients for each axis
        coeff_x = quintic_coeffs(p0[0], p1[0], T)
        coeff_y = quintic_coeffs(p0[1], p1[1], T)
        coeff_z = quintic_coeffs(p0[2], p1[2], T)
        
        # Sample the segment
        t_local = np.linspace(0, T, int(T * 10))
        for t in t_local:
            time_points.append(t_current + t)
            x_traj.append(np.polyval(coeff_x[::-1], t))
            y_traj.append(np.polyval(coeff_y[::-1], t))
            z_traj.append(np.polyval(coeff_z[::-1], t))
        
        t_current += T
    
    return np.array(time_points), np.array(x_traj), np.array(y_traj), np.array(z_traj)

def visualize_trajectory(waypoints_x, waypoints_y, waypoints_z, time_segments, 
                        output_file='/tmp/trajectory_analysis.png'):
    """
    Create comprehensive trajectory visualization with 6 plots.
    
    Args:
        waypoints_x, waypoints_y, waypoints_z: Waypoint coordinates
        time_segments: Time durations for each segment
        output_file: Path to save the visualization
    """
    # Generate trajectory
    time_points, x_traj, y_traj, z_traj = generate_trajectory(
        waypoints_x, waypoints_y, waypoints_z, time_segments
    )
    
    # Create figure with 6 subplots
    fig = plt.figure(figsize=(16, 10))
    
    # 1. 3D trajectory
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(x_traj, y_traj, z_traj, 'b-', linewidth=2, label='Trajectory')
    ax1.scatter(waypoints_x, waypoints_y, waypoints_z, c='r', s=100, marker='o', label='Waypoints')
    for i, (x, y, z) in enumerate(zip(waypoints_x, waypoints_y, waypoints_z)):
        ax1.text(x, y, z, f'  W{i}', fontsize=9)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory', fontsize=12, fontweight='bold')
    ax1.legend()
    ax1.grid(True)
    
    # 2. Top view (XY plane)
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(x_traj, y_traj, 'b-', linewidth=2)
    ax2.scatter(waypoints_x, waypoints_y, c='r', s=100, marker='o', zorder=5)
    for i, (x, y) in enumerate(zip(waypoints_x, waypoints_y)):
        ax2.annotate(f'W{i}', (x, y), xytext=(5, 5), textcoords='offset points', fontsize=9)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (XY Plane)', fontsize=12, fontweight='bold')
    ax2.grid(True)
    ax2.axis('equal')
    
    # 3. Side view (XZ plane)
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(x_traj, z_traj, 'b-', linewidth=2)
    ax3.scatter(waypoints_x, waypoints_z, c='r', s=100, marker='o', zorder=5)
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('Side View (XZ Plane)', fontsize=12, fontweight='bold')
    ax3.grid(True)
    
    # 4. Position vs time
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(time_points, x_traj, 'r-', label='X', linewidth=2)
    ax4.plot(time_points, y_traj, 'g-', label='Y', linewidth=2)
    ax4.plot(time_points, z_traj, 'b-', label='Z', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Position (m)')
    ax4.set_title('Position vs Time', fontsize=12, fontweight='bold')
    ax4.legend()
    ax4.grid(True)
    
    # 5. Velocity profile
    dt = time_points[1] - time_points[0]
    vx = np.gradient(x_traj, dt)
    vy = np.gradient(y_traj, dt)
    vz = np.gradient(z_traj, dt)
    v_mag = np.sqrt(vx**2 + vy**2 + vz**2)
    
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(time_points, vx, 'r-', label='Vx', alpha=0.7)
    ax5.plot(time_points, vy, 'g-', label='Vy', alpha=0.7)
    ax5.plot(time_points, vz, 'b-', label='Vz', alpha=0.7)
    ax5.plot(time_points, v_mag, 'k-', label='|V|', linewidth=2)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Velocity (m/s)')
    ax5.set_title('Velocity Profile', fontsize=12, fontweight='bold')
    ax5.legend()
    ax5.grid(True)
    
    # 6. Acceleration profile
    ax = np.gradient(vx, dt)
    ay = np.gradient(vy, dt)
    az = np.gradient(vz, dt)
    a_mag = np.sqrt(ax**2 + ay**2 + az**2)
    
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(time_points, ax, 'r-', label='Ax', alpha=0.7)
    ax6.plot(time_points, ay, 'g-', label='Ay', alpha=0.7)
    ax6.plot(time_points, az, 'b-', label='Az', alpha=0.7)
    ax6.plot(time_points, a_mag, 'k-', label='|A|', linewidth=2)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Acceleration (m/s²)')
    ax6.set_title('Acceleration Profile', fontsize=12, fontweight='bold')
    ax6.legend()
    ax6.grid(True)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    
    # Print statistics
    print("\n" + "="*80)
    print("TRAJECTORY ANALYSIS")
    print("="*80)
    
    # Basic statistics
    total_distance = np.sum(np.sqrt(np.diff(x_traj)**2 + np.diff(y_traj)**2 + np.diff(z_traj)**2))
    print(f"\nSTATISTICS:")
    print(f"   Total Duration: {time_points[-1]:.1f} seconds")
    print(f"   Total Distance: {total_distance:.2f} meters")
    print(f"   Number of Waypoints: {len(waypoints_x)}")
    print(f"   Number of Segments: {len(time_segments)}")
    
    print(f"\n🚀 VELOCITY:")
    print(f"   Max: {np.max(v_mag):.3f} m/s")
    print(f"   Average: {np.mean(v_mag):.3f} m/s")
    print(f"   Min: {np.min(v_mag):.3f} m/s")
    
    print(f"\n⚡ ACCELERATION:")
    print(f"   Max: {np.max(a_mag):.3f} m/s²")
    print(f"   Average: {np.mean(a_mag):.3f} m/s²")
    
    print(f"\n📍 WAYPOINTS:")
    for i in range(len(waypoints_x)):
        t_cumulative = sum(time_segments[:i]) if i > 0 else 0
        print(f"   W{i}: ({waypoints_x[i]:.1f}, {waypoints_y[i]:.1f}, {waypoints_z[i]:.1f}) at t={t_cumulative:.1f}s")
    
    print(f"\nSEGMENTS:")
    for i, T in enumerate(time_segments):
        dist = np.sqrt((waypoints_x[i+1]-waypoints_x[i])**2 + 
                       (waypoints_y[i+1]-waypoints_y[i])**2 + 
                       (waypoints_z[i+1]-waypoints_z[i])**2)
        avg_speed = dist / T if T > 0 else 0
        print(f"   Segment {i}: {T:.1f}s, dist={dist:.2f}m, speed={avg_speed:.3f}m/s")
    
    print(f"\nSaved to: {output_file}")
    print("="*80 + "\n")
    
    return fig

def main():
    """Main function with example trajectories."""
    
    if len(sys.argv) > 1 and sys.argv[1] == '--help':
        print("Trajectory Visualization Tool")
        print("\nUsage:")
        print("  python3 visualize_trajectory.py           # Visualize default trajectory")
        print("  python3 visualize_trajectory.py --help    # Show this help")
        print("\nTo customize, edit the waypoints and time_segments in the script.")
        return
    
    # Default square trajectory from trajectory_generator.h:
    # (0,0,0) → (0,0,1) → (1,0,1) → (1,1,1) → (0,1,1) → (0,0,1)
    waypoints_x = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0]
    waypoints_y = [0.0, 0.0, 0.0, 1.0, 1.0, 0.0]
    waypoints_z = [0.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    time_segments = [5.0, 15.0, 15.0, 15.0, 15.0]  # Total: 65 seconds
    
    visualize_trajectory(waypoints_x, waypoints_y, waypoints_z, time_segments)
    plt.show()

if __name__ == "__main__":
    main()
