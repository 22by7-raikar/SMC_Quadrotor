#!/usr/bin/env python3
"""
Post-process and visualize trajectory comparison AFTER Gazebo simulation.
Reads logged trajectory data and generates comparison plots.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

class TrajectoryComparison:
    def __init__(self):
        # Expected trajectory parameters (default square)
        self.waypoints_x = [0.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        self.waypoints_y = [0.0, 0.0, 0.0, 1.0, 1.0, 0.0]
        self.waypoints_z = [0.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        self.time_segments = [5.0, 15.0, 15.0, 15.0, 15.0]
        
        # Generate expected trajectory
        self.expected_time, self.expected_x, self.expected_y, self.expected_z = self.generate_expected_trajectory()
        
        # Load actual trajectory from log file
        self.actual_time = []
        self.actual_x = []
        self.actual_y = []
        self.actual_z = []
        self.load_actual_trajectory()
    
    def quintic_coeffs(self, p0, pf, t0, tf):
        """Calculate quintic polynomial coefficients."""
        T = tf - t0
        a0 = p0
        a1 = 0
        a2 = 0
        a3 = 10 * (pf - p0) / T**3
        a4 = -15 * (pf - p0) / T**4
        a5 = 6 * (pf - p0) / T**5
        return [a0, a1, a2, a3, a4, a5]
    
    def eval_quintic(self, coeffs, t):
        """Evaluate quintic polynomial at time t."""
        a0, a1, a2, a3, a4, a5 = coeffs
        tau = t  # relative time
        return a0 + a1*tau + a2*tau**2 + a3*tau**3 + a4*tau**4 + a5*tau**5
    
    def generate_expected_trajectory(self):
        """Generate expected trajectory using quintic polynomials."""
        waypoints = list(zip(self.waypoints_x, self.waypoints_y, self.waypoints_z))
        total_time = sum(self.time_segments)
        
        time_array = np.linspace(0, total_time, int(total_time * 100))
        x_traj, y_traj, z_traj = [], [], []
        
        cumulative_time = 0
        for seg_idx, (p0, pf) in enumerate(zip(waypoints[:-1], waypoints[1:])):
            T = self.time_segments[seg_idx]
            t0 = cumulative_time
            tf = cumulative_time + T
            
            coeffs_x = self.quintic_coeffs(p0[0], pf[0], t0, tf)
            coeffs_y = self.quintic_coeffs(p0[1], pf[1], t0, tf)
            coeffs_z = self.quintic_coeffs(p0[2], pf[2], t0, tf)
            
            for t in time_array:
                if t0 <= t <= tf:
                    tau = t - t0
                    x_traj.append(self.eval_quintic(coeffs_x, tau))
                    y_traj.append(self.eval_quintic(coeffs_y, tau))
                    z_traj.append(self.eval_quintic(coeffs_z, tau))
            
            cumulative_time = tf
        
        return time_array[:len(x_traj)], x_traj, y_traj, z_traj
    
    def load_actual_trajectory(self):
        """Load actual trajectory from log file."""
        log_file = 'log_cpp.txt'
        
        if not os.path.exists(log_file):
            print(f"ERROR: Log file '{log_file}' not found!")
            print("Make sure Gazebo simulation has completed and data was logged.")
            sys.exit(1)
        
        try:
            with open(log_file, 'r') as f:
                for line in f:
                    if line.startswith('#'):
                        continue
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        self.actual_time.append(float(parts[0]))
                        self.actual_x.append(float(parts[1]))
                        self.actual_y.append(float(parts[2]))
                        self.actual_z.append(float(parts[3]))
            
            if not self.actual_time:
                print("ERROR: No data found in log file!")
                sys.exit(1)
            
            print(f"Loaded {len(self.actual_time)} samples from {log_file}")
            print(f"Trajectory duration: {self.actual_time[-1]:.2f} seconds")
        except Exception as e:
            print(f"ERROR reading log file: {e}")
            sys.exit(1)
    
    def calculate_errors(self):
        """Calculate tracking errors."""
        if not self.actual_time:
            print("No actual trajectory data!")
            return None, None, None
        
        # Interpolate expected trajectory to actual time points
        expected_x_interp = np.interp(self.actual_time, self.expected_time, self.expected_x)
        expected_y_interp = np.interp(self.actual_time, self.expected_time, self.expected_y)
        expected_z_interp = np.interp(self.actual_time, self.expected_time, self.expected_z)
        
        # Calculate errors
        error_x = np.array(self.actual_x) - expected_x_interp
        error_y = np.array(self.actual_y) - expected_y_interp
        error_z = np.array(self.actual_z) - expected_z_interp
        
        euclidean_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        return euclidean_error, (error_x, error_y, error_z), (expected_x_interp, expected_y_interp, expected_z_interp)
    
    def plot_comparison(self):
        """Plot expected vs actual trajectory."""
        euclidean_error, component_errors, expected_interp = self.calculate_errors()
        error_x, error_y, error_z = component_errors
        expected_x_interp, expected_y_interp, expected_z_interp = expected_interp
        
        fig = plt.figure(figsize=(16, 12))
        
        # 3D trajectory comparison
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        ax1.plot(self.expected_x, self.expected_y, self.expected_z, 'b-', linewidth=2.5, label='Expected', alpha=0.8)
        ax1.plot(self.actual_x, self.actual_y, self.actual_z, 'r--', linewidth=1.5, label='Actual', alpha=0.8)
        ax1.scatter(self.waypoints_x, self.waypoints_y, self.waypoints_z, c='green', s=150, marker='o', 
                   label='Waypoints', edgecolors='darkgreen', linewidth=2)
        ax1.set_xlabel('X (m)', fontsize=10)
        ax1.set_ylabel('Y (m)', fontsize=10)
        ax1.set_zlabel('Z (m)', fontsize=10)
        ax1.set_title('3D Trajectory Comparison', fontsize=12, fontweight='bold')
        ax1.legend(fontsize=9)
        ax1.grid(True, alpha=0.3)
        ax1.view_init(elev=20, azim=45)
        
        # X position vs time
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(self.expected_time, self.expected_x, 'b-', linewidth=2.5, label='Expected', alpha=0.8)
        ax2.plot(self.actual_time, self.actual_x, 'r--', linewidth=1.5, label='Actual', alpha=0.8)
        ax2.fill_between(self.actual_time, self.actual_x, expected_x_interp, alpha=0.2, color='orange')
        ax2.set_xlabel('Time (s)', fontsize=10)
        ax2.set_ylabel('X Position (m)', fontsize=10)
        ax2.set_title('X Position Tracking', fontsize=12, fontweight='bold')
        ax2.legend(fontsize=9)
        ax2.grid(True, alpha=0.3)
        
        # Y position vs time
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.plot(self.expected_time, self.expected_y, 'b-', linewidth=2.5, label='Expected', alpha=0.8)
        ax3.plot(self.actual_time, self.actual_y, 'r--', linewidth=1.5, label='Actual', alpha=0.8)
        ax3.fill_between(self.actual_time, self.actual_y, expected_y_interp, alpha=0.2, color='orange')
        ax3.set_xlabel('Time (s)', fontsize=10)
        ax3.set_ylabel('Y Position (m)', fontsize=10)
        ax3.set_title('Y Position Tracking', fontsize=12, fontweight='bold')
        ax3.legend(fontsize=9)
        ax3.grid(True, alpha=0.3)
        
        # Z position vs time
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(self.expected_time, self.expected_z, 'b-', linewidth=2.5, label='Expected', alpha=0.8)
        ax4.plot(self.actual_time, self.actual_z, 'r--', linewidth=1.5, label='Actual', alpha=0.8)
        ax4.fill_between(self.actual_time, self.actual_z, expected_z_interp, alpha=0.2, color='orange')
        ax4.set_xlabel('Time (s)', fontsize=10)
        ax4.set_ylabel('Z Position (m)', fontsize=10)
        ax4.set_title('Z Position Tracking', fontsize=12, fontweight='bold')
        ax4.legend(fontsize=9)
        ax4.grid(True, alpha=0.3)
        
        # Euclidean error vs time
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.plot(self.actual_time, euclidean_error, 'purple', linewidth=2)
        ax5.fill_between(self.actual_time, euclidean_error, alpha=0.3, color='purple')
        mean_error = np.mean(euclidean_error)
        max_error = np.max(euclidean_error)
        ax5.axhline(y=mean_error, color='k', linestyle='--', linewidth=1.5, label=f'Mean: {mean_error:.4f}m')
        ax5.axhline(y=max_error, color='r', linestyle=':', linewidth=1.5, label=f'Max: {max_error:.4f}m')
        ax5.set_xlabel('Time (s)', fontsize=10)
        ax5.set_ylabel('Euclidean Error (m)', fontsize=10)
        ax5.set_title('Tracking Error', fontsize=12, fontweight='bold')
        ax5.legend(fontsize=9)
        ax5.grid(True, alpha=0.3)
        
        # Component errors vs time
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.plot(self.actual_time, error_x, 'r-', linewidth=1.5, label='X Error', alpha=0.8)
        ax6.plot(self.actual_time, error_y, 'g-', linewidth=1.5, label='Y Error', alpha=0.8)
        ax6.plot(self.actual_time, error_z, 'b-', linewidth=1.5, label='Z Error', alpha=0.8)
        ax6.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
        ax6.set_xlabel('Time (s)', fontsize=10)
        ax6.set_ylabel('Error (m)', fontsize=10)
        ax6.set_title('Component-wise Errors', fontsize=12, fontweight='bold')
        ax6.legend(fontsize=9)
        ax6.grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_file = 'trajectory_comparison.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Comparison plot saved to: {output_file}")
        
        # Print statistics
        print("\n" + "="*70)
        print("TRAJECTORY TRACKING PERFORMANCE STATISTICS")
        print("="*70)
        print(f"Trajectory Duration: {self.actual_time[-1]:.2f} seconds")
        print(f"Samples Collected: {len(self.actual_time)}")
        print(f"Expected Duration: {self.expected_time[-1]:.2f} seconds")
        print(f"\nTracking Error Statistics:")
        print(f"  Mean Error: {mean_error:.6f} m ({mean_error*100:.3f} cm)")
        print(f"  Max Error:  {max_error:.6f} m ({max_error*100:.3f} cm)")
        print(f"  Min Error:  {np.min(euclidean_error):.6f} m ({np.min(euclidean_error)*1000:.3f} mm)")
        print(f"  Std Dev:    {np.std(euclidean_error):.6f} m ({np.std(euclidean_error)*100:.3f} cm)")
        print(f"\nComponent Errors (Mean Absolute):")
        print(f"  X Error: {np.mean(np.abs(error_x)):.6f} m ({np.mean(np.abs(error_x))*100:.3f} cm)")
        print(f"  Y Error: {np.mean(np.abs(error_y)):.6f} m ({np.mean(np.abs(error_y))*100:.3f} cm)")
        print(f"  Z Error: {np.mean(np.abs(error_z)):.6f} m ({np.mean(np.abs(error_z))*100:.3f} cm)")
        print(f"\nComponent Errors (Max Absolute):")
        print(f"  X Error: {np.max(np.abs(error_x)):.6f} m ({np.max(np.abs(error_x))*100:.3f} cm)")
        print(f"  Y Error: {np.max(np.abs(error_y)):.6f} m ({np.max(np.abs(error_y))*100:.3f} cm)")
        print(f"  Z Error: {np.max(np.abs(error_z)):.6f} m ({np.max(np.abs(error_z))*100:.3f} cm)")
        print("="*70)

def main():
    print("="*70)
    print("TRAJECTORY COMPARISON - POST-PROCESSING")
    print("="*70)
    print("This script reads logged trajectory data and generates")
    print("comparison plots between expected and actual trajectories.\n")
    
    comparison = TrajectoryComparison()
    comparison.plot_comparison()
    
    print("\nDone! View the generated plot: trajectory_comparison.png")
    print("="*70)

if __name__ == '__main__':
    main()
