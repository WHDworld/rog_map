#!/usr/bin/env python3
"""
轨迹日志可视化分析脚本
分析规划轨迹的平滑性、跟踪精度和跳变情况
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec
import pandas as pd
import sys
import os

def load_trajectory_log(log_file):
    """Load trajectory log CSV file"""
    if not os.path.exists(log_file):
        print(f"Error: Log file not found: {log_file}")
        sys.exit(1)
    
    df = pd.read_csv(log_file)
    print(f"✓ Logs Loaded: {len(df)} records")
    print(f"  Time Range: {df['timestamp'].min():.2f}s - {df['timestamp'].max():.2f}s")
    print(f"  Duration: {df['timestamp'].max() - df['timestamp'].min():.2f}s")
    return df

def analyze_smoothness(data, label):
    """Analyze trajectory smoothness"""
    if len(data) < 2:
        return {"max_jump": 0, "mean_diff": 0, "std_diff": 0}
    
    diff = np.diff(data)
    return {
        "max_jump": np.max(np.abs(diff)),
        "mean_diff": np.mean(np.abs(diff)),
        "std_diff": np.std(diff)
    }

def analyze_tracking_error(df):
    """Analyze tracking error"""
    pos_error = np.sqrt((df['cmd_x'] - df['odom_x'])**2 + 
                        (df['cmd_y'] - df['odom_y'])**2 + 
                        (df['cmd_z'] - df['odom_z'])**2)
    
    vel_error = np.sqrt((df['cmd_vx'] - df['odom_vx'])**2 + 
                        (df['cmd_vy'] - df['odom_vy'])**2 + 
                        (df['cmd_vz'] - df['odom_vz'])**2)
    
    yaw_error = np.abs(df['cmd_yaw'] - df['odom_yaw'])
    # Normalize to [-π, π]
    yaw_error = np.minimum(yaw_error, 2*np.pi - yaw_error)
    
    print("\n=== Tracking Error Statistics ===")
    print(f"Pos Error: Mean {pos_error.mean():.4f}m, Max {pos_error.max():.4f}m")
    print(f"Vel Error: Mean {vel_error.mean():.4f}m/s, Max {vel_error.max():.4f}m/s")
    print(f"Yaw Error: Mean {np.degrees(yaw_error.mean()):.2f}deg, Max {np.degrees(yaw_error.max()):.2f}deg")
    
    return pos_error, vel_error, yaw_error

def plot_trajectory_analysis(df, save_path=None):
    """Plot comprehensive analysis charts"""
    fig = plt.figure(figsize=(20, 12))
    gs = GridSpec(4, 3, figure=fig, hspace=0.3, wspace=0.3)
    
    t = df['timestamp'].values - df['timestamp'].values[0]
    
    # === Row 1: Position XYZ ===
    for i, axis in enumerate(['x', 'y', 'z']):
        ax = fig.add_subplot(gs[0, i])
        ax.plot(t, df[f'cmd_{axis}'], 'b-', label='Command', linewidth=1.5)
        ax.plot(t, df[f'odom_{axis}'], 'r--', label='Odom', linewidth=1, alpha=0.7)
        ax.plot(t, df[f'traj_end_{axis}'], 'g:', label='Traj End', linewidth=1)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(f'{axis.upper()} (m)')
        ax.set_title(f'Position - {axis.upper()} Axis')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        
        # Analyze smoothness
        smooth = analyze_smoothness(df[f'cmd_{axis}'].values, f'{axis}')
        ax.text(0.02, 0.98, f"Max Jump: {smooth['max_jump']:.4f}m", 
                transform=ax.transAxes, fontsize=8, va='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # === Row 2: Velocity XYZ ===
    for i, axis in enumerate(['vx', 'vy', 'vz']):
        ax = fig.add_subplot(gs[1, i])
        ax.plot(t, df[f'cmd_{axis}'], 'b-', label='Command', linewidth=1.5)
        ax.plot(t, df[f'odom_{axis}'], 'r--', label='Odom', linewidth=1, alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(f'V{axis[-1].upper()} (m/s)')
        ax.set_title(f'Velocity - {axis[-1].upper()} Axis')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        
        smooth = analyze_smoothness(df[f'cmd_{axis}'].values, f'v{axis[-1]}')
        ax.text(0.02, 0.98, f"Max Jump: {smooth['max_jump']:.4f}m/s", 
                transform=ax.transAxes, fontsize=8, va='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # === Row 3: Yaw + 3D Trajectory ===
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(t, np.degrees(df['cmd_yaw']), 'b-', label='Cmd Yaw', linewidth=1.5)
    ax.plot(t, np.degrees(df['odom_yaw']), 'r--', label='Odom Yaw', linewidth=1, alpha=0.7)
    
    # [New] Plot Plan Target Yaw
    if 'plan_target_yaw' in df.columns:
        ax.plot(t, np.degrees(df['plan_target_yaw']), 'g:', label='Target Yaw', linewidth=1.5)
        
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw (deg)')
    ax.set_title('Yaw Angle')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # [New] Visualize Replan Events (vertical lines)
    if 'plan_time' in df.columns:
        # Detect changes in plan_time
        plan_changes = np.where(np.diff(df['plan_time'].fillna(0)) != 0)[0]
        for idx in plan_changes:
            ax.axvline(t[idx], color='k', linestyle='--', alpha=0.2, linewidth=0.5)
    
    smooth = analyze_smoothness(df['cmd_yaw'].values, 'yaw')
    ax.text(0.02, 0.98, f"Max Jump: {np.degrees(smooth['max_jump']):.2f}deg", 
            transform=ax.transAxes, fontsize=8, va='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 3D Trajectory Plot
    ax = fig.add_subplot(gs[2, 1:], projection='3d')
    ax.plot(df['cmd_x'], df['cmd_y'], df['cmd_z'], 'b-', label='Cmd Traj', linewidth=1.5)
    ax.plot(df['odom_x'], df['odom_y'], df['odom_z'], 'r--', label='Odom Traj', linewidth=1, alpha=0.7)
    ax.scatter(df['traj_end_x'].iloc[0], df['traj_end_y'].iloc[0], df['traj_end_z'].iloc[0], 
               c='g', marker='o', s=100, label='Start')
    ax.scatter(df['traj_end_x'].iloc[-1], df['traj_end_y'].iloc[-1], df['traj_end_z'].iloc[-1], 
               c='r', marker='s', s=100, label='End')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectory')
    ax.legend(fontsize=8)
    
    # === Row 4: Tracking Error Analysis ===
    pos_err, vel_err, yaw_err = analyze_tracking_error(df)
    
    ax = fig.add_subplot(gs[3, 0])
    ax.plot(t, pos_err, 'r-', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Pos Error (m)')
    ax.set_title('Position Tracking Error')
    ax.grid(True, alpha=0.3)
    ax.axhline(pos_err.mean(), color='b', linestyle='--', label=f'Mean: {pos_err.mean():.4f}m')
    ax.legend(fontsize=8)
    
    ax = fig.add_subplot(gs[3, 1])
    ax.plot(t, vel_err, 'g-', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Vel Error (m/s)')
    ax.set_title('Velocity Tracking Error')
    ax.grid(True, alpha=0.3)
    ax.axhline(vel_err.mean(), color='b', linestyle='--', label=f'Mean: {vel_err.mean():.4f}m/s')
    ax.legend(fontsize=8)
    
    ax = fig.add_subplot(gs[3, 2])
    ax.plot(t, np.degrees(yaw_err), 'm-', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw Error (deg)')
    ax.set_title('Yaw Tracking Error')
    ax.grid(True, alpha=0.3)
    ax.axhline(np.degrees(yaw_err.mean()), color='b', linestyle='--', 
               label=f'Mean: {np.degrees(yaw_err.mean()):.2f}deg')
    ax.legend(fontsize=8)
    
    plt.suptitle('Trajectory Smoothness & Tracking Accuracy Analysis', fontsize=16, fontweight='bold')
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n✓ Chart saved: {save_path}")
    
    plt.show()

def main():
    # Auto-detect log directory (script directory)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = script_dir  # script is in log dir
    log_file = os.path.join(log_dir, "trajectory_log.csv")
    
    print("=" * 60)
    print("Trajectory Log Visualization Tool")
    print("=" * 60)
    print(f"Log Directory: {log_dir}")
    
    df = load_trajectory_log(log_file)
    
    # Generate visualization report
    save_path = os.path.join(log_dir, "trajectory_analysis.png")
    plot_trajectory_analysis(df, save_path)
    
    print("\nAnalysis Complete!")

if __name__ == "__main__":
    main()
