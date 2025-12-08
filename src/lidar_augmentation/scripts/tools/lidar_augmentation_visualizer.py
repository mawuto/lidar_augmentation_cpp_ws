#!/usr/bin/env python3

import rospy
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import defaultdict, deque
from std_msgs.msg import String
from threading import Lock
import time
from datetime import datetime

class LidarAugmentationVisualizer:
    def __init__(self, buffer_size=200, update_interval=1000):
        """
        Real-time LiDAR Augmentation Visualization Dashboard
        
        Args:
            buffer_size (int): Number of data points to display
            update_interval (int): Update interval in milliseconds
        """
        rospy.init_node('lidar_augmentation_visualizer', anonymous=True)
        
        # Configuration
        self.buffer_size = buffer_size
        self.update_interval = update_interval
        self.data_lock = Lock()
        
        # Data storage for each sensor
        self.sensor_data = {}
        self.sensor_colors = {
            'auto_sensor_1': '#1f77b4',  # Blue for Livox Avia
            'auto_sensor_2': '#ff7f0e',  # Orange for Ouster  
            'auto_sensor_3': '#2ca02c',  # Green for Livox Mid360
        }
        
        # Statistics
        self.total_messages = 0
        self.start_time = time.time()
        
        #  WHITE BACKGROUND SETUP
        plt.style.use('default')
        plt.rcParams['figure.facecolor'] = 'white'
        plt.rcParams['axes.facecolor'] = 'white'
        plt.rcParams['savefig.facecolor'] = 'white'
        plt.rcParams['text.color'] = 'black'
        plt.rcParams['axes.labelcolor'] = 'black'
        plt.rcParams['axes.edgecolor'] = 'black'
        plt.rcParams['xtick.color'] = 'black'
        plt.rcParams['ytick.color'] = 'black'
        plt.rcParams['grid.color'] = 'gray'
        plt.rcParams['grid.alpha'] = 0.3
        
        self.setup_plots()
        
        # Subscribe to statistics
        self.stats_subscriber = rospy.Subscriber(
            '/lidar_augmentation/statistics',
            String,
            self.statistics_callback,
            queue_size=100
        )
        
        rospy.loginfo(" LiDAR Augmentation Visualizer initialized")
        rospy.loginfo(f" Buffer size: {buffer_size} points")
        rospy.loginfo(f" Update interval: {update_interval}ms")
        
    def setup_plots(self):
        """Setup the 4-plot dashboard"""
        self.fig = plt.figure(figsize=(16, 12))
        #  BLACK TEXT FOR WHITE BACKGROUND
        self.fig.suptitle(' LiDAR Augmentation Real-Time Dashboard', 
                         fontsize=16, fontweight='bold', color='black')
        
        # Create 2x2 subplot grid
        self.ax1 = plt.subplot(2, 2, 1)  # Point counts timeline
        self.ax2 = plt.subplot(2, 2, 2)  # Reduction percentage
        self.ax3 = plt.subplot(2, 2, 3)  # Processing time
        self.ax4 = plt.subplot(2, 2, 4)  # Statistics summary
        
        # Configure Plot 1: Point Counts Timeline
        self.ax1.set_title(' Point Counts Timeline', fontweight='bold', pad=20, color='black')
        self.ax1.set_xlabel('Time (relative seconds)', color='black')
        self.ax1.set_ylabel('Point Count', color='black')
        self.ax1.grid(True, alpha=0.3, color='gray')
        self.ax1.legend(loc='upper right')
        
        # Configure Plot 2: Reduction Percentage
        self.ax2.set_title(' Point Reduction Percentage', fontweight='bold', pad=20, color='black')
        self.ax2.set_xlabel('Time (relative seconds)', color='black')
        self.ax2.set_ylabel('Reduction (%)', color='black')
        self.ax2.grid(True, alpha=0.3, color='gray')
        self.ax2.legend(loc='upper right')
        
        # Configure Plot 3: Processing Time
        self.ax3.set_title(' Processing Time Comparison', fontweight='bold', pad=20, color='black')
        self.ax3.set_xlabel('Time (relative seconds)', color='black')
        self.ax3.set_ylabel('Processing Time (ms)', color='black')
        self.ax3.grid(True, alpha=0.3, color='gray')
        self.ax3.legend(loc='upper right')
        
        # Configure Plot 4: Statistics Summary (text-based)
        self.ax4.set_title(' Live Statistics Summary', fontweight='bold', pad=20, color='black')
        self.ax4.axis('off')  # Turn off axes for text display
        
        # Store line objects for each sensor
        self.lines = {
            'original_points': {},
            'augmented_points': {},
            'reduction_percent': {},
            'processing_time': {}
        }
        
        # Adjust layout
        plt.tight_layout()
        
    def statistics_callback(self, msg):
        """Process incoming statistics messages"""
        try:
            stats = json.loads(msg.data)
            
            # Extract data
            sensor_name = stats.get('sensor', 'unknown')
            sensor_type = stats.get('sensor_type', 'unknown')
            timestamp = float(stats.get('timestamp', 0))
            original_points = int(stats.get('original_points', 0))
            augmented_points = int(stats.get('augmented_points', 0))
            reduction_percent = float(stats.get('point_reduction_percent', 0))
            processing_time = int(stats.get('processing_time_ms', 0))
            active_scenario = stats.get('active_scenario', 'unknown')
            scenario_parameters = stats.get('scenario_parameters', {})
            
            # Calculate relative time (seconds since start)
            current_time = time.time()
            relative_time = current_time - self.start_time
            
            # Thread-safe data storage
            with self.data_lock:
                if sensor_name not in self.sensor_data:
                    self.sensor_data[sensor_name] = {
                        'sensor_type': sensor_type,
                        'relative_times': deque(maxlen=self.buffer_size),
                        'timestamps': deque(maxlen=self.buffer_size),
                        'original_points': deque(maxlen=self.buffer_size),
                        'augmented_points': deque(maxlen=self.buffer_size),
                        'reduction_percent': deque(maxlen=self.buffer_size),
                        'processing_time': deque(maxlen=self.buffer_size),
                        'message_count': 0,
                        'active_scenario': 'unknown',
                        'scenario_parameters': {}
                    }
                
                data = self.sensor_data[sensor_name]
                data['active_scenario'] = active_scenario
                data['scenario_parameters'] = scenario_parameters
                data['relative_times'].append(relative_time)
                data['timestamps'].append(timestamp)
                data['original_points'].append(original_points)
                data['augmented_points'].append(augmented_points)
                data['reduction_percent'].append(reduction_percent)
                data['processing_time'].append(processing_time)
                data['message_count'] += 1
                
                self.total_messages += 1
                
        except Exception as e:
            rospy.logwarn(f" Statistics processing error: {e}")
    
    def update_plots(self, frame):
        """Update all plots with latest data"""
        try:
            with self.data_lock:
                if not self.sensor_data:
                    return
                
                # Clear all axes
                self.ax1.clear()
                self.ax2.clear()
                self.ax3.clear()
                self.ax4.clear()
                
                # Reconfigure axes
                self.ax1.set_title(' Point Counts Timeline', fontweight='bold', pad=20, color='black')
                self.ax1.set_xlabel('Time (seconds)', color='black')
                self.ax1.set_ylabel('Point Count', color='black')
                self.ax1.grid(True, alpha=0.3, color='gray')
                
                self.ax2.set_title(' Point Reduction Percentage', fontweight='bold', pad=20, color='black')
                self.ax2.set_xlabel('Time (seconds)', color='black')
                self.ax2.set_ylabel('Reduction (%)', color='black')
                self.ax2.grid(True, alpha=0.3, color='gray')
                
                self.ax3.set_title(' Processing Time Comparison', fontweight='bold', pad=20, color='black')
                self.ax3.set_xlabel('Time (seconds)', color='black')
                self.ax3.set_ylabel('Processing Time (ms)', color='black')
                self.ax3.grid(True, alpha=0.3, color='gray')
                
                self.ax4.set_title(' Live Statistics Summary', fontweight='bold', pad=20, color='black')
                self.ax4.axis('off')
                
                # Plot data for each sensor
                summary_text = []
                for sensor_name, data in self.sensor_data.items():
                    if len(data['relative_times']) == 0:
                        continue
                    
                    # Get color for this sensor
                    color = self.sensor_colors.get(sensor_name, '#888888')
                    
                    # Convert deques to numpy arrays
                    times = np.array(data['relative_times'])
                    original_points = np.array(data['original_points'])
                    augmented_points = np.array(data['augmented_points'])
                    reduction_percent = np.array(data['reduction_percent'])
                    processing_time = np.array(data['processing_time'])
                    
                    # Plot 1: Point counts
                    label_orig = f"{sensor_name} Original ({data['sensor_type']})"
                    label_aug = f"{sensor_name} Augmented"
                    
                    self.ax1.plot(times, original_points, 
                                 color=color, linewidth=2, alpha=0.8, 
                                 label=label_orig, linestyle='-')
                    self.ax1.plot(times, augmented_points, 
                                 color=color, linewidth=2, alpha=0.6, 
                                 label=label_aug, linestyle='--')
                    
                    # Plot 2: Reduction percentage
                    self.ax2.plot(times, reduction_percent, 
                                 color=color, linewidth=2, alpha=0.8,
                                 label=f"{sensor_name} ({data['sensor_type']})")
                    
                    # Plot 3: Processing time
                    self.ax3.plot(times, processing_time, 
                                 color=color, linewidth=2, alpha=0.8,
                                 label=f"{sensor_name} ({data['sensor_type']})")
                    
                    # Prepare summary statistics
                    if len(original_points) > 0:
                        avg_original = np.mean(original_points)
                        avg_augmented = np.mean(augmented_points)
                        avg_reduction = np.mean(reduction_percent)
                        avg_processing = np.mean(processing_time)
                        scenario_info = f"\n  Scenario: {data.get('active_scenario', 'unknown').upper()}"
                        
                        # Add active parameters summary
                        params = data.get('scenario_parameters', {})
                        active_augs = []

                        if params.get('dropout', {}).get('enabled'): 
                            rate = params.get('dropout', {}).get('rate', 0)
                            active_augs.append(f"Dropout({rate:.1%})")

                        if params.get('noise', {}).get('enabled'): 
                            std = params.get('noise', {}).get('gaussian_std', 0)
                            active_augs.append(f"Noise({std:.3f})")

                        if params.get('fov_reduction', {}).get('enabled'): 
                            fov_h = params.get('fov_reduction', {}).get('horizontal', 0)
                            fov_v = params.get('fov_reduction', {}).get('vertical', 0)
                            active_augs.append(f"FOV_Reduction(H:{fov_h:.1%},V:{fov_v:.1%})")

                        if params.get('occlusion', {}).get('enabled'): 
                            dist = params.get('occlusion', {}).get('distance_threshold', 0)
                            num_obj = params.get('occlusion', {}).get('num_objects', 0)
                            obj_size = params.get('occlusion', {}).get('object_size', 0)
                            active_augs.append(f"Occlusion(≤{dist}m,{num_obj}obj@{obj_size}m)")

                        if params.get('sparse_scan', {}).get('enabled'): 
                            skip = params.get('sparse_scan', {}).get('skip_factor', 1)
                            reduction = (skip - 1) / skip * 100
                            active_augs.append(f"Sparse_Scan({reduction:.0f}%↓)")

                        if params.get('motion_distortion', {}).get('enabled'): 
                            lin_vel = params.get('motion_distortion', {}).get('linear_velocity', 0)
                            ang_vel = params.get('motion_distortion', {}).get('angular_velocity', 0)
                            active_augs.append(f"Motion_Distortion(v:{lin_vel:.1f}m/s,ω:{ang_vel:.1f}rad/s)")
                        
                        if active_augs:
                            aug_lines = []
                            for i in range(0, len(active_augs), 2):
                                line_augs = active_augs[i:i+2]
                                aug_lines.append(' • '.join(line_augs))
                            scenario_info += f"\n  Active:\n    " + "\n    ".join(aug_lines)
                        else:
                            scenario_info += f"\n  Active: None"
                        
                        summary_text.append(
                            f"   {sensor_name.upper()} ({data['sensor_type']})\n"
                            f"   Messages: {data['message_count']}\n"
                            f"   Avg Original: {avg_original:,.0f} points\n"
                            f"   Avg Augmented: {avg_augmented:,.0f} points\n"
                            f"   Avg Reduction: {avg_reduction:.1f}%\n"
                            f"   Avg Processing: {avg_processing:.1f}ms\n"
                            + scenario_info + "\n"
                        )
                
                # Add legends
                self.ax1.legend(loc='upper left', fontsize=8)
                self.ax2.legend(loc='upper right', fontsize=8)
                self.ax3.legend(loc='upper right', fontsize=8)

                # Global scenario summary
                global_scenario = None
                global_params = None
                for sensor_name, data in self.sensor_data.items():
                    if data.get('active_scenario'):
                        global_scenario = data['active_scenario']
                        global_params = data.get('scenario_parameters', {})
                        break

                if global_scenario:
                    summary_text.insert(0, 
                        f"ACTIVE SCENARIO: {global_scenario.upper()}\n"
                        f"Launch Command: roslaunch ... scenario:={global_scenario}\n"
                        f"Config File: rosbag_test_config.yaml\n"
                    )
                
                # Plot 4: Summary text
                if summary_text:
                    full_summary = "\n".join(summary_text)
                    full_summary += f"\ Total Messages: {self.total_messages}\n"
                    full_summary += f" Runtime: {time.time() - self.start_time:.1f}s"
                    
                    #  LIGHT GRAY BOX WITH BLACK TEXT 
                    self.ax4.text(0.05, 0.95, full_summary, 
                                 transform=self.ax4.transAxes,
                                 fontsize=8, verticalalignment='top',
                                 fontfamily='monospace',
                                 color='black',
                                 bbox=dict(boxstyle="round,pad=0.5", 
                                          facecolor='lightgray', 
                                          edgecolor='black',
                                          alpha=0.3))
                
                # Auto-scale axes
                for ax in [self.ax1, self.ax2, self.ax3]:
                    ax.relim()
                    ax.autoscale_view()
                
        except Exception as e:
            rospy.logwarn(f" Plot update error: {e}")
    
    def start_visualization(self):
        """Start the real-time visualization"""
        self.animation = animation.FuncAnimation(
            self.fig, 
            self.update_plots,
            interval=self.update_interval,
            blit=False,
            cache_frame_data=False
        )
        
        rospy.loginfo(" Starting real-time visualization...")
        plt.show()

def main():
    try:
        visualizer = LidarAugmentationVisualizer(
            buffer_size=200,
            update_interval=1000
        )
        visualizer.start_visualization()
        
    except rospy.ROSInterruptException:
        rospy.loginfo(" Visualizer stopped")
    except KeyboardInterrupt:
        rospy.loginfo(" Visualizer interrupted by user")

if __name__ == '__main__':
    main()