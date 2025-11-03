#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from threading import Lock
import time
import os
from datetime import datetime

class LidarBEVVisualizer:
    def __init__(self, update_interval=2000):
        """FIXED BEV Visualizer - Preserves actual point counts"""
        rospy.init_node('lidar_bev_visualizer', anonymous=True)
        
        self.update_interval = update_interval
        self.data_lock = Lock()
        
        # Data storage
        self.latest_clouds = {}
        self.has_ever_received_data = False
        self.last_data_time = None
        self.data_timeout = 15.0
        
        # Screenshot control
        self.auto_save_enabled = True
        self.save_directory = os.path.expanduser("~/lidar_bev_screenshots")
        self.last_save_time = 0
        self.save_interval = 30.0
        self.final_screenshot_saved = False
        self.rosbag_finished = False
        
        if self.auto_save_enabled:
            os.makedirs(self.save_directory, exist_ok=True)
            rospy.loginfo(f" Auto-save: {self.save_directory}")
        
        # Sensor topics
        self.sensor_topics = {
            'livox_avia': {
                'original': '/avia/livox/lidar',
                'augmented': '/avia/livox/lidar_augmented',
                'color': '#1f77b4'
            },
            'ouster': {
                'original': '/ouster/points',
                'augmented': '/ouster/points_augmented', 
                'color': '#ff7f0e'
            },
            'livox_mid360': {
                'original': '/mid360/livox/lidar',
                'augmented': '/mid360/livox/lidar_augmented',
                'color': '#2ca02c'
            }
        }
        
        self.bev_range = 100.0     #100 for outdoor and 50 for indoor
        self.point_size = 0.6
        
        plt.style.use('dark_background')
        plt.rcParams['figure.max_open_warning'] = 0
        
        self.setup_plots()
        self.setup_subscribers()
        
        rospy.loginfo(" FIXED BEV Visualizer initialized")
        
    def setup_subscribers(self):
        """Setup subscribers"""
        self.subscribers = {}
        
        for sensor_name, topics in self.sensor_topics.items():
            self.subscribers[f"{sensor_name}_original"] = rospy.Subscriber(
                topics['original'], PointCloud2, 
                lambda msg, sn=sensor_name: self.original_callback(msg, sn),
                queue_size=1, buff_size=2**24
            )
            
            self.subscribers[f"{sensor_name}_augmented"] = rospy.Subscriber(
                topics['augmented'], PointCloud2,
                lambda msg, sn=sensor_name: self.augmented_callback(msg, sn),
                queue_size=1, buff_size=2**24
            )
            
        rospy.loginfo(" Waiting for topics...")
    
    def pointcloud2_to_xyz_all(self, cloud_msg):
        """Convert ALL points without artificial limits"""
        try:
            points_list = []
            
            for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = point[0], point[1], point[2]
                
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    points_list.append([x, y, z])
            
            return np.array(points_list) if points_list else np.empty((0, 3))
            
        except Exception as e:
            rospy.logwarn(f"Conversion error: {e}")
            return np.empty((0, 3))
    
    def setup_plots(self):
        """Setup visualization plots"""
        self.fig = plt.figure(figsize=(18, 14))
        self.fig.suptitle('LiDAR BEV Comparison - FIXED', 
                         fontsize=16, fontweight='bold', color='white')
        
        self.ax1 = plt.subplot(2, 2, 1)
        self.ax2 = plt.subplot(2, 2, 2)
        self.ax3 = plt.subplot(2, 2, 3)
        self.ax4 = plt.subplot(2, 2, 4)
        
        axes_info = [
            (self.ax1, 'Original Point Clouds'),
            (self.ax2, 'Augmented Point Clouds'),
            (self.ax3, 'Removed Points'),
            (self.ax4, 'Side-by-Side')
        ]
        
        for ax, title in axes_info:
            ax.set_title(title, fontweight='bold', fontsize=12)
            ax.set_xlabel('X (meters)')
            ax.set_ylabel('Y (meters)')
            ax.set_xlim(-self.bev_range, self.bev_range)
            ax.set_ylim(-self.bev_range, self.bev_range)
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
        
        plt.tight_layout()
    
    def downsample_for_vis(self, points, max_points=4000):
        """Downsample only for visualization"""
        if len(points) <= max_points:
            return points
            
        bev_mask = (np.abs(points[:, 0]) <= self.bev_range) & \
                   (np.abs(points[:, 1]) <= self.bev_range)
        bev_points = points[bev_mask]
        
        if len(bev_points) <= max_points:
            return bev_points
            
        indices = np.random.choice(len(bev_points), max_points, replace=False)
        return bev_points[indices]
    
    def find_removed_points(self, original_points, augmented_points):
        """Show representative removed points"""
        if len(original_points) == 0:
            return np.empty((0, 3))
        
        if len(augmented_points) == 0:
            return self.downsample_for_vis(original_points, 500)
        
        actual_removed = len(original_points) - len(augmented_points)
        
        if actual_removed <= 0:
            return np.empty((0, 3))
        
        bev_mask = (np.abs(original_points[:, 0]) <= self.bev_range) & \
                   (np.abs(original_points[:, 1]) <= self.bev_range)
        bev_original = original_points[bev_mask]
        
        if len(bev_original) == 0:
            return np.empty((0, 3))
        
        removal_percentage = actual_removed / len(original_points)
        sample_size = min(int(len(bev_original) * removal_percentage), 1000)
        
        if sample_size > 0:
            indices = np.random.choice(len(bev_original), sample_size, replace=False)
            return bev_original[indices]
        
        return np.empty((0, 3))
    
    def save_screenshot(self, suffix=""):
        """Save screenshot"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"lidar_bev_{timestamp}{suffix}.png"
            filepath = os.path.join(self.save_directory, filename)
            
            self.fig.savefig(filepath, dpi=300, bbox_inches='tight', 
                           facecolor='black', edgecolor='none')
            
            self.last_save_time = time.time()
            rospy.loginfo(f" Saved: {filename}")
            return filepath
            
        except Exception as e:
            rospy.logwarn(f" Save error: {e}")
            return None
    def are_data_synchronized(self, cloud_data, max_time_diff=0.5):
        """Check if original and augmented data are from similar timestamps"""
        if 'original_timestamp' not in cloud_data or 'augmented_timestamp' not in cloud_data:
            return False
        
        # SYNC FOR OUSTER 
        sensor_name = cloud_data.get('sensor_name', '')
        if 'ouster' in sensor_name.lower():
            max_time_diff = 0.2  # Stricter sync for Ouster
        
        time_diff = abs(cloud_data['original_timestamp'] - cloud_data['augmented_timestamp'])
        return time_diff < max_time_diff

    def original_callback(self, msg, sensor_name):
        """FIXED: Add timestamp tracking"""
        try:
            points = self.pointcloud2_to_xyz_all(msg)
            
            with self.data_lock:
                if sensor_name not in self.latest_clouds:
                    self.latest_clouds[sensor_name] = {}
                
                self.latest_clouds[sensor_name]['original'] = points
                self.latest_clouds[sensor_name]['original_count'] = len(points)
                self.latest_clouds[sensor_name]['original_timestamp'] = time.time()
                
                self.last_data_time = time.time()
                if not self.has_ever_received_data:
                    self.has_ever_received_data = True
                    rospy.loginfo(f"First data from {sensor_name}!")
                    
        except Exception as e:
            rospy.logwarn(f"Error {sensor_name}: {e}")

    def augmented_callback(self, msg, sensor_name):
        """FIXED: Add timestamp tracking"""
        try:
            points = self.pointcloud2_to_xyz_all(msg)
            
            with self.data_lock:
                if sensor_name not in self.latest_clouds:
                    self.latest_clouds[sensor_name] = {}
                
                self.latest_clouds[sensor_name]['augmented'] = points
                self.latest_clouds[sensor_name]['augmented_count'] = len(points)
                self.latest_clouds[sensor_name]['augmented_timestamp'] = time.time()
                
                self.last_data_time = time.time()
                if not self.has_ever_received_data:
                    self.has_ever_received_data = True
                    rospy.loginfo(f"First data from {sensor_name}!")
                    
        except Exception as e:
            rospy.logwarn(f"Error {sensor_name}: {e}")
    
    def update_plots(self, frame):
        """Update all plots"""
        try:
            with self.data_lock:
                current_time = time.time()
                
                data_is_stale = (self.last_data_time is not None and 
                               current_time - self.last_data_time > self.data_timeout)
                
                if data_is_stale and self.has_ever_received_data and not self.rosbag_finished:
                    self.rosbag_finished = True
                    
                    if not self.final_screenshot_saved:
                        self.save_screenshot("_FINAL")
                        self.final_screenshot_saved = True
                        rospy.loginfo(" Final screenshot saved!")
                
                # Clear axes
                for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
                    ax.clear()
                    ax.set_xlim(-self.bev_range, self.bev_range)
                    ax.set_ylim(-self.bev_range, self.bev_range)
                    ax.grid(True, alpha=0.3)
                    ax.set_aspect('equal')
                
                self.ax1.set_title('Original Point Clouds', fontweight='bold')
                self.ax2.set_title('Augmented Point Clouds', fontweight='bold')
                self.ax3.set_title('Removed Points', fontweight='bold')
                self.ax4.set_title('Side-by-Side', fontweight='bold')
                
                if not self.has_ever_received_data:
                    self.ax1.text(0, 0, 'Waiting for data...', 
                                 ha='center', va='center', color='yellow')
                    return
                
                elif self.rosbag_finished:
                    self.ax1.text(0, 0, 'ROSBAG FINISHED \nPress Ctrl+C to exit', 
                                 ha='center', va='center', color='lightgreen')
                    return
                
                # Process active data
                has_data = False
                total_removed = 0
                
                for sensor_name, cloud_data in self.latest_clouds.items():
                    if 'original' not in cloud_data or 'augmented' not in cloud_data:
                        continue

                    # SYNCHRONIZATION CHECK
                    if not self.are_data_synchronized(cloud_data):
                        continue  # Skip unsynchronized data
                    
                    original_points = cloud_data['original']
                    augmented_points = cloud_data['augmented']
                    
                    original_count = cloud_data.get('original_count', len(original_points))
                    augmented_count = cloud_data.get('augmented_count', len(augmented_points))
                    
                    if original_count == 0 and augmented_count == 0:
                        continue
                    
                    has_data = True
                    removed_count = max(0, original_count - augmented_count)
                    total_removed += removed_count
                    
                    color = self.sensor_topics.get(sensor_name, {}).get('color', '#888888')
                    
                    # Visualization downsampling
                    orig_vis = self.downsample_for_vis(original_points)
                    aug_vis = self.downsample_for_vis(augmented_points)
                    
                    # Plot original
                    if len(orig_vis) > 0:
                        self.ax1.scatter(orig_vis[:, 0], orig_vis[:, 1], 
                                        c=color, s=self.point_size, alpha=0.7,
                                        label=f"{sensor_name} ({original_count:,})")
                    
                    # Plot augmented
                    if len(aug_vis) > 0:
                        self.ax2.scatter(aug_vis[:, 0], aug_vis[:, 1], 
                                        c=color, s=self.point_size, alpha=0.7,
                                        label=f"{sensor_name} ({augmented_count:,})")
                    
                    # Plot removed points
                    removed_vis = self.find_removed_points(original_points, augmented_points)
                    if len(removed_vis) > 0:
                        self.ax3.scatter(removed_vis[:, 0], removed_vis[:, 1], 
                                        c='red', s=self.point_size*2, alpha=0.9,
                                        label=f"{sensor_name} (-{removed_count:,})")
                    
                    # Side-by-side
                    if len(orig_vis) > 0:
                        self.ax4.scatter(orig_vis[:, 0] - 25, orig_vis[:, 1], 
                                        c=color, s=self.point_size, alpha=0.7)
                    
                    if len(aug_vis) > 0:
                        self.ax4.scatter(aug_vis[:, 0] + 25, aug_vis[:, 1], 
                                        c=color, s=self.point_size, alpha=0.7, marker='x')
                
                # Add legends
                if has_data:
                    for ax in [self.ax1, self.ax2, self.ax3]:
                        handles, labels = ax.get_legend_handles_labels()
                        if handles:
                            ax.legend(loc='upper right', fontsize=9)
                
                # Total removed info
                if total_removed > 0:
                    self.ax3.text(0, self.bev_range*0.9, f'Total Removed: {total_removed:,}', 
                                 ha='center', fontweight='bold', color='red', fontsize=12,
                                 bbox=dict(boxstyle="round", facecolor='black', edgecolor='red'))
                
                # Side-by-side labels
                self.ax4.axvline(x=0, color='white', linestyle='--', alpha=0.7)
                self.ax4.text(-25, self.bev_range*0.9, 'ORIGINAL', 
                             ha='center', fontweight='bold', color='white')
                self.ax4.text(25, self.bev_range*0.9, 'AUGMENTED', 
                             ha='center', fontweight='bold', color='white')
                
                # Auto-save logic
                if has_data and not self.rosbag_finished:
                    if current_time - self.last_save_time > self.save_interval:
                        self.save_screenshot()
                
        except Exception as e:
            rospy.logwarn(f" Update error: {e}")
    
    def start_visualization(self):
        """Start visualization"""
        try:
            self.animation = animation.FuncAnimation(
                self.fig, self.update_plots, interval=self.update_interval,
                blit=False, cache_frame_data=False
            )
            
            rospy.loginfo(" Starting visualization...")
            plt.show(block=True)
            
        except KeyboardInterrupt:
            rospy.loginfo(" Stopped by user")
        except Exception as e:
            rospy.logerr(f" Error: {e}")

def main():
    try:
        # 500ms = 0.5s = 2Hz (matching C++ statistics throttling)
        update_rate_ms = 500
        
        visualizer = LidarBEVVisualizer(update_interval=update_rate_ms)
        visualizer.start_visualization()
        
    except rospy.ROSInterruptException:
        rospy.loginfo(" ROS shutdown")
    except KeyboardInterrupt:
        rospy.loginfo(" Interrupted")
    finally:
        plt.close('all')

if __name__ == '__main__':
    main()