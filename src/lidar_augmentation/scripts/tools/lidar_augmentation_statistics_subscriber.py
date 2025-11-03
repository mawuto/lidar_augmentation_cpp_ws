#!/usr/bin/env python3

import rospy
import json
import numpy as np
from collections import defaultdict, deque
from std_msgs.msg import String
from threading import Lock
import time

class LidarAugmentationStatsSubscriber:
    def __init__(self, buffer_size=1000):
        """
        Initialize the statistics subscriber
        
        Args:
            buffer_size (int): Maximum number of data points to keep per sensor
        """
        rospy.init_node('lidar_stats_subscriber', anonymous=True)
        
        # Data storage with thread safety
        self.buffer_size = buffer_size
        self.data_lock = Lock()
        
        # Separate data storage for each sensor
        self.sensor_data = {}
        
        # Statistics tracking
        self.total_messages = 0
        self.error_count = 0
        self.start_time = time.time()
        
        # Subscribe to statistics topic
        self.stats_subscriber = rospy.Subscriber(
            '/lidar_augmentation/statistics',
            String,
            self.statistics_callback,
            queue_size=100
        )
        
        rospy.loginfo("LiDAR Statistics Subscriber initialized")
        rospy.loginfo(f"Buffer size: {buffer_size} points per sensor")
        rospy.loginfo("Waiting for statistics messages...")
        
    def statistics_callback(self, msg):
        """
        Process incoming statistics messages
        """
        try:
            # Parse JSON data
            stats = json.loads(msg.data)
            
            # Extract key information
            sensor_name = stats.get('sensor', 'unknown')
            sensor_type = stats.get('sensor_type', 'unknown')
            timestamp = float(stats.get('timestamp', 0))
            original_points = int(stats.get('original_points', 0))
            augmented_points = int(stats.get('augmented_points', 0))
            reduction_percent = float(stats.get('point_reduction_percent', 0))
            processing_time = int(stats.get('processing_time_ms', 0))
            active_scenario = stats.get('active_scenario', 'unknown')
            scenario_parameters = stats.get('scenario_parameters', {})

            # Print scenario info
            print(f"   Active Scenario: {active_scenario}")
            if scenario_parameters:
                print(f"   Parameters: {scenario_parameters}")
                        
            # Thread-safe data storage
            with self.data_lock:
                # Initialize sensor data structure if needed
                if sensor_name not in self.sensor_data:
                    self.sensor_data[sensor_name] = {
                        'sensor_type': sensor_type,
                        'timestamps': deque(maxlen=self.buffer_size),
                        'original_points': deque(maxlen=self.buffer_size),
                        'augmented_points': deque(maxlen=self.buffer_size),
                        'reduction_percent': deque(maxlen=self.buffer_size),
                        'processing_time': deque(maxlen=self.buffer_size),
                        'message_count': 0,
                        'last_update': time.time()
                    }
                
                # Store data
                sensor_data = self.sensor_data[sensor_name]
                sensor_data['timestamps'].append(timestamp)
                sensor_data['original_points'].append(original_points)
                sensor_data['augmented_points'].append(augmented_points)
                sensor_data['reduction_percent'].append(reduction_percent)
                sensor_data['processing_time'].append(processing_time)
                sensor_data['message_count'] += 1
                sensor_data['last_update'] = time.time()
                
                self.total_messages += 1
                
                # Log periodic updates (every 50 messages)
                if self.total_messages % 50 == 0:
                    self.log_status()
                    
        except json.JSONDecodeError as e:
            self.error_count += 1
            rospy.logwarn(f" JSON parsing error: {e}")
        except Exception as e:
            self.error_count += 1
            rospy.logwarn(f" Statistics processing error: {e}")
    
    def get_sensor_list(self):
        """Get list of available sensors"""
        with self.data_lock:
            return list(self.sensor_data.keys())
    
    def get_sensor_data(self, sensor_name):
        """
        Get time-series data for a specific sensor
        
        Returns:
            dict: Sensor data or None if sensor not found
        """
        with self.data_lock:
            if sensor_name in self.sensor_data:
                data = self.sensor_data[sensor_name].copy()
                # Convert deques to lists for easier handling
                for key in ['timestamps', 'original_points', 'augmented_points', 
                           'reduction_percent', 'processing_time']:
                    data[key] = list(data[key])
                return data
            return None
    
    def get_all_sensor_data(self):
        """Get data for all sensors"""
        with self.data_lock:
            all_data = {}
            for sensor_name in self.sensor_data:
                all_data[sensor_name] = self.get_sensor_data(sensor_name)
            return all_data
    
    def get_latest_stats(self):
        """Get the most recent statistics for all sensors"""
        stats = {}
        with self.data_lock:
            for sensor_name, data in self.sensor_data.items():
                if len(data['timestamps']) > 0:
                    stats[sensor_name] = {
                        'sensor_type': data['sensor_type'],
                        'latest_timestamp': data['timestamps'][-1],
                        'latest_original_points': data['original_points'][-1],
                        'latest_augmented_points': data['augmented_points'][-1],
                        'latest_reduction_percent': data['reduction_percent'][-1],
                        'latest_processing_time': data['processing_time'][-1],
                        'message_count': data['message_count'],
                        'avg_reduction': np.mean(data['reduction_percent']),
                        'avg_processing_time': np.mean(data['processing_time'])
                    }
        return stats
    
    def log_status(self):
        """Log current status"""
        uptime = time.time() - self.start_time
        rate = self.total_messages / uptime if uptime > 0 else 0
        
        rospy.loginfo("=" * 60)
        rospy.loginfo(f" STATISTICS SUMMARY (Uptime: {uptime:.1f}s)")
        rospy.loginfo(f" Total messages: {self.total_messages} ({rate:.1f} msg/s)")
        rospy.loginfo(f" Errors: {self.error_count}")
        rospy.loginfo(f" Active sensors: {len(self.sensor_data)}")
        
        # Per-sensor summary
        latest_stats = self.get_latest_stats()
        for sensor_name, stats in latest_stats.items():
            rospy.loginfo(f"   {sensor_name} ({stats['sensor_type']}): "
                         f"{stats['message_count']} msgs, "
                         f"{stats['avg_reduction']:.1f}% avg reduction, "
                         f"{stats['avg_processing_time']:.1f}ms avg processing")
        rospy.loginfo("=" * 60)
    
    def print_realtime_stats(self):
        """Print real-time statistics to console"""
        rate = rospy.Rate(2)  # 2 Hz updates
        
        while not rospy.is_shutdown():
            latest_stats = self.get_latest_stats()
            
            if latest_stats:
                print("\n" + "="*80)
                print(" REAL-TIME LIDAR AUGMENTATION STATISTICS")
                print("="*80)
                
                for sensor_name, stats in latest_stats.items():
                    print(f" {sensor_name.upper()} ({stats['sensor_type']})")
                    print(f"    Points: {stats['latest_original_points']:,} → "
                          f"{stats['latest_augmented_points']:,} "
                          f"({stats['latest_reduction_percent']:+.1f}%)")
                    print(f"     Processing: {stats['latest_processing_time']}ms "
                          f"(avg: {stats['avg_processing_time']:.1f}ms)")
                    print(f"    Messages: {stats['message_count']} "
                          f"(avg reduction: {stats['avg_reduction']:.1f}%)")
                    print()
                
                print(f" Total messages: {self.total_messages}")
                print("="*80)
            else:
                print(" Waiting for statistics data...")
            
            rate.sleep()

def main():
    try:
        # Create subscriber
        subscriber = LidarAugmentationStatsSubscriber(buffer_size=1000)
        
        # Print real-time stats to console
        subscriber.print_realtime_stats()
        
    except rospy.ROSInterruptException:
        rospy.loginfo(" Statistics subscriber stopped")
    except KeyboardInterrupt:
        rospy.loginfo(" Statistics subscriber interrupted by user")

if __name__ == '__main__':
    main()
