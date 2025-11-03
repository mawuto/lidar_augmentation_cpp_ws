#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import time

def main():
    rospy.init_node('topic_diagnostics', anonymous=True)
    
    print(" Scanning for PointCloud2 topics...")
    time.sleep(2)  # Wait for topics to be available
    
    # Get all topics
    topics = rospy.get_published_topics()
    
    print("\n Available PointCloud2 topics:")
    print("="*50)
    
    lidar_topics = []
    augmented_topics = []
    
    for topic_name, topic_type in topics:
        if topic_type == 'sensor_msgs/PointCloud2':
            print(f"  {topic_name}")
            
            if 'augmented' in topic_name or 'lidar_augmentation' in topic_name:
                augmented_topics.append(topic_name)
            else:
                lidar_topics.append(topic_name)
    
    print(f"\n ORIGINAL TOPICS ({len(lidar_topics)}):")
    for topic in lidar_topics:
        print(f"   {topic}")
    
    print(f"\n AUGMENTED TOPICS ({len(augmented_topics)}):")
    for topic in augmented_topics:
        print(f"   {topic}")
    
    if not lidar_topics:
        print("\n No original LiDAR topics found!")
        print("   Make sure your rosbag is playing.")
    
    if not augmented_topics:
        print("\n No augmented topics found!")
        print("   Make sure your C++ augmentation node is running.")
    
    print("\n" + "="*50)

if __name__ == '__main__':
    main()
