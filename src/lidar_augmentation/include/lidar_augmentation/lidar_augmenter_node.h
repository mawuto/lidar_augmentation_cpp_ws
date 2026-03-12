// include/lidar_augmentation/lidar_augmenter_node.h
#ifndef LIDAR_AUGMENTATION_LIDAR_AUGMENTER_NODE_H
#define LIDAR_AUGMENTATION_LIDAR_AUGMENTER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <unordered_map>
#include <string>
#include <memory>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include "point_cloud_processor.h"
#include "augmentation_methods.h"
#include "imu_synchronizer.h"

namespace lidar_augmentation
{

    // Augmentation parameters
    struct AugmentationParams
    {
        struct
        {
            bool enabled = false;
            float rate = 0.3f;
            std::string mode = "random";
        } dropout;

        struct
        {
            bool enabled = false;
            float horizontal = 0.2f;
            float vertical = 0.2f;
        } fov_reduction;

        struct
        {
            bool enabled = false;
            float gaussian_std = 0.02f;
            float outlier_rate = 0.01f;
            float outlier_std = 0.5f;
        } noise;

        struct
        {
            bool enabled = false;
            float distance_threshold = 50.0f;
            struct
            {
                int count = 3;
                float size = 1.5f;
            } random_patches;
        } occlusion;

        struct
        {
            bool enabled = false;
            int factor = 2;
        } sparse_scan;

        struct
        {
            bool enabled = false;
            std::vector<float> linear_velocity = {0.0f, 0.0f, 0.0f};
            std::vector<float> angular_velocity = {0.0f, 0.0f, 0.0f};
        } motion_distortion;
    };

    // Add statistics data structure
    struct AugmentationStatistics
    {
        std::string sensor_name;
        std::string sensor_type;
        ros::Time timestamp;

        // Point cloud statistics
        uint32_t original_points;
        uint32_t augmented_points;
        float point_reduction_percent;

        // Distance analysis
        float original_max_distance;
        float augmented_max_distance;
        float distance_preservation_percent;

        // Processing performance
        int64_t processing_time_ms;

        // Applied augmentations
        std::vector<std::string> applied_augmentations;
        std::string augmentation_params_summary;

        // Intensity statistics (if available)
        float original_intensity_mean;
        float original_intensity_std;
        float augmented_intensity_mean;
        float augmented_intensity_std;
    };

    class LidarAugmenterNode
    {
    public:
        LidarAugmenterNode();
        ~LidarAugmenterNode() = default;

        void run();

    private:
        // ROS components
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // Add after existing publishers for stats
        ros::Publisher stats_publisher_;
        bool publish_statistics_;

        // Statistics publishing throttling
        std::unordered_map<std::string, ros::Time> last_stats_publish_time_;
        double stats_publish_rate_; // Hz for statistics publishing

        // getting scenario param
        std::string active_scenario_name_ = "unknown";
        std::string getActiveScenario() const { return active_scenario_name_; }
        std::string getScenarioParametersJSON() const;

        std::unordered_map<std::string, ros::Subscriber> subscribers_;
        std::unordered_map<std::string, ros::Publisher> publishers_;
        std::unordered_map<std::string, ros::Subscriber> imu_subscribers_;

        // Processing components
        std::unique_ptr<PointCloudProcessor> processor_;
        std::unique_ptr<LidarAugmenter> augmenter_;
        std::unique_ptr<IMUSynchronizer> imu_sync_;

        // Configuration
        std::unordered_map<std::string, std::string> input_topics_;
        std::unordered_map<std::string, std::string> imu_topics_;
        std::string output_suffix_;
        AugmentationParams aug_params_;
        bool use_imu_;

        // Methods
        void loadParameters();
        void setupPublishers();
        void setupSubscribers();

        // Callbacks
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                const std::string &sensor);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

        // Processing methods
        template <typename PointT>
        void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg,
                               const std::string &sensor,
                               const std::string &sensor_type);

        // Missing utility methods
        void updateFieldsWithMask(
            std::unordered_map<std::string, std::vector<float>> &fields,
            const std::vector<bool> &mask);

        template <typename PointT>
        void updateFieldsWithCloud(
            std::unordered_map<std::string, std::vector<float>> &fields,
            const typename pcl::PointCloud<PointT>::Ptr &cloud);

        template <typename PointT>
        void updateXYZFields(
            std::unordered_map<std::string, std::vector<float>> &fields,
            const typename pcl::PointCloud<PointT>::Ptr &cloud);
        template <typename PointT>

        typename pcl::PointCloud<PointT>::Ptr simulateMotionDistortion(
            const typename pcl::PointCloud<PointT>::Ptr &cloud,
            const std::vector<uint32_t> &timestamps,
            const MotionParams &motion_params);

        template <typename PointT>
        void publishStatistics(const sensor_msgs::PointCloud2::ConstPtr &original_msg,
                               const typename pcl::PointCloud<PointT>::Ptr &augmented_cloud,
                               const std::string &sensor,
                               const std::string &sensor_type,
                               const std::chrono::milliseconds &processing_time);

        // Parameter loading
        void loadAugmentationParams();
        AugmentationParams parseAugmentationParams(const std::string &scenario_name);
        void overrideAugmentationParams();
        bool parseScenarioYAML(const YAML::Node &scenario, AugmentationParams &params);
        AugmentationParams getDefaultAugmentationParams(const std::string &scenario_name);
        void logAugmentationConfig();
    };

} // namespace lidar_augmentation

#endif // LIDAR_AUGMENTATION_LIDAR_AUGMENTER_NODE_H
