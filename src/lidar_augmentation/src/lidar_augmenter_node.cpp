// src/cpp/lidar_augmenter_node.cpp
#include "lidar_augmentation/lidar_augmenter_node.h"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <chrono>
#include <fstream>
#include <sstream>
#include <std_msgs/String.h>
#include <iomanip>

namespace lidar_augmentation
{

    LidarAugmenterNode::LidarAugmenterNode()
        : nh_(), private_nh_("~")
    {

        ROS_INFO("Initializing LiDAR Augmenter Node (C++ Implementation)");

        // Initialize processing components
        processor_ = std::make_unique<PointCloudProcessor>();
        augmenter_ = std::make_unique<LidarAugmenter>();
        imu_sync_ = std::make_unique<IMUSynchronizer>(1000);

        // Configure statistics publishing rate (default 2Hz for performance)
        private_nh_.param<double>("stats_publish_rate", stats_publish_rate_, 2.0);
        ROS_INFO_STREAM("Statistics publishing rate: " << stats_publish_rate_ << " Hz");

        // Load configuration parameters
        loadParameters();

        // Setup ROS interface
        setupPublishers();
        setupSubscribers();

        ROS_INFO("LiDAR Augmenter Node initialized successfully");
        ROS_INFO_STREAM("Configured sensors: " << input_topics_.size());
        for (const auto &[sensor_name, topic] : input_topics_)
        {
            ROS_INFO_STREAM("  - " << sensor_name << ": " << topic << " -> " << topic << output_suffix_);
        }

        if (use_imu_)
        {
            ROS_INFO_STREAM("IMU synchronization enabled with " << imu_topics_.size() << " topics");

            // Load synchronization delay parameter
            private_nh_.param<double>("sync_delay", sync_delay_sec_, 0.030);
            ROS_INFO_STREAM("IMU-LiDAR synchronization delay buffer: "
                            << sync_delay_sec_ * 1000.0 << " ms");

            // Start timer at 1 kHz to flush delayed output buffers
            sync_flush_timer_ = nh_.createTimer(
                ros::Duration(0.001),
                &LidarAugmenterNode::syncFlushCallback, this);
        }
    }

    void LidarAugmenterNode::loadParameters()
    {
        // ==========================================================================
        // GENERIC TOPIC LOADING - Works with ANY rosbag topic structure
        // ==========================================================================

        input_topics_.clear();

        private_nh_.param<bool>("output/publish_statistics", publish_statistics_, false);

        if (publish_statistics_)
        {
            ROS_INFO("Statistics publishing ENABLED - Python analyzer can subscribe");
        }
        else
        {
            ROS_INFO("Statistics publishing DISABLED - use publish_statistics:=true to enable");
        }

        // Method 1: Load from ROS parameters
        XmlRpc::XmlRpcValue input_topics_param;
        if (private_nh_.getParam("input_topics", input_topics_param))
        {

            if (input_topics_param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                // Parse structured input_topics parameter
                for (auto &item : input_topics_param)
                {
                    std::string sensor_name = item.first;
                    std::string topic_name = static_cast<std::string>(item.second);
                    input_topics_[sensor_name] = topic_name;
                    ROS_INFO_STREAM("Configured sensor '" << sensor_name << "' -> topic '" << topic_name << "'");
                }
            }
        }
        else
        {
            // Method 2: Auto-discover from individual topic parameters
            std::vector<std::string> potential_sensors = {
                "sensor1", "sensor2", "sensor3", "sensor4", "sensor5",   // Generic sensors
                "lidar1", "lidar2", "lidar3",                            // Generic LiDARs
                "ouster", "ouster1", "ouster2",                          // Ouster variants
                "livox", "livox1", "livox2",                             // Livox variants
                "avia", "avia1", "avia2",                                // Avia variants
                "mid360", "mid360_1", "mid360_2",                        // Mid360 variants
                "velodyne", "hesai", "robosense",                        // Other brands
                "front_lidar", "rear_lidar", "left_lidar", "right_lidar" // Positional
            };

            for (const std::string &sensor : potential_sensors)
            {
                std::string topic;
                if (private_nh_.getParam("topics/" + sensor, topic) ||
                    private_nh_.getParam(sensor + "_topic", topic))
                {
                    input_topics_[sensor] = topic;
                    ROS_INFO_STREAM("Auto-discovered sensor '" << sensor << "' -> topic '" << topic << "'");
                }
            }

            // Method 3: Single topic mode for simple cases
            if (input_topics_.empty())
            {
                std::string single_topic;
                if (private_nh_.getParam("topic", single_topic) ||
                    private_nh_.getParam("input_topic", single_topic))
                {
                    input_topics_["main_sensor"] = single_topic;
                    ROS_INFO_STREAM("Single sensor mode: '" << single_topic << "'");
                }
            }

            // Method 4: Fallback to common topic patterns if nothing configured
            if (input_topics_.empty())
            {
                ROS_WARN("No input topics configured! Attempting to auto-detect common patterns...");

                // Try multiple times to find topics (rosbag compatibility)
                int max_retries = 15; // 15 attempts = 30 seconds (2s intervals)
                int retry_count = 0;

                while (input_topics_.empty() && retry_count < max_retries && ros::ok())
                {
                    ros::master::V_TopicInfo topic_infos;
                    if (ros::master::getTopics(topic_infos))
                    {
                        std::vector<std::string> point_cloud_topics;
                        for (const auto &topic_info : topic_infos)
                        {
                            if (topic_info.datatype == "sensor_msgs/PointCloud2")
                            {
                                point_cloud_topics.push_back(topic_info.name);
                            }
                        }

                        // Configure detected topics
                        for (size_t i = 0; i < point_cloud_topics.size() && i < 5; ++i)
                        {
                            std::string sensor_name = "auto_sensor_" + std::to_string(i + 1);
                            input_topics_[sensor_name] = point_cloud_topics[i];
                            ROS_INFO_STREAM("Auto-detected: '" << sensor_name << "' -> '" << point_cloud_topics[i] << "'");
                        }
                    }

                    if (input_topics_.empty())
                    {
                        retry_count++;
                        ROS_INFO_STREAM("No PointCloud2 topics found yet. Retrying in 2 seconds... ("
                                        << retry_count << "/" << max_retries << ")");
                        ros::Duration(2.0).sleep(); // Wait 2 seconds
                    }
                }

                if (input_topics_.empty())
                {
                    ROS_ERROR("No PointCloud2 topics found after 30 seconds! Please start your rosbag or configure input_topics parameter.");
                    return;
                }
            }
        }

        // Load output suffix
        output_suffix_ = "_augmented";
        private_nh_.param<std::string>("output_suffix", output_suffix_, "_augmented");

        // ==========================================================================
        // GENERIC IMU LOADING - Works with ANY IMU topic structure
        // ==========================================================================

        imu_topics_.clear();
        use_imu_ = false;
        // Read from "use_imu" direct param OR from "imu/enabled" (YAML structure)
        if (!private_nh_.param<bool>("use_imu", use_imu_, false))
        {
            private_nh_.param<bool>("imu/enabled", use_imu_, false);
        }
        ROS_INFO_STREAM("IMU usage: " << (use_imu_ ? "ENABLED" : "DISABLED")
                                      << " (param: use_imu=" << use_imu_ << ")");

        if (use_imu_)
        {
            // Method 1: Load from structured IMU topics parameter
            XmlRpc::XmlRpcValue imu_topics_param;
            if (private_nh_.getParam("imu_topics", imu_topics_param))
            {

                if (imu_topics_param.getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    for (auto &item : imu_topics_param)
                    {
                        std::string sensor_name = item.first;
                        std::string imu_topic = static_cast<std::string>(item.second);
                        imu_topics_[sensor_name] = imu_topic;
                        ROS_INFO_STREAM("Configured IMU for '" << sensor_name << "' -> '" << imu_topic << "'");
                    }
                }
            }
            else
            {
                // Method 2: Auto-discover IMU topics for configured sensors
                for (const auto &[sensor_name, lidar_topic] : input_topics_)
                {
                    std::string imu_topic;

                    // Try various parameter naming patterns
                    if (private_nh_.getParam("imu_topics/" + sensor_name, imu_topic) ||
                        private_nh_.getParam(sensor_name + "_imu", imu_topic) ||
                        private_nh_.getParam(sensor_name + "_imu_topic", imu_topic))
                    {

                        imu_topics_[sensor_name] = imu_topic;
                        ROS_INFO_STREAM("Found IMU topic for '" << sensor_name << "': " << imu_topic);
                    }
                }

                // Method 3: Single IMU for all sensors
                if (imu_topics_.empty())
                {
                    std::string global_imu_topic;
                    if (private_nh_.getParam("imu_topic", global_imu_topic))
                    {
                        for (const auto &[sensor_name, lidar_topic] : input_topics_)
                        {
                            imu_topics_[sensor_name] = global_imu_topic;
                        }
                        ROS_INFO_STREAM("Using single IMU topic for all sensors: " << global_imu_topic);
                    }
                }

                // Method 4: Auto-detect IMU topics by topic pattern matching
                if (imu_topics_.empty())
                {
                    ros::master::V_TopicInfo topic_infos;
                    if (ros::master::getTopics(topic_infos))
                    {

                        for (const auto &[sensor_name, lidar_topic] : input_topics_)
                        {
                            // Try to find IMU topic with similar name pattern
                            for (const auto &topic_info : topic_infos)
                            {
                                if (topic_info.datatype == "sensor_msgs/Imu")
                                {
                                    std::string topic_name = topic_info.name;

                                    // Check if IMU topic has similar namespace to LiDAR topic
                                    size_t last_slash = lidar_topic.find_last_of('/');
                                    if (last_slash != std::string::npos)
                                    {
                                        std::string lidar_namespace = lidar_topic.substr(0, last_slash);
                                        if (topic_name.find(lidar_namespace) == 0)
                                        {
                                            imu_topics_[sensor_name] = topic_name;
                                            ROS_INFO_STREAM("Auto-matched IMU: '" << sensor_name << "' -> '" << topic_name << "'");
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (imu_topics_.empty())
            {
                ROS_WARN("IMU enabled but no IMU topics configured. Motion distortion will use static parameters.");
            }
        }

        // Load augmentation parameters
        loadAugmentationParams();

        ROS_INFO_STREAM("Configuration loaded: " << input_topics_.size() << " sensors, "
                                                 << "output_suffix='" << output_suffix_ << "', use_imu=" << use_imu_);
    }

    void LidarAugmenterNode::loadAugmentationParams()
    {
        // Load scenario name - fully configurable
        std::string scenario_name = "moderate";
        private_nh_.param<std::string>("scenario", scenario_name, "moderate");

        // Store active scenario name after loading it
        active_scenario_name_ = scenario_name;

        ROS_INFO_STREAM("Loading augmentation scenario: '" << scenario_name << "'");

        // Parse augmentation parameters from scenario
        aug_params_ = parseAugmentationParams(scenario_name);

        // Allow runtime parameter overrides for any augmentation parameter
        overrideAugmentationParams();

        // Log final configuration
        logAugmentationConfig();
    }

    void LidarAugmenterNode::overrideAugmentationParams()
    {
        // Allow runtime overrides of any augmentation parameter

        // Dropout overrides
        private_nh_.param<bool>("augmentation/dropout/enabled", aug_params_.dropout.enabled, aug_params_.dropout.enabled);
        private_nh_.param<float>("augmentation/dropout/rate", aug_params_.dropout.rate, aug_params_.dropout.rate);
        private_nh_.param<std::string>("augmentation/dropout/mode", aug_params_.dropout.mode, aug_params_.dropout.mode);

        // FOV reduction overrides
        private_nh_.param<bool>("augmentation/fov_reduction/enabled", aug_params_.fov_reduction.enabled, aug_params_.fov_reduction.enabled);
        private_nh_.param<float>("augmentation/fov_reduction/horizontal", aug_params_.fov_reduction.horizontal, aug_params_.fov_reduction.horizontal);
        private_nh_.param<float>("augmentation/fov_reduction/vertical", aug_params_.fov_reduction.vertical, aug_params_.fov_reduction.vertical);

        // Noise overrides
        private_nh_.param<bool>("augmentation/noise/enabled", aug_params_.noise.enabled, aug_params_.noise.enabled);
        private_nh_.param<float>("augmentation/noise/gaussian_std", aug_params_.noise.gaussian_std, aug_params_.noise.gaussian_std);
        private_nh_.param<float>("augmentation/noise/outlier_rate", aug_params_.noise.outlier_rate, aug_params_.noise.outlier_rate);
        private_nh_.param<float>("augmentation/noise/outlier_std", aug_params_.noise.outlier_std, aug_params_.noise.outlier_std);

        // Occlusion overrides
        private_nh_.param<bool>("augmentation/occlusion/enabled", aug_params_.occlusion.enabled, aug_params_.occlusion.enabled);
        private_nh_.param<float>("augmentation/occlusion/distance_threshold", aug_params_.occlusion.distance_threshold, aug_params_.occlusion.distance_threshold);
        private_nh_.param<int>("augmentation/occlusion/random_patches_count", aug_params_.occlusion.random_patches.count, aug_params_.occlusion.random_patches.count);
        private_nh_.param<float>("augmentation/occlusion/random_patches_size", aug_params_.occlusion.random_patches.size, aug_params_.occlusion.random_patches.size);

        // Sparse scan overrides
        private_nh_.param<bool>("augmentation/sparse_scan/enabled", aug_params_.sparse_scan.enabled, aug_params_.sparse_scan.enabled);
        private_nh_.param<int>("augmentation/sparse_scan/factor", aug_params_.sparse_scan.factor, aug_params_.sparse_scan.factor);

        // Motion distortion overrides
        private_nh_.param<bool>("augmentation/motion_distortion/enabled", aug_params_.motion_distortion.enabled, aug_params_.motion_distortion.enabled);

        // Load motion vectors if provided
        XmlRpc::XmlRpcValue linear_vel_param, angular_vel_param;
        if (private_nh_.getParam("augmentation/motion_distortion/linear_velocity", linear_vel_param))
        {
            if (linear_vel_param.getType() == XmlRpc::XmlRpcValue::TypeArray && linear_vel_param.size() >= 3)
            {
                aug_params_.motion_distortion.linear_velocity = {
                    static_cast<float>(static_cast<double>(linear_vel_param[0])),
                    static_cast<float>(static_cast<double>(linear_vel_param[1])),
                    static_cast<float>(static_cast<double>(linear_vel_param[2]))};
            }
        }

        if (private_nh_.getParam("augmentation/motion_distortion/angular_velocity", angular_vel_param))
        {
            if (angular_vel_param.getType() == XmlRpc::XmlRpcValue::TypeArray && angular_vel_param.size() >= 3)
            {
                aug_params_.motion_distortion.angular_velocity = {
                    static_cast<float>(static_cast<double>(angular_vel_param[0])),
                    static_cast<float>(static_cast<double>(angular_vel_param[1])),
                    static_cast<float>(static_cast<double>(angular_vel_param[2]))};
            }
        }
    }

    AugmentationParams LidarAugmenterNode::parseAugmentationParams(const std::string &scenario_name)
    {
        AugmentationParams params;

        // Read directly from ROS parameters (already loaded from YAML)
        std::string param_prefix = "scenarios/" + scenario_name + "/";

        // Check if scenario exists in ROS parameter server
        if (!private_nh_.hasParam(param_prefix + "dropout/enabled"))
        {
            ROS_WARN_STREAM("Could not load scenario '" << scenario_name << "' from ROS parameters. Using defaults.");
            return getDefaultAugmentationParams(scenario_name);
        }

        ROS_INFO_STREAM("Loading scenario '" << scenario_name << "' from ROS parameters");

        // Read dropout parameters
        private_nh_.param(param_prefix + "dropout/enabled", params.dropout.enabled, false);
        private_nh_.param(param_prefix + "dropout/rate", params.dropout.rate, 0.15f);
        private_nh_.param(param_prefix + "dropout/mode", params.dropout.mode, std::string("random"));

        // Read FOV reduction parameters
        private_nh_.param(param_prefix + "fov_reduction/enabled", params.fov_reduction.enabled, false);
        private_nh_.param(param_prefix + "fov_reduction/horizontal", params.fov_reduction.horizontal, 0.0f);
        private_nh_.param(param_prefix + "fov_reduction/vertical", params.fov_reduction.vertical, 0.0f);

        // Read noise parameters
        private_nh_.param(param_prefix + "noise/enabled", params.noise.enabled, false);
        private_nh_.param(param_prefix + "noise/gaussian_std", params.noise.gaussian_std, 0.02f);
        private_nh_.param(param_prefix + "noise/outlier_rate", params.noise.outlier_rate, 0.01f);
        private_nh_.param(param_prefix + "noise/outlier_std", params.noise.outlier_std, 0.5f);

        // Read occlusion parameters
        private_nh_.param(param_prefix + "occlusion/enabled", params.occlusion.enabled, false);
        private_nh_.param(param_prefix + "occlusion/distance_threshold", params.occlusion.distance_threshold, 50.0f);
        private_nh_.param(param_prefix + "occlusion/num_objects", params.occlusion.random_patches.count, 3);
        private_nh_.param(param_prefix + "occlusion/object_size", params.occlusion.random_patches.size, 1.5f);

        // Read sparse scan parameters
        private_nh_.param(param_prefix + "sparse_scan/enabled", params.sparse_scan.enabled, false);
        private_nh_.param(param_prefix + "sparse_scan/skip_factor", params.sparse_scan.factor, 2);

        // Read motion distortion parameters
        private_nh_.param(param_prefix + "motion_distortion/enabled", params.motion_distortion.enabled, false);
        double linear_vel_scalar, angular_vel_scalar;
        private_nh_.param(param_prefix + "motion_distortion/linear_velocity", linear_vel_scalar, 0.5);
        private_nh_.param(param_prefix + "motion_distortion/angular_velocity", angular_vel_scalar, 0.1);

        // Set the scalar values to the first component (assuming forward motion and yaw rotation)
        params.motion_distortion.linear_velocity[0] = static_cast<float>(linear_vel_scalar);
        params.motion_distortion.linear_velocity[1] = 0.0f;
        params.motion_distortion.linear_velocity[2] = 0.0f;

        params.motion_distortion.angular_velocity[0] = 0.0f;
        params.motion_distortion.angular_velocity[1] = 0.0f;
        params.motion_distortion.angular_velocity[2] = static_cast<float>(angular_vel_scalar);

        ROS_INFO_STREAM("Successfully loaded scenario '" << scenario_name << "' from ROS parameters");
        return params;
    }

    bool LidarAugmenterNode::parseScenarioYAML(const YAML::Node &scenario, AugmentationParams &params)
    {
        try
        {
            // Parse all augmentation parameters with safe defaults

            if (scenario["dropout"])
            {
                const auto &dropout = scenario["dropout"];
                params.dropout.enabled = dropout["enabled"].as<bool>(false);
                params.dropout.rate = dropout["rate"].as<float>(0.3f);
                params.dropout.mode = dropout["mode"].as<std::string>("random");
            }

            if (scenario["fov_reduction"])
            {
                const auto &fov = scenario["fov_reduction"];
                params.fov_reduction.enabled = fov["enabled"].as<bool>(false);
                params.fov_reduction.horizontal = fov["horizontal"].as<float>(0.2f);
                params.fov_reduction.vertical = fov["vertical"].as<float>(0.2f);
            }

            if (scenario["noise"])
            {
                const auto &noise = scenario["noise"];
                params.noise.enabled = noise["enabled"].as<bool>(false);
                params.noise.gaussian_std = noise["gaussian_std"].as<float>(0.02f);
                params.noise.outlier_rate = noise["outlier_rate"].as<float>(0.01f);
                params.noise.outlier_std = noise["outlier_std"].as<float>(0.5f);
            }

            if (scenario["occlusion"])
            {
                const auto &occlusion = scenario["occlusion"];
                params.occlusion.enabled = occlusion["enabled"].as<bool>(false);
                params.occlusion.distance_threshold = occlusion["distance_threshold"].as<float>(50.0f);

                if (occlusion["random_patches"])
                {
                    const auto &patches = occlusion["random_patches"];
                    params.occlusion.random_patches.count = patches["count"].as<int>(3);
                    params.occlusion.random_patches.size = patches["size"].as<float>(1.5f);
                }
            }

            if (scenario["sparse_scan"])
            {
                const auto &sparse = scenario["sparse_scan"];
                params.sparse_scan.enabled = sparse["enabled"].as<bool>(false);
                params.sparse_scan.factor = sparse["factor"].as<int>(2);
            }

            if (scenario["motion_distortion"])
            {
                const auto &motion = scenario["motion_distortion"];
                params.motion_distortion.enabled = motion["enabled"].as<bool>(false);

                if (motion["linear_velocity"] && motion["linear_velocity"].size() >= 3)
                {
                    params.motion_distortion.linear_velocity = motion["linear_velocity"].as<std::vector<float>>();
                }

                if (motion["angular_velocity"] && motion["angular_velocity"].size() >= 3)
                {
                    params.motion_distortion.angular_velocity = motion["angular_velocity"].as<std::vector<float>>();
                }
            }

            return true;
        }
        catch (const YAML::Exception &e)
        {
            ROS_WARN_STREAM("Error parsing scenario YAML: " << e.what());
            return false;
        }
    }

    AugmentationParams LidarAugmenterNode::getDefaultAugmentationParams(const std::string &scenario_name)
    {
        AugmentationParams params;

        // Provide sensible defaults based on scenario name
        if (scenario_name == "light")
        {
            params.noise.enabled = true;
            params.noise.gaussian_std = 0.01f;
        }
        else if (scenario_name == "moderate")
        {
            params.dropout.enabled = true;
            params.dropout.rate = 0.2f;
            params.noise.enabled = true;
            params.noise.gaussian_std = 0.02f;
        }
        else if (scenario_name == "heavy")
        {
            params.dropout.enabled = true;
            params.dropout.rate = 0.3f;
            params.fov_reduction.enabled = true;
            params.fov_reduction.horizontal = 0.1f;
            params.fov_reduction.vertical = 0.1f;
            params.noise.enabled = true;
            params.noise.gaussian_std = 0.03f;
            params.occlusion.enabled = true;
        }
        else if (scenario_name == "extreme")
        {
            params.dropout.enabled = true;
            params.dropout.rate = 0.4f;
            params.fov_reduction.enabled = true;
            params.fov_reduction.horizontal = 0.2f;
            params.fov_reduction.vertical = 0.2f;
            params.noise.enabled = true;
            params.noise.gaussian_std = 0.05f;
            params.occlusion.enabled = true;
            params.sparse_scan.enabled = true;
            params.sparse_scan.factor = 2;
        }

        return params;
    }

    void LidarAugmenterNode::logAugmentationConfig()
    {
        ROS_INFO("=== Final Augmentation Configuration ===");
        ROS_INFO_STREAM("  Dropout: " << (aug_params_.dropout.enabled ? "ENABLED" : "DISABLED"));
        if (aug_params_.dropout.enabled)
        {
            ROS_INFO_STREAM("    Rate: " << aug_params_.dropout.rate << ", Mode: " << aug_params_.dropout.mode);
        }

        ROS_INFO_STREAM("  FOV Reduction: " << (aug_params_.fov_reduction.enabled ? "ENABLED" : "DISABLED"));
        if (aug_params_.fov_reduction.enabled)
        {
            ROS_INFO_STREAM("    Horizontal: " << aug_params_.fov_reduction.horizontal
                                               << ", Vertical: " << aug_params_.fov_reduction.vertical);
        }

        ROS_INFO_STREAM("  Noise: " << (aug_params_.noise.enabled ? "ENABLED" : "DISABLED"));
        if (aug_params_.noise.enabled)
        {
            ROS_INFO_STREAM("    Gaussian STD: " << aug_params_.noise.gaussian_std
                                                 << ", Outlier Rate: " << aug_params_.noise.outlier_rate
                                                 << ", Outlier STD: " << aug_params_.noise.outlier_std);
        }

        ROS_INFO_STREAM("  Occlusion: " << (aug_params_.occlusion.enabled ? "ENABLED" : "DISABLED"));
        if (aug_params_.occlusion.enabled)
        {
            ROS_INFO_STREAM("    Distance Threshold: " << aug_params_.occlusion.distance_threshold << "m"
                                                       << ", Patches: " << aug_params_.occlusion.random_patches.count
                                                       << ", Size: " << aug_params_.occlusion.random_patches.size << "m");
        }

        ROS_INFO_STREAM("  Sparse Scan: " << (aug_params_.sparse_scan.enabled ? "ENABLED" : "DISABLED"));
        if (aug_params_.sparse_scan.enabled)
        {
            ROS_INFO_STREAM("    Sparsity Factor: " << aug_params_.sparse_scan.factor);
        }

        ROS_INFO_STREAM("  Motion Distortion: " << (aug_params_.motion_distortion.enabled ? "ENABLED" : "DISABLED"));
        if (aug_params_.motion_distortion.enabled)
        {
            if (use_imu_ && !imu_topics_.empty())
            {
                ROS_INFO("    Mode: IMU-based motion estimation");
            }
            else
            {
                ROS_INFO_STREAM("    Mode: Static parameters");
                ROS_INFO_STREAM("    Linear Velocity: ["
                                << aug_params_.motion_distortion.linear_velocity[0] << ", "
                                << aug_params_.motion_distortion.linear_velocity[1] << ", "
                                << aug_params_.motion_distortion.linear_velocity[2] << "] m/s");
                ROS_INFO_STREAM("    Angular Velocity: ["
                                << aug_params_.motion_distortion.angular_velocity[0] << ", "
                                << aug_params_.motion_distortion.angular_velocity[1] << ", "
                                << aug_params_.motion_distortion.angular_velocity[2] << "] rad/s");
            }
        }
        ROS_INFO("========================================");
    }

    void LidarAugmenterNode::setupPublishers()
    {
        publishers_.clear();

        for (const auto &[sensor_name, input_topic] : input_topics_)
        {
            std::string output_topic = input_topic + output_suffix_;

            ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 50);
            publishers_[sensor_name] = pub;

            ROS_DEBUG_STREAM("Publisher: " << sensor_name << " -> " << output_topic);
        }

        ROS_INFO_STREAM("Created " << publishers_.size() << " publishers");

        // Delayed IMU republishers
        if (use_imu_)
        {
            for (const auto &[sensor_name, imu_topic] : imu_topics_)
            {
                std::string delayed_imu_topic = imu_topic + output_suffix_;
                imu_delayed_publishers_[sensor_name] =
                    nh_.advertise<sensor_msgs::Imu>(delayed_imu_topic, 200);
                ROS_INFO_STREAM("Delayed IMU publisher: " << sensor_name
                                                          << " -> " << delayed_imu_topic);
            }
        }

        if (publish_statistics_)
        {
            stats_publisher_ = nh_.advertise<std_msgs::String>("/lidar_augmentation/statistics", 10);
            ROS_INFO("Created statistics publisher -> /lidar_augmentation/statistics");
        }
    }

    void LidarAugmenterNode::setupSubscribers()
    {
        subscribers_.clear();
        imu_subscribers_.clear();

        for (const auto &[sensor_name, input_topic] : input_topics_)
        {
            ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(input_topic, 50,
                                                                          boost::bind(&LidarAugmenterNode::pointCloudCallback, this, _1, sensor_name));
            subscribers_[sensor_name] = sub;

            ROS_DEBUG_STREAM("Subscriber created: " << sensor_name << " <- " << input_topic);
        }

        // IMU subscribers
        if (use_imu_)
        {
            for (const auto &[sensor_name, imu_topic] : imu_topics_)
            {
                ros::Subscriber imu_sub = nh_.subscribe<sensor_msgs::Imu>(
                    imu_topic, 100,
                    boost::bind(&LidarAugmenterNode::imuCallback, this, _1));

                imu_subscribers_[sensor_name] = imu_sub;
                ROS_DEBUG_STREAM("IMU Subscriber: " << sensor_name << " <- " << imu_topic);
            }
        }

        ROS_INFO_STREAM("Created " << subscribers_.size() << " point cloud subscribers");
        if (use_imu_)
        {
            ROS_INFO_STREAM("Created " << imu_subscribers_.size() << " IMU subscribers");
        }
    }

    void LidarAugmenterNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                                const std::string &sensor)

    {

        // Capture wall-clock arrival time FIRST, before any processing
        const ros::Time wall_receive_time = ros::Time::now();

        auto start_time = std::chrono::high_resolution_clock::now();

        try
        {
            // Detect sensor type
            std::string sensor_type = processor_->detectSensorType(msg);

            ROS_DEBUG_STREAM("Processing " << sensor << " (" << sensor_type << ") with "
                                           << msg->width << " points");

            // Process based on detected sensor type
            if (sensor_type == "ouster")
            {
                processPointCloud<OusterPoint>(msg, sensor, sensor_type, wall_receive_time);
            }
            else if (sensor_type == "livox_mid360" || sensor_type == "livox_avia")
            {
                processPointCloud<LivoxPoint>(msg, sensor, sensor_type, wall_receive_time);
            }
            else
            {
                // Generic point cloud processing
                processPointCloud<pcl::PointXYZI>(msg, sensor, sensor_type, wall_receive_time);
            }

            // Performance logging
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            ROS_INFO_THROTTLE(2.0, "[%s] Processed in %ld ms | %u points",
                              sensor.c_str(), duration.count(), msg->width);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("Error processing " << sensor << " point cloud: " << e.what());
        }
    }

    template <typename PointT>
    void LidarAugmenterNode::processPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                               const std::string &sensor,
                                               const std::string &sensor_type,
                                               const ros::Time &wall_receive_time)
    {

        // Extract points and fields
        typename pcl::PointCloud<PointT>::Ptr cloud;
        std::unordered_map<std::string, std::vector<float>> fields;
        std::vector<std::string> field_names;

        processor_->extractPointsAndFields<PointT>(msg, cloud, fields, field_names);

        auto start_time = std::chrono::high_resolution_clock::now();

        if (!cloud || cloud->empty())
        {
            ROS_WARN_STREAM("Extracted empty point cloud from " << sensor);
            return;
        }

        // Store original fields for timestamp preservation
        auto original_fields = fields;
        typename pcl::PointCloud<PointT>::Ptr augmented_cloud = cloud;

        // Apply augmentations in sequence

        // 1. Dropout augmentation
        if (aug_params_.dropout.enabled)
        {
            std::vector<bool> dropout_mask;

            if (aug_params_.dropout.mode == "structured")
            {
                std::tie(augmented_cloud, dropout_mask) = augmenter_->structuredDropout<PointT>(
                    augmented_cloud, "ring", aug_params_.dropout.rate);
            }
            else
            {
                std::tie(augmented_cloud, dropout_mask) = augmenter_->randomDropout<PointT>(
                    augmented_cloud, aug_params_.dropout.rate);
            }

            // Update fields based on dropout mask
            if (!dropout_mask.empty())
            {
                updateFieldsWithMask(fields, dropout_mask);
            }
        }

        // 2. FOV reduction
        if (aug_params_.fov_reduction.enabled)
        {
            std::unordered_map<std::string, float> fov_params = {
                {"horizontal", aug_params_.fov_reduction.horizontal},
                {"vertical", aug_params_.fov_reduction.vertical}};

            augmented_cloud = augmenter_->reduceFOV<PointT>(augmented_cloud, fov_params);

            // Update fields to match reduced cloud size
            updateFieldsWithCloud<PointT>(fields, augmented_cloud);
        }

        // 3. Noise injection
        if (aug_params_.noise.enabled)
        {
            std::unordered_map<std::string, float> noise_params = {
                {"gaussian_std", aug_params_.noise.gaussian_std},
                {"outlier_rate", aug_params_.noise.outlier_rate},
                {"outlier_std", aug_params_.noise.outlier_std}};

            augmented_cloud = augmenter_->addNoise<PointT>(augmented_cloud, noise_params);

            // Update XYZ fields with noisy coordinates
            updateXYZFields<PointT>(fields, augmented_cloud);
        }

        // 4. Motion distortion
        if (aug_params_.motion_distortion.enabled)
        {
            MotionParams motion_params;

            if (use_imu_ && !imu_topics_.empty())
            {
                // Use IMU-based motion estimation
                double start_time = msg->header.stamp.toSec() - 0.1; // 100ms before scan
                double end_time = msg->header.stamp.toSec();

                motion_params = imu_sync_->estimateMotionParams(start_time, end_time);
            }
            else
            {
                // Use configured motion parameters
                motion_params.linear_velocity.assign(
                    aug_params_.motion_distortion.linear_velocity.begin(),
                    aug_params_.motion_distortion.linear_velocity.end());
                // for angular_velocity
                motion_params.angular_velocity.assign(
                    aug_params_.motion_distortion.angular_velocity.begin(),
                    aug_params_.motion_distortion.angular_velocity.end());
            }

            // Extract timestamps for motion distortion
            std::vector<uint32_t> timestamps = processor_->getTimestamps<PointT>(fields, sensor_type);

            if (!timestamps.empty())
            {
                augmented_cloud = augmenter_->simulateMotionDistortion<PointT>(
                    augmented_cloud, timestamps, motion_params);

                // Update XYZ fields with distorted coordinates
                updateXYZFields<PointT>(fields, augmented_cloud);
            }
        }

        // 5. Occlusion simulation
        if (aug_params_.occlusion.enabled)
        {
            std::unordered_map<std::string, float> occlusion_params = {
                {"distance_threshold", aug_params_.occlusion.distance_threshold},
                {"random_patches_count", static_cast<float>(aug_params_.occlusion.random_patches.count)},
                {"random_patches_size", aug_params_.occlusion.random_patches.size}};

            augmented_cloud = augmenter_->simulateOcclusion<PointT>(augmented_cloud, occlusion_params);

            // Update fields to match occluded cloud size
            updateFieldsWithCloud<PointT>(fields, augmented_cloud);
        }

        // 6. Sparse scan pattern
        if (aug_params_.sparse_scan.enabled)
        {
            augmented_cloud = augmenter_->sparseScanPattern<PointT>(
                augmented_cloud, aug_params_.sparse_scan.factor);

            // Update fields to match sparse cloud size
            updateFieldsWithCloud<PointT>(fields, augmented_cloud);
        }

        // Preserve original timestamps
        fields = processor_->preserveOriginalTimestamps<PointT>(original_fields, fields);

        // Create augmented message
        sensor_msgs::PointCloud2 augmented_msg = processor_->createAugmentedMsg<PointT>(
            msg->header, augmented_cloud, fields, msg->fields);

        // Buffer for delayed synchronized publishing
        if (use_imu_)
        {
            std::lock_guard<std::mutex> lock(sync_buffer_mutex_);
            BufferedCloud buffered;
            buffered.wall_receive_time = wall_receive_time;
            buffered.sensor = sensor;
            buffered.augmented_msg = augmented_msg;
            cloud_output_buffer_.push_back(buffered);
        }
        else
        {
            // No IMU → no sync needed, publish immediately as before
            if (publishers_.count(sensor))
            {
                publishers_[sensor].publish(augmented_msg);
            }
        }

        ROS_DEBUG_STREAM("Published augmented " << sensor << " cloud with "
                                                << augmented_cloud->size() << " points");

        // publisher statistics
        if (publish_statistics_)
        {
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            publishStatistics<PointT>(msg, augmented_cloud, sensor, sensor_type, duration);
        }
    }

    void LidarAugmenterNode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        if (use_imu_ && imu_sync_)
        {
            // Feed IMU synchronizer for motion estimation
            imu_sync_->addIMUMsg(msg);
            ROS_DEBUG_THROTTLE(5.0, "IMU buffer size: %zu", imu_sync_->getBufferSize());

            // Buffer IMU message for delayed republishing
            {
                std::lock_guard<std::mutex> lock(sync_buffer_mutex_);
                BufferedIMU buffered;
                buffered.wall_receive_time = ros::Time::now();
                buffered.msg = *msg;
                imu_output_buffer_.push_back(buffered);
            }
        }
    }

    void LidarAugmenterNode::updateFieldsWithMask(
        std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<bool> &mask)
    {

        for (auto &[field_name, field_data] : fields)
        {
            std::vector<float> filtered_data;
            filtered_data.reserve(std::count(mask.begin(), mask.end(), true));

            for (size_t i = 0; i < mask.size() && i < field_data.size(); ++i)
            {
                if (mask[i])
                {
                    filtered_data.push_back(field_data[i]);
                }
            }

            field_data = std::move(filtered_data);
        }
    }

    std::string LidarAugmenterNode::getScenarioParametersJSON() const
    {
        std::ostringstream params;
        params << "{"
               << "\"dropout\":{\"enabled\":" << (aug_params_.dropout.enabled ? "true" : "false")
               << ",\"rate\":" << aug_params_.dropout.rate << "},"
               << "\"fov_reduction\":{\"enabled\":" << (aug_params_.fov_reduction.enabled ? "true" : "false")
               << ",\"horizontal\":" << aug_params_.fov_reduction.horizontal
               << ",\"vertical\":" << aug_params_.fov_reduction.vertical << "},"
               << "\"noise\":{\"enabled\":" << (aug_params_.noise.enabled ? "true" : "false")
               << ",\"gaussian_std\":" << aug_params_.noise.gaussian_std << "},"
               << "\"occlusion\":{\"enabled\":" << (aug_params_.occlusion.enabled ? "true" : "false")
               << ",\"distance_threshold\":" << aug_params_.occlusion.distance_threshold
               << ",\"num_objects\":" << aug_params_.occlusion.random_patches.count
               << ",\"object_size\":" << aug_params_.occlusion.random_patches.size << "},"
               << "\"sparse_scan\":{\"enabled\":" << (aug_params_.sparse_scan.enabled ? "true" : "false")
               << ",\"skip_factor\":" << aug_params_.sparse_scan.factor << "},"
               << "\"motion_distortion\":{\"enabled\":" << (aug_params_.motion_distortion.enabled ? "true" : "false")
               << ",\"linear_velocity\":" << aug_params_.motion_distortion.linear_velocity[0]
               << ",\"angular_velocity\":" << aug_params_.motion_distortion.angular_velocity[2] << "}"
               << "}";
        return params.str();
    }

    template <typename PointT>
    void LidarAugmenterNode::updateFieldsWithCloud(
        std::unordered_map<std::string, std::vector<float>> &fields,
        const typename pcl::PointCloud<PointT>::Ptr &cloud)
    {

        size_t new_size = cloud->size();

        // Resize all fields to match new cloud size
        for (auto &[field_name, field_data] : fields)
        {
            if (field_data.size() > new_size)
            {
                field_data.resize(new_size);
            }
            else if (field_data.size() < new_size)
            {
                // Pad with zeros if needed
                field_data.resize(new_size, 0.0f);
            }
        }

        // Update XYZ fields with current cloud coordinates
        updateXYZFields<PointT>(fields, cloud);
    }

    template <typename PointT>
    void LidarAugmenterNode::updateXYZFields(
        std::unordered_map<std::string, std::vector<float>> &fields,
        const typename pcl::PointCloud<PointT>::Ptr &cloud)
    {

        if (cloud->empty())
            return;

        // Update XYZ coordinates from cloud
        if (fields.count("x"))
        {
            fields["x"].resize(cloud->size());
            for (size_t i = 0; i < cloud->size(); ++i)
            {
                fields["x"][i] = (*cloud)[i].x;
            }
        }

        if (fields.count("y"))
        {
            fields["y"].resize(cloud->size());
            for (size_t i = 0; i < cloud->size(); ++i)
            {
                fields["y"][i] = (*cloud)[i].y;
            }
        }

        if (fields.count("z"))
        {
            fields["z"].resize(cloud->size());
            for (size_t i = 0; i < cloud->size(); ++i)
            {
                fields["z"][i] = (*cloud)[i].z;
            }
        }
    }

    void LidarAugmenterNode::syncFlushCallback(const ros::TimerEvent & /*event*/)
    {
        std::lock_guard<std::mutex> lock(sync_buffer_mutex_);
        ros::Time now = ros::Time::now();
        ros::Duration delay(sync_delay_sec_);

        // Flush IMU messages that have waited long enough
        while (!imu_output_buffer_.empty())
        {
            auto &front = imu_output_buffer_.front();
            if ((now - front.wall_receive_time) >= delay)
            {
                for (auto &[sensor_name, pub] : imu_delayed_publishers_)
                {
                    pub.publish(front.msg);
                }
                imu_output_buffer_.pop_front();
            }
            else
            {
                break; // remaining messages are newer, wait
            }
        }

        // Flush augmented LiDAR messages that have waited long enough
        while (!cloud_output_buffer_.empty())
        {
            auto &front = cloud_output_buffer_.front();
            if ((now - front.wall_receive_time) >= delay)
            {
                if (publishers_.count(front.sensor))
                {
                    publishers_[front.sensor].publish(front.augmented_msg);
                }
                cloud_output_buffer_.pop_front();
            }
            else
            {
                break; // remaining messages are newer, wait
            }
        }
    }

    void LidarAugmenterNode::run()
    {
        ROS_INFO("LiDAR Augmenter Node is running...");
        ROS_INFO("Configuration is completely flexible - works with any rosbag topic structure!");
        ROS_INFO("Press Ctrl+C to stop");

        ros::spin();

        ROS_INFO("LiDAR Augmenter Node shutting down");
    }

    // Explicit template instantiations for all methods
    template void LidarAugmenterNode::processPointCloud<OusterPoint>(
        const sensor_msgs::PointCloud2::ConstPtr &, const std::string &, const std::string &, const ros::Time &);

    template void LidarAugmenterNode::processPointCloud<LivoxPoint>(
        const sensor_msgs::PointCloud2::ConstPtr &, const std::string &, const std::string &, const ros::Time &);

    template void LidarAugmenterNode::processPointCloud<pcl::PointXYZI>(
        const sensor_msgs::PointCloud2::ConstPtr &, const std::string &, const std::string &, const ros::Time &);

    template void LidarAugmenterNode::updateFieldsWithCloud<OusterPoint>(
        std::unordered_map<std::string, std::vector<float>> &, const pcl::PointCloud<OusterPoint>::Ptr &);

    template void LidarAugmenterNode::updateFieldsWithCloud<LivoxPoint>(
        std::unordered_map<std::string, std::vector<float>> &, const pcl::PointCloud<LivoxPoint>::Ptr &);

    template void LidarAugmenterNode::updateFieldsWithCloud<pcl::PointXYZI>(
        std::unordered_map<std::string, std::vector<float>> &, const pcl::PointCloud<pcl::PointXYZI>::Ptr &);

    template void LidarAugmenterNode::updateXYZFields<OusterPoint>(
        std::unordered_map<std::string, std::vector<float>> &, const pcl::PointCloud<OusterPoint>::Ptr &);

    template void LidarAugmenterNode::updateXYZFields<LivoxPoint>(
        std::unordered_map<std::string, std::vector<float>> &, const pcl::PointCloud<LivoxPoint>::Ptr &);

    template void LidarAugmenterNode::updateXYZFields<pcl::PointXYZI>(
        std::unordered_map<std::string, std::vector<float>> &, const pcl::PointCloud<pcl::PointXYZI>::Ptr &);

    template void LidarAugmenterNode::publishStatistics<OusterPoint>(
        const sensor_msgs::PointCloud2::ConstPtr &, const pcl::PointCloud<OusterPoint>::Ptr &,
        const std::string &, const std::string &, const std::chrono::milliseconds &);

    template void LidarAugmenterNode::publishStatistics<LivoxPoint>(
        const sensor_msgs::PointCloud2::ConstPtr &, const pcl::PointCloud<LivoxPoint>::Ptr &,
        const std::string &, const std::string &, const std::chrono::milliseconds &);

    template void LidarAugmenterNode::publishStatistics<pcl::PointXYZI>(
        const sensor_msgs::PointCloud2::ConstPtr &, const pcl::PointCloud<pcl::PointXYZI>::Ptr &,
        const std::string &, const std::string &, const std::chrono::milliseconds &);

    template <typename PointT>
    void LidarAugmenterNode::publishStatistics(
        const sensor_msgs::PointCloud2::ConstPtr &original_msg,
        const typename pcl::PointCloud<PointT>::Ptr &augmented_cloud,
        const std::string &sensor,
        const std::string &sensor_type,
        const std::chrono::milliseconds &processing_time)
    {
        // THROTTLING
        ros::Time now = ros::Time::now();

        // Check if enough time has passed since last statistics publish for this sensor
        auto &last_time = last_stats_publish_time_[sensor];
        double time_since_last = last_time.isZero() ? 999.0 : (now - last_time).toSec();

        if (time_since_last < (1.0 / stats_publish_rate_))
        {
            return; // Skip publishing, not enough time passed
        }

        // Update last publish time
        last_time = now;

        ROS_INFO_STREAM("Publishing statistics for sensor: " << sensor
                                                             << " | Original Points: " << original_msg->width
                                                             << " | Augmented Points: " << augmented_cloud->size()
                                                             << " | Processing Time: " << processing_time.count() << " ms");

        // Calculate basic statistics
        uint32_t original_points = (original_msg->width * original_msg->height > 0)
                                       ? original_msg->width * original_msg->height
                                       : original_msg->data.size() / original_msg->point_step;
        uint32_t augmented_points = augmented_cloud->size();
        float point_reduction_percent = 100.0f * (1.0f - static_cast<float>(augmented_points) / original_points);

        // Create JSON statistics message
        std::ostringstream json_stream;
        json_stream << "{"
                    << "\"sensor\":\"" << sensor << "\","
                    << "\"sensor_type\":\"" << sensor_type << "\","
                    << "\"timestamp\":" << original_msg->header.stamp.toSec() << ","
                    << "\"original_points\":" << original_points << ","
                    << "\"augmented_points\":" << augmented_points << ","
                    << "\"point_reduction_percent\":" << std::fixed << std::setprecision(2) << point_reduction_percent << ","
                    << "\"processing_time_ms\":" << processing_time.count() << ","
                    << "\"active_scenario\":\"" << getActiveScenario() << "\","
                    << "\"scenario_parameters\":" << getScenarioParametersJSON()
                    << "}";

        std_msgs::String stats_msg;
        stats_msg.data = json_stream.str();
        stats_publisher_.publish(stats_msg);

        ROS_DEBUG_STREAM("Published statistics: " << stats_msg.data);
    }
} // namespace lidar_augmentation
