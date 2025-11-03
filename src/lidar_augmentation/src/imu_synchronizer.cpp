// src/cpp/imu_synchronizer.cpp
#include "lidar_augmentation/imu_synchronizer.h"
#include <ros/console.h>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace lidar_augmentation
{

    IMUSynchronizer::IMUSynchronizer(size_t buffer_size)
        : buffer_size_(buffer_size)
    {
        imu_buffer_.clear();
        last_imu_.reset();
        ROS_DEBUG_STREAM("IMU Synchronizer initialized with buffer size: " << buffer_size);
    }

    void IMUSynchronizer::addIMUMsg(const sensor_msgs::Imu::ConstPtr &msg)
    {
        // Convert ROS IMU message to internal IMU data structure
        IMUData imu_data;
        imu_data.timestamp = msg->header.stamp.toSec();

        // Extract linear acceleration
        imu_data.linear_acceleration = Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);

        // Extract angular velocity
        imu_data.angular_velocity = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        // Validate IMU data
        if (!isValidIMUData(imu_data))
        {
            ROS_WARN_STREAM("Received invalid IMU data at timestamp " << std::fixed << imu_data.timestamp);
            return;
        }

        // Add to circular buffer with size management
        if (imu_buffer_.size() >= buffer_size_)
        {
            imu_buffer_.pop_front(); // Remove oldest entry
        }

        // Insert in chronological order (maintain sorted buffer)
        auto insert_pos = std::upper_bound(
            imu_buffer_.begin(),
            imu_buffer_.end(),
            imu_data,
            [](const IMUData &a, const IMUData &b)
            {
                return a.timestamp < b.timestamp;
            });

        imu_buffer_.insert(insert_pos, imu_data);
        last_imu_ = imu_data;

        ROS_DEBUG_THROTTLE(1.0, "IMU buffer size: %zu, latest timestamp: %.6f",
                           imu_buffer_.size(), imu_data.timestamp);
    }

    std::optional<IMUData> IMUSynchronizer::getMotionAtTimestamp(double timestamp)
    {
        if (imu_buffer_.empty())
        {
            ROS_DEBUG_STREAM("IMU buffer is empty, cannot get motion at timestamp " << timestamp);
            return std::nullopt;
        }

        // Find the closest IMU measurements for interpolation
        auto lower_bound = std::lower_bound(
            imu_buffer_.begin(),
            imu_buffer_.end(),
            timestamp,
            [](const IMUData &imu_data, double ts)
            {
                return imu_data.timestamp < ts;
            });

        // Case 1: Timestamp is before all IMU data
        if (lower_bound == imu_buffer_.begin())
        {
            ROS_DEBUG_STREAM("Timestamp " << timestamp << " is before all IMU data, using first measurement");
            return imu_buffer_.front();
        }

        // Case 2: Timestamp is after all IMU data
        if (lower_bound == imu_buffer_.end())
        {
            ROS_DEBUG_STREAM("Timestamp " << timestamp << " is after all IMU data, using last measurement");
            return imu_buffer_.back();
        }

        // Case 3: Interpolation between two IMU measurements
        auto upper_bound = lower_bound;
        --lower_bound; // Get the previous element

        const IMUData &imu1 = *lower_bound; // Earlier measurement
        const IMUData &imu2 = *upper_bound; // Later measurement

        // Linear interpolation factor
        double dt = imu2.timestamp - imu1.timestamp;
        if (dt <= 0.0)
        {
            ROS_WARN_STREAM("Invalid time difference for interpolation: " << dt);
            return imu1; // Return earlier measurement
        }

        double alpha = (timestamp - imu1.timestamp) / dt;
        alpha = std::clamp(alpha, 0.0, 1.0); // Ensure alpha is in [0, 1]

        // Interpolate IMU data
        IMUData interpolated_imu;
        interpolated_imu.timestamp = timestamp;
        interpolated_imu.linear_acceleration = (1.0 - alpha) * imu1.linear_acceleration + alpha * imu2.linear_acceleration;
        interpolated_imu.angular_velocity = (1.0 - alpha) * imu1.angular_velocity + alpha * imu2.angular_velocity;

        ROS_DEBUG_STREAM("Interpolated IMU data at timestamp " << timestamp
                                                               << " between " << imu1.timestamp << " and " << imu2.timestamp
                                                               << " (alpha = " << alpha << ")");

        return interpolated_imu;
    }

    MotionParams IMUSynchronizer::estimateMotionParams(double start_time, double end_time)
    {
        MotionParams motion_params;
        motion_params.linear_velocity = {0.0, 0.0, 0.0};
        motion_params.angular_velocity = {0.0, 0.0, 0.0};

        if (imu_buffer_.empty())
        {
            ROS_WARN("IMU buffer is empty, cannot estimate motion parameters");
            return motion_params;
        }

        if (start_time >= end_time)
        {
            ROS_WARN_STREAM("Invalid time range: start_time (" << start_time
                                                               << ") >= end_time (" << end_time << ")");
            return motion_params;
        }

        // Find IMU measurements within the time range
        std::vector<IMUData> relevant_measurements;

        for (const auto &imu_data : imu_buffer_)
        {
            if (imu_data.timestamp >= start_time && imu_data.timestamp <= end_time)
            {
                relevant_measurements.push_back(imu_data);
            }
        }

        if (relevant_measurements.empty())
        {
            ROS_DEBUG_STREAM("No IMU measurements found in time range [" << start_time
                                                                         << ", " << end_time << "]");
            return motion_params;
        }

        // Method 1: Simple averaging
        Eigen::Vector3d avg_linear_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d avg_angular_vel = Eigen::Vector3d::Zero();

        for (const auto &imu_data : relevant_measurements)
        {
            avg_linear_acc += imu_data.linear_acceleration;
            avg_angular_vel += imu_data.angular_velocity;
        }

        size_t n_measurements = relevant_measurements.size();
        avg_linear_acc /= static_cast<double>(n_measurements);
        avg_angular_vel /= static_cast<double>(n_measurements);

        // Method 2: Integration for velocity estimation
        double dt = end_time - start_time;
        Eigen::Vector3d integrated_velocity = Eigen::Vector3d::Zero();

        if (n_measurements > 1)
        {
            // Trapezoidal integration
            for (size_t i = 0; i < n_measurements - 1; ++i)
            {
                const auto &imu1 = relevant_measurements[i];
                const auto &imu2 = relevant_measurements[i + 1];

                double dt_segment = imu2.timestamp - imu1.timestamp;
                if (dt_segment > 0.0)
                {
                    // Remove gravity component for better velocity estimation
                    Eigen::Vector3d acc1_no_gravity = removeGravity(imu1.linear_acceleration);
                    Eigen::Vector3d acc2_no_gravity = removeGravity(imu2.linear_acceleration);

                    // Trapezoidal rule
                    Eigen::Vector3d avg_acc = 0.5 * (acc1_no_gravity + acc2_no_gravity);
                    integrated_velocity += avg_acc * dt_segment;
                }
            }
        }
        else
        {
            // Single measurement case - use simple integration
            Eigen::Vector3d acc_no_gravity = removeGravity(avg_linear_acc);
            integrated_velocity = acc_no_gravity * dt;
        }

        // Fill motion parameters
        motion_params.linear_velocity = {
            integrated_velocity.x(),
            integrated_velocity.y(),
            integrated_velocity.z()};

        motion_params.angular_velocity = {
            avg_angular_vel.x(),
            avg_angular_vel.y(),
            avg_angular_vel.z()};

        ROS_DEBUG_STREAM("Estimated motion parameters from " << n_measurements
                                                             << " IMU measurements over " << dt << "s:");
        ROS_DEBUG_STREAM("  Linear velocity: [" << motion_params.linear_velocity[0]
                                                << ", " << motion_params.linear_velocity[1]
                                                << ", " << motion_params.linear_velocity[2] << "]");
        ROS_DEBUG_STREAM("  Angular velocity: [" << motion_params.angular_velocity[0]
                                                 << ", " << motion_params.angular_velocity[1]
                                                 << ", " << motion_params.angular_velocity[2] << "]");

        return motion_params;
    }

    // Private helper methods

    bool IMUSynchronizer::isValidIMUData(const IMUData &imu_data)
    {

        // Check timestamp validity
        if (imu_data.timestamp <= 0.0)
        {
            return false;
        }

        // Check for finite values in linear acceleration
        if (!std::isfinite(imu_data.linear_acceleration.x()) ||
            !std::isfinite(imu_data.linear_acceleration.y()) ||
            !std::isfinite(imu_data.linear_acceleration.z()))
        {
            return false;
        }

        // Check for finite values in angular velocity
        if (!std::isfinite(imu_data.angular_velocity.x()) ||
            !std::isfinite(imu_data.angular_velocity.y()) ||
            !std::isfinite(imu_data.angular_velocity.z()))
        {
            return false;
        }

        // Check for reasonable acceleration values (within ±50 m/s²)
        const double max_acceleration = 50.0; // m/s²
        if (imu_data.linear_acceleration.norm() > max_acceleration)
        {
            ROS_DEBUG_STREAM("IMU linear acceleration magnitude too high: "
                             << imu_data.linear_acceleration.norm() << " m/s²");
            return false;
        }

        // Check for reasonable angular velocity values (within ±50 rad/s)
        const double max_angular_velocity = 50.0; // rad/s
        if (imu_data.angular_velocity.norm() > max_angular_velocity)
        {
            ROS_DEBUG_STREAM("IMU angular velocity magnitude too high: "
                             << imu_data.angular_velocity.norm() << " rad/s");
            return false;
        }

        return true;
    }

    Eigen::Vector3d IMUSynchronizer::removeGravity(const Eigen::Vector3d &acceleration)
    {
        // Simple gravity removal - assumes IMU is roughly level
        // This is a basic implementation - can be enhanced with orientation data
        const double gravity_magnitude = 9.81; // m/s²

        // If acceleration is close to gravity magnitude and mostly in Z direction,
        // assume it's primarily gravity
        if (std::abs(acceleration.norm() - gravity_magnitude) < 2.0 &&
            std::abs(acceleration.z()) > 0.7 * gravity_magnitude)
        {
            // Remove gravity vector (assuming Z is up)
            return acceleration - Eigen::Vector3d(0.0, 0.0, -gravity_magnitude);
        }

        return acceleration; // Return original if not clearly gravity-dominated
    }

    void IMUSynchronizer::cleanupOldData(double current_time, double max_age_seconds)
    {
        // Remove IMU data older than max_age_seconds
        double cutoff_time = current_time - max_age_seconds;

        auto cutoff_iter = std::lower_bound(
            imu_buffer_.begin(),
            imu_buffer_.end(),
            cutoff_time,
            [](const IMUData &imu_data, double ts)
            {
                return imu_data.timestamp < ts;
            });

        size_t removed_count = std::distance(imu_buffer_.begin(), cutoff_iter);
        imu_buffer_.erase(imu_buffer_.begin(), cutoff_iter);

        if (removed_count > 0)
        {
            ROS_DEBUG_STREAM("Cleaned up " << removed_count << " old IMU measurements");
        }
    }

    std::pair<double, double> IMUSynchronizer::getTimestampRange() const
    {
        if (imu_buffer_.empty())
        {
            return {0.0, 0.0};
        }

        return {imu_buffer_.front().timestamp, imu_buffer_.back().timestamp};
    }

    size_t IMUSynchronizer::getBufferSize() const
    {
        return imu_buffer_.size();
    }

    void IMUSynchronizer::clearBuffer()
    {
        imu_buffer_.clear();
        last_imu_.reset();
        ROS_DEBUG("IMU buffer cleared");
    }

    bool IMUSynchronizer::hasRecentData(double current_time, double max_age_seconds) const
    {
        if (imu_buffer_.empty())
        {
            return false;
        }

        double latest_timestamp = imu_buffer_.back().timestamp;
        return (current_time - latest_timestamp) <= max_age_seconds;
    }

    std::vector<IMUData> IMUSynchronizer::getDataInRange(double start_time, double end_time) const
    {
        std::vector<IMUData> data_in_range;

        for (const auto &imu_data : imu_buffer_)
        {
            if (imu_data.timestamp >= start_time && imu_data.timestamp <= end_time)
            {
                data_in_range.push_back(imu_data);
            }
        }

        return data_in_range;
    }

    double IMUSynchronizer::getAverageFrequency() const
    {
        if (imu_buffer_.size() < 2)
        {
            return 0.0;
        }

        double time_span = imu_buffer_.back().timestamp - imu_buffer_.front().timestamp;
        if (time_span <= 0.0)
        {
            return 0.0;
        }

        return static_cast<double>(imu_buffer_.size() - 1) / time_span;
    }

} // namespace lidar_augmentation
