// include/lidar_augmentation/imu_synchronizer.h
#ifndef LIDAR_AUGMENTATION_IMU_SYNCHRONIZER_H
#define LIDAR_AUGMENTATION_IMU_SYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <deque>
#include <unordered_map>
#include <string>
#include <vector>
#include <optional>
#include <Eigen/Dense>

namespace lidar_augmentation
{

    struct IMUData
    {
        double timestamp;
        Eigen::Vector3d linear_acceleration;
        Eigen::Vector3d angular_velocity;
    };

    struct MotionParams
    {
        std::vector<double> linear_velocity;
        std::vector<double> angular_velocity;
    };

    class IMUSynchronizer
    {
    public:
        explicit IMUSynchronizer(size_t buffer_size = 1000);
        ~IMUSynchronizer() = default;

        // EXACT match to Python interface
        void addIMUMsg(const sensor_msgs::Imu::ConstPtr &msg);

        std::optional<IMUData> getMotionAtTimestamp(double timestamp);

        MotionParams estimateMotionParams(double start_time, double end_time);

        size_t buffer_size_;

        size_t getBufferSize() const;

    private:
        std::deque<IMUData> imu_buffer_;
        std::optional<IMUData> last_imu_;

        //  ADD MISSING FUNCTION DECLARATIONS
        bool isValidIMUData(const IMUData &imu_data);
        Eigen::Vector3d removeGravity(const Eigen::Vector3d &acceleration);
        void cleanupOldData(double current_time, double max_age_seconds);
        std::pair<double, double> getTimestampRange() const;
        void clearBuffer();
        bool hasRecentData(double current_time, double max_age_seconds) const;
        std::vector<IMUData> getDataInRange(double start_time, double end_time) const;
        double getAverageFrequency() const;
    };

} // namespace lidar_augmentation

#endif // LIDAR_AUGMENTATION_IMU_SYNCHRONIZER_H
