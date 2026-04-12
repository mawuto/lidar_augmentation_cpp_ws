// include/lidar_augmentation/augmentation_methods.h
#ifndef LIDAR_AUGMENTATION_AUGMENTATION_METHODS_H
#define LIDAR_AUGMENTATION_AUGMENTATION_METHODS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <random>
#include <unordered_map>
#include <vector>
#include <string>
#include <tuple>
#include <cstdint>
#include "point_cloud_processor.h"
#include "imu_synchronizer.h"

namespace lidar_augmentation
{

    // Forward declarations for parameter structures
    struct NoiseParams
    {
        float gaussian_noise = 0.02f;
        float outlier_ratio = 0.01f;
        float intensity_noise = 0.1f;
    };

    struct OcclusionParams
    {
        float occlusion_ratio = 0.1f;
        float min_distance = 1.0f;
        float max_distance = 50.0f;
        std::string pattern = "random";
    };

    class LidarAugmenter
    {
    public:
        LidarAugmenter();
        ~LidarAugmenter() = default;

        // Dropout methods
        template <typename PointT>
        std::tuple<typename pcl::PointCloud<PointT>::Ptr, std::vector<bool>>
        randomDropout(typename pcl::PointCloud<PointT>::Ptr &cloud, float rate = 0.3f);

        template <typename PointT>
        std::tuple<typename pcl::PointCloud<PointT>::Ptr, std::vector<bool>>
        structuredDropout(typename pcl::PointCloud<PointT>::Ptr &cloud,
                          const std::string &pattern, float rate = 0.3f);

        // FOV reduction
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr reduceFOV(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            const std::unordered_map<std::string, float> &fov_reduction);

        // Noise injection
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr addNoise(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            const std::unordered_map<std::string, float> &noise_params);

        // Alternative overload for NoiseParams struct
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr addNoise(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            const NoiseParams &params);

        // Motion distortion
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr simulateMotionDistortion(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            const std::vector<uint32_t> &timestamps,
            const MotionParams &motion_params);

        // Occlusion simulation
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr simulateOcclusion(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            const std::unordered_map<std::string, float> &occlusion_params);

        // Alternative overload for OcclusionParams struct
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr simulateOcclusion(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            const OcclusionParams &params);

        // Sparse scan simulation
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr sparseScanPattern(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            int sparsity_factor = 2);

        // Utility methods from error analysis
        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr filterByDistance(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            float min_distance, float max_distance);

        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr filterByIntensity(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            float min_intensity, float max_intensity);

    private:
        std::mt19937 rng_;
        std::uniform_real_distribution<float> uniform_dist_;
        std::normal_distribution<float> normal_dist_;

        // Helper methods
        template <typename PointT>
        float calculateDistance(const PointT &point);

        template <typename PointT>
        float calculateAzimuthAngle(const PointT &point);

        template <typename PointT>
        float calculateElevationAngle(const PointT &point);

        // Private helper methods
        template <typename PointT>
        bool isPointInFOV(const PointT &point,
                          float azimuth_min, float azimuth_max,
                          float elevation_min, float elevation_max);

        template <typename PointT>
        void addGaussianNoise(PointT &point, float noise_std);

        template <typename PointT>
        void addOutlierNoise(PointT &point, float outlier_probability);

        template <typename PointT>
        typename pcl::PointCloud<PointT>::Ptr applyMotionTransform(
            typename pcl::PointCloud<PointT>::Ptr &cloud,
            const std::vector<uint32_t> &timestamps,
            const MotionParams &motion_params);

        template <typename PointT>
        std::vector<bool> createRingBasedDropoutMask(typename pcl::PointCloud<PointT>::Ptr &cloud, float rate);

        template <typename PointT>
        std::vector<bool> createSectorBasedDropoutMask(typename pcl::PointCloud<PointT>::Ptr &cloud, float rate);

        template <typename PointT>
        std::vector<bool> createDistanceBasedDropoutMask(typename pcl::PointCloud<PointT>::Ptr &cloud, float rate);

        template <typename PointT>
        std::vector<bool> createCheckerboardDropoutMask(typename pcl::PointCloud<PointT>::Ptr &cloud, float rate);

        template <typename PointT>
        void extractRingInfo(typename pcl::PointCloud<PointT>::Ptr &cloud, std::vector<int> &rings);

        Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f &v);
    };

} // namespace lidar_augmentation

#endif // LIDAR_AUGMENTATION_AUGMENTATION_METHODS_H
