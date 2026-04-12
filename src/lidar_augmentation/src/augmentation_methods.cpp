// src/cpp/augmentation_methods.cpp
#include "lidar_augmentation/augmentation_methods.h"
#include <ros/console.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <chrono>

namespace lidar_augmentation
{

    LidarAugmenter::LidarAugmenter()
        : rng_(std::chrono::steady_clock::now().time_since_epoch().count()),
          uniform_dist_(0.0f, 1.0f),
          normal_dist_(0.0f, 1.0f)
    {
        ROS_DEBUG("LiDAR Augmenter initialized");
    }

    // =============================================================================
    // DROPOUT METHODS
    // =============================================================================

    template <typename PointT>
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, std::vector<bool>>
    LidarAugmenter::randomDropout(typename pcl::PointCloud<PointT>::Ptr &cloud, float rate)
    {
        auto result_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        std::vector<bool> mask;

        if (!cloud || cloud->empty())
        {
            ROS_WARN("Random dropout: input cloud is empty");
            return std::make_tuple(result_cloud, mask);
        }

        size_t original_size = cloud->size();
        mask.reserve(original_size);

        // rate is dropout rate (points to remove)
        float keep_rate = 1.0f - std::clamp(rate, 0.0f, 1.0f);

        if (keep_rate <= 0.0f)
        {
            ROS_WARN_STREAM("Random dropout rate too high (" << rate << "), returning empty cloud");
            mask.assign(original_size, false);
            return std::make_tuple(result_cloud, mask);
        }

        // Generate random mask
        for (size_t i = 0; i < original_size; ++i)
        {
            bool keep_point = uniform_dist_(rng_) < keep_rate;
            mask.push_back(keep_point);

            if (keep_point)
            {
                result_cloud->push_back((*cloud)[i]);
            }
        }

        // Copy header information
        result_cloud->header = cloud->header;
        result_cloud->height = 1;
        result_cloud->width = result_cloud->size();
        result_cloud->is_dense = cloud->is_dense;

        ROS_DEBUG_STREAM("Random dropout: " << original_size << " -> " << result_cloud->size()
                                            << " points (rate=" << rate << ", kept=" << keep_rate << ")");

        return std::make_tuple(result_cloud, mask);
    }

    template <typename PointT>
    std::tuple<typename pcl::PointCloud<PointT>::Ptr, std::vector<bool>>
    LidarAugmenter::structuredDropout(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                      const std::string &pattern, float rate)
    {
        auto result_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        std::vector<bool> mask;

        if (!cloud || cloud->empty())
        {
            ROS_WARN("Structured dropout: input cloud is empty");
            return std::make_tuple(result_cloud, mask);
        }

        size_t original_size = cloud->size();
        mask.assign(original_size, true); // Start with all points kept

        rate = std::clamp(rate, 0.0f, 1.0f);

        if (rate <= 0.0f)
        {
            // No dropout, return original cloud
            *result_cloud = *cloud;
            return std::make_tuple(result_cloud, mask);
        }

        // Structured dropout patterns
        if (pattern == "ring" || pattern == "line")
        {
            // Ring-based dropout for Ouster, line-based for Livox
            mask = createRingBasedDropoutMask<PointT>(cloud, rate);
        }
        else if (pattern == "sector")
        {
            // Sector-based dropout - remove angular sectors
            mask = createSectorBasedDropoutMask<PointT>(cloud, rate);
        }
        else if (pattern == "distance")
        {
            // Distance-based dropout - remove distance bands
            mask = createDistanceBasedDropoutMask<PointT>(cloud, rate);
        }
        else if (pattern == "checkerboard")
        {
            // Checkerboard pattern dropout
            mask = createCheckerboardDropoutMask<PointT>(cloud, rate);
            ;
        }
        else
        {
            ROS_WARN_STREAM("Unknown structured dropout pattern: " << pattern << ", using random");
            return randomDropout<PointT>(cloud, rate);
        }

        // Apply mask to create result cloud
        for (size_t i = 0; i < original_size; ++i)
        {
            if (mask[i])
            {
                result_cloud->push_back((*cloud)[i]);
            }
        }

        // Copy header information
        result_cloud->header = cloud->header;
        result_cloud->height = 1;
        result_cloud->width = result_cloud->size();
        result_cloud->is_dense = cloud->is_dense;

        ROS_DEBUG_STREAM("Structured dropout (" << pattern << "): " << original_size
                                                << " -> " << result_cloud->size() << " points (rate=" << rate << ")");

        return std::make_tuple(result_cloud, mask);
    }

    // =============================================================================
    // FOV REDUCTION
    // =============================================================================

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr LidarAugmenter::reduceFOV(
        typename pcl::PointCloud<PointT>::Ptr &cloud,
        const std::unordered_map<std::string, float> &fov_reduction)
    {

        auto result_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

        if (!cloud || cloud->empty())
        {
            ROS_WARN("FOV reduction: input cloud is empty");
            return result_cloud;
        }

        // FOV reduction parameters
        float horizontal_reduction = 0.0f;
        float vertical_reduction = 0.0f;

        if (fov_reduction.count("horizontal"))
        {
            horizontal_reduction = std::clamp(fov_reduction.at("horizontal"), 0.0f, 0.9f);
        }
        if (fov_reduction.count("vertical"))
        {
            vertical_reduction = std::clamp(fov_reduction.at("vertical"), 0.0f, 0.9f);
        }

        if (horizontal_reduction <= 0.0f && vertical_reduction <= 0.0f)
        {
            // No FOV reduction, return copy of original
            *result_cloud = *cloud;
            return result_cloud;
        }

        // Calculate FOV limits
        float horizontal_fov = M_PI * 2.0f * (1.0f - horizontal_reduction); // Remaining FOV
        float horizontal_limit = horizontal_fov / 2.0f;                     // ±limit

        float vertical_fov = M_PI * (1.0f - vertical_reduction); // Remaining FOV
        float vertical_limit = vertical_fov / 2.0f;              // ±limit

        size_t original_size = cloud->size();
        size_t kept_points = 0;

        for (const auto &point : *cloud)
        {
            bool keep_point = true;

            // Horizontal FOV check
            if (horizontal_reduction > 0.0f)
            {
                float azimuth = std::atan2(point.y, point.x);
                if (std::abs(azimuth) > horizontal_limit)
                {
                    keep_point = false;
                }
            }

            // Vertical FOV check
            if (keep_point && vertical_reduction > 0.0f)
            {
                float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                if (range > 0.0f)
                {
                    float elevation = std::asin(point.z / range);
                    if (std::abs(elevation) > vertical_limit)
                    {
                        keep_point = false;
                    }
                }
            }

            if (keep_point)
            {
                result_cloud->push_back(point);
                ++kept_points;
            }
        }

        // Copy header information
        result_cloud->header = cloud->header;
        result_cloud->height = 1;
        result_cloud->width = result_cloud->size();
        result_cloud->is_dense = cloud->is_dense;

        ROS_DEBUG_STREAM("FOV reduction: " << original_size << " -> " << kept_points
                                           << " points (h_red=" << horizontal_reduction << ", v_red=" << vertical_reduction << ")");

        return result_cloud;
    }

    // =============================================================================
    // NOISE INJECTION
    // =============================================================================

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr LidarAugmenter::addNoise(
        typename pcl::PointCloud<PointT>::Ptr &cloud,
        const std::unordered_map<std::string, float> &noise_params)
    {

        auto result_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>(*cloud));

        if (!cloud || cloud->empty())
        {
            ROS_WARN("Add noise: input cloud is empty");
            return result_cloud;
        }

        // noise parameters
        float gaussian_std = 0.0f;
        float outlier_rate = 0.0f;
        float outlier_std = 0.0f;

        if (noise_params.count("gaussian_std"))
        {
            gaussian_std = std::max(0.0f, noise_params.at("gaussian_std"));
        }
        if (noise_params.count("outlier_rate"))
        {
            outlier_rate = std::clamp(noise_params.at("outlier_rate"), 0.0f, 1.0f);
        }
        if (noise_params.count("outlier_std"))
        {
            outlier_std = std::max(0.0f, noise_params.at("outlier_std"));
        }

        if (gaussian_std <= 0.0f && outlier_rate <= 0.0f)
        {
            ROS_DEBUG("No noise parameters specified, returning original cloud");
            return result_cloud;
        }

        size_t noise_points_added = 0;

        // Apply Gaussian noise (radial)
        if (gaussian_std > 0.0f)
        {
            std::normal_distribution<float> gaussian_noise(0.0f, gaussian_std);
            const float min_range = 0.1f; // 10cm minimum (typical LiDAR near-field limit)

            for (auto &point : *result_cloud)
            {
                // Calculate range
                float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                if (range > 1e-6f) // Avoid division by zero
                {
                    // Generate radial noise
                    float range_noise = gaussian_noise(rng_);

                    // Clamp to prevent negative/flipped points
                    float new_range = std::max(range + range_noise, min_range);

                    // Apply noise along beam direction only
                    float scale = new_range / range;
                    point.x *= scale;
                    point.y *= scale;
                    point.z *= scale;
                }
            }

            ROS_DEBUG_STREAM("Applied radial Gaussian noise with std=" << gaussian_std);
        }

        // Add outlier noise (radial)
        if (outlier_rate > 0.0f && outlier_std > 0.0f)
        {
            std::normal_distribution<float> outlier_noise(0.0f, outlier_std);
            const float min_range = 0.1f;

            for (auto &point : *result_cloud)
            {
                if (uniform_dist_(rng_) < outlier_rate)
                {
                    // Calculate range
                    float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                    if (range > 1e-6f)
                    {
                        // Generate radial outlier (larger magnitude)
                        float range_outlier = outlier_noise(rng_);
                        float new_range = std::max(range + range_outlier, min_range);

                        // Apply along beam direction
                        float scale = new_range / range;
                        point.x *= scale;
                        point.y *= scale;
                        point.z *= scale;
                    }
                    ++noise_points_added;
                }
            }

            ROS_DEBUG_STREAM("Applied radial outlier noise to " << noise_points_added
                                                                << " points (rate=" << outlier_rate << ", std=" << outlier_std << ")");
        }

        return result_cloud;
    }

    // =============================================================================
    // MOTION DISTORTION
    // =============================================================================

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr LidarAugmenter::simulateMotionDistortion(
        typename pcl::PointCloud<PointT>::Ptr &cloud,
        const std::vector<uint32_t> &timestamps,
        const MotionParams &motion_params)
    {

        auto result_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>(*cloud));

        if (!cloud || cloud->empty())
        {
            ROS_WARN("Motion distortion: input cloud is empty");
            return result_cloud;
        }

        if (timestamps.size() != cloud->size())
        {
            ROS_WARN_STREAM("Motion distortion: timestamp count (" << timestamps.size()
                                                                   << ") doesn't match point count (" << cloud->size() << ")");
            return result_cloud;
        }

        // Check if motion parameters are significant
        Eigen::Vector3f linear_vel(
            motion_params.linear_velocity[0],
            motion_params.linear_velocity[1],
            motion_params.linear_velocity[2]);

        Eigen::Vector3f angular_vel(
            motion_params.angular_velocity[0],
            motion_params.angular_velocity[1],
            motion_params.angular_velocity[2]);

        if (linear_vel.norm() < 1e-6f && angular_vel.norm() < 1e-6f)
        {
            ROS_DEBUG("Motion parameters are negligible, no distortion applied");
            return result_cloud;
        }

        // Find timestamp range
        uint32_t min_timestamp = *std::min_element(timestamps.begin(), timestamps.end());
        uint32_t max_timestamp = *std::max_element(timestamps.begin(), timestamps.end());

        if (min_timestamp == max_timestamp)
        {
            ROS_DEBUG("All timestamps are identical, no motion distortion applied");
            return result_cloud;
        }

        double time_span = (max_timestamp - min_timestamp) * 1e-9; // Convert nanoseconds to seconds

        // Apply motion distortion to each point
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            double relative_time = (timestamps[i] - min_timestamp) * 1e-9; // Time relative to first point
            double time_factor = relative_time / time_span;                // Normalized time [0, 1]

            auto &point = (*result_cloud)[i];
            Eigen::Vector3f original_pos(point.x, point.y, point.z);

            // Apply linear motion
            Eigen::Vector3f linear_displacement = linear_vel * static_cast<float>(relative_time);

            // Apply rotational motion
            Eigen::Vector3f angular_displacement = angular_vel * static_cast<float>(relative_time);

            // Simple rotation approximation (for small angles)
            if (angular_displacement.norm() < 0.1f)
            { // Small angle approximation
                Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity() +
                                                  skewSymmetric(angular_displacement);
                original_pos = rotation_matrix * original_pos;
            }
            else
            {
                // Full rotation matrix for larger angles
                float angle = angular_displacement.norm();
                if (angle > 1e-6f)
                {
                    Eigen::Vector3f axis = angular_displacement / angle;
                    Eigen::Matrix3f rotation_matrix = Eigen::AngleAxisf(angle, axis).toRotationMatrix();
                    original_pos = rotation_matrix * original_pos;
                }
            }

            // Update point position
            point.x = original_pos.x() + linear_displacement.x();
            point.y = original_pos.y() + linear_displacement.y();
            point.z = original_pos.z() + linear_displacement.z();
        }

        ROS_DEBUG_STREAM("Applied motion distortion over " << time_span << "s with linear_vel="
                                                           << linear_vel.norm() << " m/s, angular_vel=" << angular_vel.norm() << " rad/s");

        return result_cloud;
    }

    // =============================================================================
    // OCCLUSION SIMULATION
    // =============================================================================

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr LidarAugmenter::simulateOcclusion(
        typename pcl::PointCloud<PointT>::Ptr &cloud,
        const std::unordered_map<std::string, float> &occlusion_params)
    {

        auto result_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

        if (!cloud || cloud->empty())
        {
            ROS_WARN("Occlusion simulation: input cloud is empty");
            return result_cloud;
        }

        // occlusion parameters
        float distance_threshold = 50.0f;
        int patch_count = 3;
        float patch_size = 1.5f;

        if (occlusion_params.count("distance_threshold"))
        {
            distance_threshold = std::max(0.1f, occlusion_params.at("distance_threshold"));
        }
        if (occlusion_params.count("random_patches_count"))
        {
            patch_count = std::max(1, static_cast<int>(occlusion_params.at("random_patches_count")));
        }
        if (occlusion_params.count("random_patches_size"))
        {
            patch_size = std::max(0.1f, occlusion_params.at("random_patches_size"));
        }

        // Create occlusion mask
        std::vector<bool> occlusion_mask(cloud->size(), false);

        // Generate random occlusion patches
        for (int patch = 0; patch < patch_count; ++patch)
        {
            // Select random center point that's within distance threshold
            std::vector<size_t> valid_centers;
            for (size_t i = 0; i < cloud->size(); ++i)
            {
                const auto &point = (*cloud)[i];
                float distance = calculateDistance(point);
                if (distance <= distance_threshold)
                {
                    valid_centers.push_back(i);
                }
            }

            if (valid_centers.empty())
            {
                continue;
            }

            // Select random center
            std::uniform_int_distribution<size_t> center_dist(0, valid_centers.size() - 1);
            size_t center_idx = valid_centers[center_dist(rng_)];
            const auto &center_point = (*cloud)[center_idx];

            // Find all points within patch_size of center
            for (size_t i = 0; i < cloud->size(); ++i)
            {
                const auto &point = (*cloud)[i];
                float distance_to_center = std::sqrt(
                    std::pow(point.x - center_point.x, 2) +
                    std::pow(point.y - center_point.y, 2) +
                    std::pow(point.z - center_point.z, 2));

                if (distance_to_center <= patch_size)
                {
                    occlusion_mask[i] = true;
                }
            }
        }

        // Apply occlusion mask
        size_t occluded_points = 0;
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            if (!occlusion_mask[i])
            {
                result_cloud->push_back((*cloud)[i]);
            }
            else
            {
                ++occluded_points;
            }
        }

        // Copy header information
        result_cloud->header = cloud->header;
        result_cloud->height = 1;
        result_cloud->width = result_cloud->size();
        result_cloud->is_dense = cloud->is_dense;

        ROS_DEBUG_STREAM("Occlusion simulation: removed " << occluded_points << "/" << cloud->size()
                                                          << " points using " << patch_count << " patches of size " << patch_size << "m");

        return result_cloud;
    }

    // =============================================================================
    // SPARSE SCAN PATTERN
    // =============================================================================

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr LidarAugmenter::sparseScanPattern(
        typename pcl::PointCloud<PointT>::Ptr &cloud,
        int sparsity_factor)
    {

        auto result_cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

        if (!cloud || cloud->empty())
        {
            ROS_WARN("Sparse scan: input cloud is empty");
            return result_cloud;
        }

        sparsity_factor = std::max(1, sparsity_factor);

        if (sparsity_factor == 1)
        {
            // No sparsification, return copy
            *result_cloud = *cloud;
            return result_cloud;
        }

        // sparse scan logic - systematic sampling
        for (size_t i = 0; i < cloud->size(); i += sparsity_factor)
        {
            result_cloud->push_back((*cloud)[i]);
        }

        // Copy header information
        result_cloud->header = cloud->header;
        result_cloud->height = 1;
        result_cloud->width = result_cloud->size();
        result_cloud->is_dense = cloud->is_dense;

        ROS_DEBUG_STREAM("Sparse scan: " << cloud->size() << " -> " << result_cloud->size()
                                         << " points (factor=" << sparsity_factor << ")");

        return result_cloud;
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    template <typename PointT>
    float LidarAugmenter::calculateDistance(const PointT &point)
    {
        return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    }

    template <typename PointT>
    float LidarAugmenter::calculateAzimuthAngle(const PointT &point)
    {
        return std::atan2(point.y, point.x);
    }

    template <typename PointT>
    std::vector<bool> LidarAugmenter::createRingBasedDropoutMask(
        typename pcl::PointCloud<PointT>::Ptr &cloud, float rate)
    {

        std::vector<bool> mask(cloud->size(), true);

        // Extract ring information if available
        std::vector<int> rings;
        extractRingInfo<PointT>(cloud, rings);
        ;

        if (rings.empty())
        {
            ROS_WARN("No ring information available for ring-based dropout");
            return mask;
        }

        // Find unique rings
        std::set<int> unique_rings(rings.begin(), rings.end());
        int num_rings_to_drop = static_cast<int>(unique_rings.size() * rate);

        // Randomly select rings to drop
        std::vector<int> rings_to_drop;
        std::sample(unique_rings.begin(), unique_rings.end(),
                    std::back_inserter(rings_to_drop), num_rings_to_drop, rng_);

        std::set<int> drop_set(rings_to_drop.begin(), rings_to_drop.end());

        // Apply mask
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            if (i < rings.size() && drop_set.count(rings[i]))
            {
                mask[i] = false;
            }
        }

        return mask;
    }

    template <typename PointT>
    std::vector<bool> LidarAugmenter::createSectorBasedDropoutMask(
        typename pcl::PointCloud<PointT>::Ptr &cloud, float rate)
    {

        std::vector<bool> mask(cloud->size(), true);

        // Divide 360° into sectors and randomly drop some
        int num_sectors = 8; // Divide into 8 sectors of 45° each
        float sector_angle = 2.0f * M_PI / num_sectors;
        int sectors_to_drop = static_cast<int>(num_sectors * rate);

        // Randomly select sectors to drop
        std::vector<int> sectors_to_drop_list;
        for (int i = 0; i < num_sectors; ++i)
        {
            sectors_to_drop_list.push_back(i);
        }
        std::shuffle(sectors_to_drop_list.begin(), sectors_to_drop_list.end(), rng_);
        sectors_to_drop_list.resize(sectors_to_drop);

        std::set<int> drop_sectors(sectors_to_drop_list.begin(), sectors_to_drop_list.end());

        // Apply mask based on azimuth angle
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            float azimuth = calculateAzimuthAngle((*cloud)[i]);
            azimuth = azimuth < 0 ? azimuth + 2.0f * M_PI : azimuth; // Normalize to [0, 2π]

            int sector = static_cast<int>(azimuth / sector_angle);
            sector = std::clamp(sector, 0, num_sectors - 1);

            if (drop_sectors.count(sector))
            {
                mask[i] = false;
            }
        }

        return mask;
    }

    template <typename PointT>
    std::vector<bool> LidarAugmenter::createDistanceBasedDropoutMask(
        typename pcl::PointCloud<PointT>::Ptr &cloud, float rate)
    {

        std::vector<bool> mask(cloud->size(), true);

        // Find min and max distances
        float min_dist = std::numeric_limits<float>::max();
        float max_dist = 0.0f;

        std::vector<float> distances;
        distances.reserve(cloud->size());

        for (const auto &point : *cloud)
        {
            float dist = calculateDistance(point);
            distances.push_back(dist);
            min_dist = std::min(min_dist, dist);
            max_dist = std::max(max_dist, dist);
        }

        // Divide distance range into bands and drop some randomly
        int num_bands = 10;
        float band_size = (max_dist - min_dist) / num_bands;
        int bands_to_drop = static_cast<int>(num_bands * rate);

        std::vector<int> band_indices;
        for (int i = 0; i < num_bands; ++i)
        {
            band_indices.push_back(i);
        }
        std::shuffle(band_indices.begin(), band_indices.end(), rng_);
        band_indices.resize(bands_to_drop);

        std::set<int> drop_bands(band_indices.begin(), band_indices.end());

        // Apply mask
        for (size_t i = 0; i < distances.size(); ++i)
        {
            int band = static_cast<int>((distances[i] - min_dist) / band_size);
            band = std::clamp(band, 0, num_bands - 1);

            if (drop_bands.count(band))
            {
                mask[i] = false;
            }
        }

        return mask;
    }

    template <typename PointT>
    std::vector<bool> LidarAugmenter::createCheckerboardDropoutMask(
        typename pcl::PointCloud<PointT>::Ptr &cloud, float rate)
    {

        std::vector<bool> mask(cloud->size(), true);

        // Create checkerboard pattern in azimuth-elevation space
        int azimuth_divisions = 16;  // 16 divisions in azimuth
        int elevation_divisions = 8; // 8 divisions in elevation

        float azimuth_step = 2.0f * M_PI / azimuth_divisions;
        float elevation_step = M_PI / elevation_divisions;

        // Calculate which squares to drop based on rate
        int total_squares = azimuth_divisions * elevation_divisions;
        int squares_to_drop = static_cast<int>(total_squares * rate);

        std::set<std::pair<int, int>> drop_squares;

        // Generate checkerboard pattern
        for (int i = 0; i < squares_to_drop; ++i)
        {
            int az_idx = i % azimuth_divisions;
            int el_idx = (i / azimuth_divisions) % elevation_divisions;

            // Checkerboard pattern: drop if (az_idx + el_idx) is even
            if ((az_idx + el_idx) % 2 == 0)
            {
                drop_squares.insert({az_idx, el_idx});
            }
        }

        // Apply mask
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            const auto &point = (*cloud)[i];

            float azimuth = calculateAzimuthAngle(point);
            azimuth = azimuth < 0 ? azimuth + 2.0f * M_PI : azimuth;

            float range = calculateDistance(point);
            float elevation = range > 0 ? std::asin(std::clamp(point.z / range, -1.0f, 1.0f)) : 0.0f;
            elevation += M_PI / 2.0f; // Shift to [0, π]

            int az_idx = static_cast<int>(azimuth / azimuth_step);
            int el_idx = static_cast<int>(elevation / elevation_step);

            az_idx = std::clamp(az_idx, 0, azimuth_divisions - 1);
            el_idx = std::clamp(el_idx, 0, elevation_divisions - 1);

            if (drop_squares.count({az_idx, el_idx}))
            {
                mask[i] = false;
            }
        }

        return mask;
    }

    template <typename PointT>
    void LidarAugmenter::extractRingInfo(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                         std::vector<int> &rings)
    {
        rings.clear();

        if constexpr (std::is_same_v<PointT, OusterPoint>)
        {
            rings.reserve(cloud->size());
            for (const auto &point : *cloud)
            {
                rings.push_back(static_cast<int>(point.ring));
            }
        }
        else if constexpr (std::is_same_v<PointT, LivoxPoint>)
        {
            rings.reserve(cloud->size());
            for (const auto &point : *cloud)
            {
                rings.push_back(static_cast<int>(point.line));
            }
        }
        // For generic point types, rings remain empty
    }

    Eigen::Matrix3f LidarAugmenter::skewSymmetric(const Eigen::Vector3f &v)
    {
        Eigen::Matrix3f skew;
        skew << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return skew;
    }

    // Explicit template instantiations for the point types we use
    template std::tuple<pcl::PointCloud<OusterPoint>::Ptr, std::vector<bool>>
    LidarAugmenter::randomDropout<OusterPoint>(pcl::PointCloud<OusterPoint>::Ptr &, float);

    template std::tuple<pcl::PointCloud<LivoxPoint>::Ptr, std::vector<bool>>
    LidarAugmenter::randomDropout<LivoxPoint>(pcl::PointCloud<LivoxPoint>::Ptr &, float);

    template std::tuple<pcl::PointCloud<OusterPoint>::Ptr, std::vector<bool>>
    LidarAugmenter::structuredDropout<OusterPoint>(pcl::PointCloud<OusterPoint>::Ptr &, const std::string &, float);

    template std::tuple<pcl::PointCloud<LivoxPoint>::Ptr, std::vector<bool>>
    LidarAugmenter::structuredDropout<LivoxPoint>(pcl::PointCloud<LivoxPoint>::Ptr &, const std::string &, float);

    template pcl::PointCloud<OusterPoint>::Ptr
    LidarAugmenter::reduceFOV<OusterPoint>(pcl::PointCloud<OusterPoint>::Ptr &, const std::unordered_map<std::string, float> &);

    template pcl::PointCloud<LivoxPoint>::Ptr
    LidarAugmenter::reduceFOV<LivoxPoint>(pcl::PointCloud<LivoxPoint>::Ptr &, const std::unordered_map<std::string, float> &);

    template pcl::PointCloud<OusterPoint>::Ptr
    LidarAugmenter::addNoise<OusterPoint>(pcl::PointCloud<OusterPoint>::Ptr &, const std::unordered_map<std::string, float> &);

    template pcl::PointCloud<LivoxPoint>::Ptr
    LidarAugmenter::addNoise<LivoxPoint>(pcl::PointCloud<LivoxPoint>::Ptr &, const std::unordered_map<std::string, float> &);

    template pcl::PointCloud<OusterPoint>::Ptr
    LidarAugmenter::simulateMotionDistortion<OusterPoint>(pcl::PointCloud<OusterPoint>::Ptr &, const std::vector<uint32_t> &, const MotionParams &);

    template pcl::PointCloud<LivoxPoint>::Ptr
    LidarAugmenter::simulateMotionDistortion<LivoxPoint>(pcl::PointCloud<LivoxPoint>::Ptr &, const std::vector<uint32_t> &, const MotionParams &);

    template pcl::PointCloud<OusterPoint>::Ptr
    LidarAugmenter::simulateOcclusion<OusterPoint>(pcl::PointCloud<OusterPoint>::Ptr &, const std::unordered_map<std::string, float> &);

    template pcl::PointCloud<LivoxPoint>::Ptr
    LidarAugmenter::simulateOcclusion<LivoxPoint>(pcl::PointCloud<LivoxPoint>::Ptr &, const std::unordered_map<std::string, float> &);

    template pcl::PointCloud<OusterPoint>::Ptr
    LidarAugmenter::sparseScanPattern<OusterPoint>(pcl::PointCloud<OusterPoint>::Ptr &, int);

    template pcl::PointCloud<LivoxPoint>::Ptr
    LidarAugmenter::sparseScanPattern<LivoxPoint>(pcl::PointCloud<LivoxPoint>::Ptr &, int);

    template std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::vector<bool>>
    LidarAugmenter::randomDropout<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float rate);

    template std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::vector<bool>>
    LidarAugmenter::structuredDropout<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                      const std::string &pattern, float rate);

    template pcl::PointCloud<pcl::PointXYZI>::Ptr
    LidarAugmenter::reduceFOV<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                              const std::unordered_map<std::string, float> &fov_reduction);

    template pcl::PointCloud<pcl::PointXYZI>::Ptr
    LidarAugmenter::addNoise<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                             const std::unordered_map<std::string, float> &noise_params);

    template pcl::PointCloud<pcl::PointXYZI>::Ptr
    LidarAugmenter::simulateMotionDistortion<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                             const std::vector<uint32_t> &timestamps,
                                                             const MotionParams &motion_params);

    template pcl::PointCloud<pcl::PointXYZI>::Ptr
    LidarAugmenter::simulateOcclusion<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                      const std::unordered_map<std::string, float> &occlusion_params);

    template pcl::PointCloud<pcl::PointXYZI>::Ptr
    LidarAugmenter::sparseScanPattern<pcl::PointXYZI>(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int sparsity_factor);

} // namespace lidar_augmentation
