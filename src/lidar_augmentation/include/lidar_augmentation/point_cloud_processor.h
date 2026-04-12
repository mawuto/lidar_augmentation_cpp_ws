// include/lidar_augmentation/point_cloud_processor.h
#ifndef LIDAR_AUGMENTATION_POINT_CLOUD_PROCESSOR_H
#define LIDAR_AUGMENTATION_POINT_CLOUD_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <cstdint>

namespace lidar_augmentation
{

    // Custom Ouster point type
    struct EIGEN_ALIGN16 OusterPoint
    {
        PCL_ADD_POINT4D;       // x, y, z, padding
        float intensity;       // intensity field
        uint32_t t;            // timestamp field
        uint16_t reflectivity; // reflectivity field
        uint16_t ring;         // ring number
        uint32_t range;        // range field
        uint16_t ambient;      // ambient light
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    // Custom Livox point type
    struct EIGEN_ALIGN16 LivoxPoint
    {
        PCL_ADD_POINT4D;    // x, y, z, padding
        float intensity;    // intensity field
        uint8_t tag;        // tag field
        uint8_t line;       // line field
        uint32_t timestamp; // timestamp field (Mid-360 only)
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class PointCloudProcessor
    {
    public:
        // Sensor field structure
        struct SensorFields
        {
            std::vector<std::string> fields;
            std::vector<std::string> required;
            std::string timestamp_field;
        };

        // Static sensor field mappings
        static const std::unordered_map<std::string, SensorFields> SENSOR_FIELDS;

        PointCloudProcessor() = default;
        ~PointCloudProcessor() = default;

        // Main methods
        static std::string detectSensorType(const sensor_msgs::PointCloud2::ConstPtr &msg);

        template <typename PointT>
        static void extractPointsAndFields(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                           typename pcl::PointCloud<PointT>::Ptr &cloud,
                                           std::unordered_map<std::string, std::vector<float>> &fields,
                                           std::vector<std::string> &field_names);

        template <typename PointT>
        static sensor_msgs::PointCloud2 createAugmentedMsg(
            const std_msgs::Header &header,
            const typename pcl::PointCloud<PointT>::Ptr &cloud,
            const std::unordered_map<std::string, std::vector<float>> &fields,
            const std::vector<sensor_msgs::PointField> &original_fields);

        template <typename PointT>
        static std::unordered_map<std::string, std::vector<float>> preserveOusterFields(
            const typename pcl::PointCloud<PointT>::Ptr &points,
            const std::unordered_map<std::string, std::vector<float>> &fields,
            const std::vector<size_t> &indices);

        template <typename PointT>
        static std::unordered_map<std::string, std::vector<float>> preserveLivoxFields(
            const typename pcl::PointCloud<PointT>::Ptr &points,
            const std::unordered_map<std::string, std::vector<float>> &fields,
            const std::vector<size_t> &indices);

        template <typename PointT>
        static std::unordered_map<std::string, std::vector<float>> preserveOriginalTimestamps(
            const std::unordered_map<std::string, std::vector<float>> &original_fields,
            std::unordered_map<std::string, std::vector<float>> &augmented_fields);

        template <typename PointT>
        static std::vector<uint32_t> getTimestamps(
            const std::unordered_map<std::string, std::vector<float>> &fields,
            const std::string &sensor_type);

    private:
        static bool isValidPoint(const pcl::PointXYZ &point);
        static sensor_msgs::PointField::_datatype_type getPointFieldType(const std::string &field_name);

        static void updateMessageFields(
            sensor_msgs::PointCloud2 &msg,
            const std::unordered_map<std::string, std::vector<float>> &fields,
            const std::vector<sensor_msgs::PointField> &original_fields);

        template <typename PointT>
        static void extractOusterFields(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                                        std::unordered_map<std::string, std::vector<float>> &fields,
                                        const std::vector<std::string> &field_names);

        template <typename PointT>
        static void extractLivoxFields(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                                       std::unordered_map<std::string, std::vector<float>> &fields,
                                       const std::vector<std::string> &field_names);

        template <typename PointT>
        static void extractGenericFields(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                                         std::unordered_map<std::string, std::vector<float>> &fields,
                                         const std::vector<std::string> &field_names);
    };

} // namespace lidar_augmentation

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_augmentation::OusterPoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint16_t, ring, ring)(uint32_t, range, range)(uint16_t, ambient, ambient))

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_augmentation::LivoxPoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, tag, tag)(uint8_t, line, line)(uint32_t, timestamp, timestamp))

#endif // LIDAR_AUGMENTATION_POINT_CLOUD_PROCESSOR_H
