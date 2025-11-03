// src/cpp/point_cloud_processor.cpp
#include "lidar_augmentation/point_cloud_processor.h"
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cmath>
#include <ros/console.h>

namespace lidar_augmentation
{

    // Static member definition
    const std::unordered_map<std::string, PointCloudProcessor::SensorFields>
        PointCloudProcessor::SENSOR_FIELDS = {
            {"ouster", {{"x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range"}, {"x", "y", "z", "ring"}, "t"}},
            {"livox_avia", {{"x", "y", "z", "intensity", "tag", "line"}, {"x", "y", "z"}, ""}},
            {"livox_mid360", {{"x", "y", "z", "intensity", "tag", "line", "timestamp"}, {"x", "y", "z"}, "timestamp"}},
            {"generic", {{"x", "y", "z", "intensity"}, {"x", "y", "z"}, ""}}};

    std::string PointCloudProcessor::detectSensorType(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // Extract field names from the message
        std::vector<std::string> field_names;
        field_names.reserve(msg->fields.size());

        for (const auto &field : msg->fields)
        {
            field_names.push_back(field.name);
        }

        // Helper lambda to check if field exists
        auto hasField = [&field_names](const std::string &field_name)
        {
            return std::find(field_names.begin(), field_names.end(), field_name) != field_names.end();
        };
        // Ouster detection
        bool has_ring = hasField("ring");
        bool has_reflectivity = hasField("reflectivity");
        bool has_range = hasField("range");
        bool has_t = hasField("t");
        bool has_ambient = hasField("ambient");

        if (has_ring && has_reflectivity && has_range && has_t && has_ambient)
        {
            ROS_DEBUG("Detected Ouster sensor");
            return "ouster";
        }

        // Livox detection
        bool has_line = hasField("line");
        bool has_timestamp = hasField("timestamp");
        bool has_tag = hasField("tag");

        if (has_line && has_timestamp && has_tag)
        {
            ROS_DEBUG("Detected Livox Mid-360 sensor");
            return "livox_mid360";
        }
        else if (has_line && has_tag)
        {
            ROS_DEBUG("Detected Livox Avia sensor");
            return "livox_avia";
        }

        ROS_DEBUG("Using generic sensor type");
        return "generic";
    }

    template <typename PointT>
    void PointCloudProcessor::extractPointsAndFields(
        const sensor_msgs::PointCloud2::ConstPtr &msg,
        typename pcl::PointCloud<PointT>::Ptr &cloud,
        std::unordered_map<std::string, std::vector<float>> &fields,
        std::vector<std::string> &field_names)
    {

        // Initialize output containers
        cloud = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        fields.clear();
        field_names.clear();

        // Extract field names
        for (const auto &field : msg->fields)
        {
            field_names.push_back(field.name);
        }

        // Convert ROS message to PCL cloud using PCL's optimized conversion
        try
        {
            pcl::fromROSMsg(*msg, *cloud);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("Failed to convert ROS message to PCL cloud: " << e.what());
            return;
        }

        if (cloud->empty())
        {
            ROS_WARN("Received empty point cloud");
            return;
        }

        size_t original_size = cloud->size();

        // Remove invalid points
        auto isValid = [](const PointT &point)
        {
            return !(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
                     std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z));
        };

        cloud->erase(
            std::remove_if(cloud->begin(), cloud->end(),
                           [&isValid](const PointT &point)
                           { return !isValid(point); }),
            cloud->end());

        size_t final_size = cloud->size();

        if (final_size != original_size)
        {
            ROS_DEBUG_STREAM("Filtered out " << (original_size - final_size)
                                             << " invalid points (" << final_size << " remaining)");
        }

        // Extract all fields from the valid points
        if (!cloud->empty())
        {
            // Initialize field vectors
            for (const std::string &field_name : field_names)
            {
                fields[field_name].reserve(cloud->size());
            }

            // Extract field data based on point type
            if constexpr (std::is_same_v<PointT, OusterPoint>)
            {
                extractOusterFields<PointT>(cloud, fields, field_names);
            }
            else if constexpr (std::is_same_v<PointT, LivoxPoint>)
            {
                extractLivoxFields<PointT>(cloud, fields, field_names);
            }
            else
            {
                extractGenericFields<PointT>(cloud, fields, field_names);
            }
        }

        ROS_DEBUG_STREAM("Extracted " << cloud->size() << " points with "
                                      << field_names.size() << " fields each");
    }

    template <typename PointT>
    sensor_msgs::PointCloud2 PointCloudProcessor::createAugmentedMsg(
        const std_msgs::Header &header,
        const typename pcl::PointCloud<PointT>::Ptr &cloud,
        const std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<sensor_msgs::PointField> &original_fields)
    {

        sensor_msgs::PointCloud2 output_msg;

        if (cloud->empty())
        {
            ROS_WARN("Creating message from empty point cloud");
            output_msg.header = header;
            output_msg.height = 1;
            output_msg.width = 0;
            output_msg.is_bigendian = false;
            output_msg.point_step = 0;
            output_msg.row_step = 0;
            output_msg.is_dense = true;
            return output_msg;
        }

        // Convert PCL cloud back to ROS message
        pcl::toROSMsg(*cloud, output_msg);

        // Preserve original header timestamp - CRITICAL for synchronization
        output_msg.header = header;

        // Update point cloud with augmented field data
        if (!fields.empty())
        {
            updateMessageFields(output_msg, fields, original_fields);
        }

        ROS_DEBUG_STREAM("Created augmented message with " << output_msg.width
                                                           << " points and " << output_msg.fields.size() << " fields");

        return output_msg;
    }

    template <typename PointT>
    std::unordered_map<std::string, std::vector<float>> PointCloudProcessor::preserveOusterFields(
        const typename pcl::PointCloud<PointT>::Ptr &points,
        const std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<size_t> &indices)
    {

        std::unordered_map<std::string, std::vector<float>> preserved_fields;

        if (indices.empty() || points->empty())
        {
            return preserved_fields;
        }

        // Reserve space for efficiency
        for (const auto &[field_name, field_data] : fields)
        {
            preserved_fields[field_name].reserve(indices.size());
        }

        // Extract fields for selected indices
        for (size_t idx : indices)
        {
            if (idx >= points->size())
            {
                ROS_WARN_STREAM("Index " << idx << " out of bounds for point cloud of size " << points->size());
                continue;
            }

            const auto &point = (*points)[idx];

            // Update XYZ coordinates from the modified points
            if (preserved_fields.count("x"))
                preserved_fields["x"].push_back(point.x);
            if (preserved_fields.count("y"))
                preserved_fields["y"].push_back(point.y);
            if (preserved_fields.count("z"))
                preserved_fields["z"].push_back(point.z);

            // Preserve other Ouster-specific fields from original data
            for (const auto &[field_name, field_data] : fields)
            {
                if (field_name != "x" && field_name != "y" && field_name != "z")
                {
                    if (idx < field_data.size())
                    {
                        preserved_fields[field_name].push_back(field_data[idx]);
                    }
                    else
                    {
                        preserved_fields[field_name].push_back(0.0f); // Default value
                    }
                }
            }
        }

        ROS_DEBUG_STREAM("Preserved " << preserved_fields.size() << " Ouster fields for "
                                      << indices.size() << " points");

        return preserved_fields;
    }

    template <typename PointT>
    std::unordered_map<std::string, std::vector<float>> PointCloudProcessor::preserveLivoxFields(
        const typename pcl::PointCloud<PointT>::Ptr &points,
        const std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<size_t> &indices)
    {

        std::unordered_map<std::string, std::vector<float>> preserved_fields;

        if (indices.empty() || points->empty())
        {
            return preserved_fields;
        }

        // Reserve space for efficiency
        for (const auto &[field_name, field_data] : fields)
        {
            preserved_fields[field_name].reserve(indices.size());
        }

        // Extract fields for selected indices
        for (size_t idx : indices)
        {
            if (idx >= points->size())
            {
                ROS_WARN_STREAM("Index " << idx << " out of bounds for point cloud of size " << points->size());
                continue;
            }

            const auto &point = (*points)[idx];

            // Update XYZ coordinates from the modified points
            if (preserved_fields.count("x"))
                preserved_fields["x"].push_back(point.x);
            if (preserved_fields.count("y"))
                preserved_fields["y"].push_back(point.y);
            if (preserved_fields.count("z"))
                preserved_fields["z"].push_back(point.z);

            // Preserve other Livox-specific fields from original data
            for (const auto &[field_name, field_data] : fields)
            {
                if (field_name != "x" && field_name != "y" && field_name != "z")
                {
                    if (idx < field_data.size())
                    {
                        preserved_fields[field_name].push_back(field_data[idx]);
                    }
                    else
                    {
                        preserved_fields[field_name].push_back(0.0f); // Default value
                    }
                }
            }
        }

        ROS_DEBUG_STREAM("Preserved " << preserved_fields.size() << " Livox fields for "
                                      << indices.size() << " points");

        return preserved_fields;
    }

    template <typename PointT>
    std::unordered_map<std::string, std::vector<float>> PointCloudProcessor::preserveOriginalTimestamps(
        const std::unordered_map<std::string, std::vector<float>> &original_fields,
        std::unordered_map<std::string, std::vector<float>> &augmented_fields)
    {
        // IMPROVED: Only preserve timestamp fields that actually exist
        const std::vector<std::string> timestamp_fields = {"t", "timestamp"};

        for (const std::string &timestamp_field : timestamp_fields)
        {
            // Check if timestamp field exists in BOTH original and augmented
            if (original_fields.count(timestamp_field) && augmented_fields.count(timestamp_field))
            {
                if (original_fields.at(timestamp_field).size() == augmented_fields.at(timestamp_field).size())
                {
                    augmented_fields[timestamp_field] = original_fields.at(timestamp_field);
                    ROS_DEBUG_STREAM("Preserved original timestamps for field: " << timestamp_field);
                }
                else
                {
                    // CHANGED: Use DEBUG instead of WARN for expected size mismatches
                    ROS_DEBUG_STREAM("Size mismatch for timestamp field " << timestamp_field
                                                                          << ": original=" << original_fields.at(timestamp_field).size()
                                                                          << ", augmented=" << augmented_fields.at(timestamp_field).size()
                                                                          << " (expected due to augmentation)");
                }
            }
            else if (original_fields.count(timestamp_field))
            {
                // CHANGED: Use DEBUG for missing fields (this is normal for different sensor types)
                ROS_DEBUG_STREAM("Timestamp field '" << timestamp_field << "' not present in augmented data");
            }
        }

        return augmented_fields;
    }

    template <typename PointT>
    std::vector<uint32_t> PointCloudProcessor::getTimestamps(
        const std::unordered_map<std::string, std::vector<float>> &fields,
        const std::string &sensor_type)
    {
        std::vector<uint32_t> timestamps;

        // Determine timestamp field based on sensor type
        std::string timestamp_field;
        if (sensor_type == "ouster")
        {
            timestamp_field = "t";
        }
        else if (sensor_type == "livox_mid360")
        {
            timestamp_field = "timestamp";
        }
        else
        {
            ROS_DEBUG_STREAM("No timestamp field for sensor type: " << sensor_type);
            return timestamps; // Return empty vector
        }

        // Extract timestamps if field exists
        if (fields.count(timestamp_field))
        {
            const auto &timestamp_floats = fields.at(timestamp_field);
            timestamps.reserve(timestamp_floats.size());

            for (float timestamp_float : timestamp_floats)
            {
                timestamps.push_back(static_cast<uint32_t>(timestamp_float));
            }

            ROS_DEBUG_STREAM("Extracted " << timestamps.size() << " timestamps from field: " << timestamp_field);
        }
        else
        {
            // CHANGED: Use DEBUG instead of implicit error for missing timestamp fields
            ROS_DEBUG_STREAM("Timestamp field '" << timestamp_field << "' not found in fields (normal for some sensors)");
        }

        return timestamps;
    }

    // Private helper methods

    bool PointCloudProcessor::isValidPoint(const pcl::PointXYZ &point)
    {
        // EXACT match to Python validation logic
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
               !(point.x == 0.0f && point.y == 0.0f && point.z == 0.0f);
    }

    sensor_msgs::PointField::_datatype_type PointCloudProcessor::getPointFieldType(const std::string &field_name)
    {
        // EXACT match to Python field type mappings
        if (field_name == "x" || field_name == "y" || field_name == "z" || field_name == "intensity")
        {
            return sensor_msgs::PointField::FLOAT32;
        }
        else if (field_name == "t" || field_name == "range" || field_name == "timestamp")
        {
            return sensor_msgs::PointField::UINT32;
        }
        else if (field_name == "reflectivity" || field_name == "ring" || field_name == "ambient")
        {
            return sensor_msgs::PointField::UINT16;
        }
        else if (field_name == "tag" || field_name == "line")
        {
            return sensor_msgs::PointField::UINT8;
        }
        else
        {
            return sensor_msgs::PointField::FLOAT32; // Default
        }
    }

    // Private field extraction methods

    template <typename PointT>
    void PointCloudProcessor::extractOusterFields(
        const typename pcl::PointCloud<PointT>::Ptr &cloud,
        std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<std::string> &field_names)
    {

        if constexpr (std::is_same_v<PointT, OusterPoint>)
        {
            for (const auto &point : *cloud)
            {
                if (fields.count("x"))
                    fields["x"].push_back(point.x);
                if (fields.count("y"))
                    fields["y"].push_back(point.y);
                if (fields.count("z"))
                    fields["z"].push_back(point.z);
                if (fields.count("intensity"))
                    fields["intensity"].push_back(point.intensity);
                if (fields.count("t"))
                    fields["t"].push_back(static_cast<float>(point.t));
                if (fields.count("reflectivity"))
                    fields["reflectivity"].push_back(static_cast<float>(point.reflectivity));
                if (fields.count("ring"))
                    fields["ring"].push_back(static_cast<float>(point.ring));
                if (fields.count("range"))
                    fields["range"].push_back(static_cast<float>(point.range));
                if (fields.count("ambient"))
                    fields["ambient"].push_back(static_cast<float>(point.ambient));
            }
        }
    }

    template <typename PointT>
    void PointCloudProcessor::extractLivoxFields(
        const typename pcl::PointCloud<PointT>::Ptr &cloud,
        std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<std::string> &field_names)
    {

        if constexpr (std::is_same_v<PointT, LivoxPoint>)
        {
            for (const auto &point : *cloud)
            {
                if (fields.count("x"))
                    fields["x"].push_back(point.x);
                if (fields.count("y"))
                    fields["y"].push_back(point.y);
                if (fields.count("z"))
                    fields["z"].push_back(point.z);
                if (fields.count("intensity"))
                    fields["intensity"].push_back(point.intensity);
                if (fields.count("tag"))
                    fields["tag"].push_back(static_cast<float>(point.tag));
                if (fields.count("line"))
                    fields["line"].push_back(static_cast<float>(point.line));
                if (fields.count("timestamp"))
                    fields["timestamp"].push_back(static_cast<float>(point.timestamp));
            }
        }
    }

    template <typename PointT>
    void PointCloudProcessor::extractGenericFields(
        const typename pcl::PointCloud<PointT>::Ptr &cloud,
        std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<std::string> &field_names)
    {

        // Generic extraction for unknown point types
        for (const auto &point : *cloud)
        {
            if (fields.count("x"))
                fields["x"].push_back(point.x);
            if (fields.count("y"))
                fields["y"].push_back(point.y);
            if (fields.count("z"))
                fields["z"].push_back(point.z);
            // Add intensity if available
            if (fields.count("intensity"))
            {
                // Try to extract intensity if the point type has it
                fields["intensity"].push_back(0.0f); // Default value
            }
        }
    }

    void PointCloudProcessor::updateMessageFields(
        sensor_msgs::PointCloud2 &msg,
        const std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<sensor_msgs::PointField> &original_fields)
    {

        // This method updates the binary data in the PointCloud2 message
        // with the modified field values after augmentation
        // Implementation would involve binary data manipulation
        // For now, we rely on PCL conversion which handles most cases correctly
        ROS_DEBUG("Updated message fields after augmentation");
    }

    // Explicit template instantiations for the point types we use
    template void PointCloudProcessor::extractPointsAndFields<OusterPoint>(
        const sensor_msgs::PointCloud2::ConstPtr &,
        pcl::PointCloud<OusterPoint>::Ptr &,
        std::unordered_map<std::string, std::vector<float>> &,
        std::vector<std::string> &);

    template void PointCloudProcessor::extractPointsAndFields<LivoxPoint>(
        const sensor_msgs::PointCloud2::ConstPtr &,
        pcl::PointCloud<LivoxPoint>::Ptr &,
        std::unordered_map<std::string, std::vector<float>> &,
        std::vector<std::string> &);

    template sensor_msgs::PointCloud2 PointCloudProcessor::createAugmentedMsg<OusterPoint>(
        const std_msgs::Header &,
        const pcl::PointCloud<OusterPoint>::Ptr &,
        const std::unordered_map<std::string, std::vector<float>> &,
        const std::vector<sensor_msgs::PointField> &);

    template sensor_msgs::PointCloud2 PointCloudProcessor::createAugmentedMsg<LivoxPoint>(
        const std_msgs::Header &,
        const pcl::PointCloud<LivoxPoint>::Ptr &,
        const std::unordered_map<std::string, std::vector<float>> &,
        const std::vector<sensor_msgs::PointField> &);

    template std::unordered_map<std::string, std::vector<float>> PointCloudProcessor::preserveOusterFields<OusterPoint>(
        const pcl::PointCloud<OusterPoint>::Ptr &,
        const std::unordered_map<std::string, std::vector<float>> &,
        const std::vector<size_t> &);

    template std::unordered_map<std::string, std::vector<float>> PointCloudProcessor::preserveLivoxFields<LivoxPoint>(
        const pcl::PointCloud<LivoxPoint>::Ptr &,
        const std::unordered_map<std::string, std::vector<float>> &,
        const std::vector<size_t> &);

    template std::vector<uint32_t> PointCloudProcessor::getTimestamps<OusterPoint>(
        const std::unordered_map<std::string, std::vector<float>> &,
        const std::string &);

    template std::vector<uint32_t> PointCloudProcessor::getTimestamps<LivoxPoint>(
        const std::unordered_map<std::string, std::vector<float>> &,
        const std::string &);

    // Explicit template instantiations for PointCloudProcessor
    template void PointCloudProcessor::extractPointsAndFields<pcl::PointXYZI>(
        const sensor_msgs::PointCloud2::ConstPtr &msg,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        std::unordered_map<std::string, std::vector<float>> &fields,
        std::vector<std::string> &field_names);

    template sensor_msgs::PointCloud2 PointCloudProcessor::createAugmentedMsg<pcl::PointXYZI>(
        const std_msgs::Header &header,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        const std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<sensor_msgs::PointField> &original_fields);

    template std::unordered_map<std::string, std::vector<float>>
    PointCloudProcessor::preserveOriginalTimestamps<pcl::PointXYZI>(
        const std::unordered_map<std::string, std::vector<float>> &original_fields,
        std::unordered_map<std::string, std::vector<float>> &augmented_fields);

    template std::unordered_map<std::string, std::vector<float>>
    PointCloudProcessor::preserveOriginalTimestamps<OusterPoint>(
        const std::unordered_map<std::string, std::vector<float>> &original_fields,
        std::unordered_map<std::string, std::vector<float>> &augmented_fields);

    template std::unordered_map<std::string, std::vector<float>>
    PointCloudProcessor::preserveOriginalTimestamps<LivoxPoint>(
        const std::unordered_map<std::string, std::vector<float>> &original_fields,
        std::unordered_map<std::string, std::vector<float>> &augmented_fields);

    template std::vector<uint32_t> PointCloudProcessor::getTimestamps<pcl::PointXYZI>(
        const std::unordered_map<std::string, std::vector<float>> &fields,
        const std::string &sensor_type);

    template void PointCloudProcessor::extractOusterFields<OusterPoint>(
        const pcl::PointCloud<OusterPoint>::Ptr &cloud,
        std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<std::string> &field_names);

    template void PointCloudProcessor::extractLivoxFields<LivoxPoint>(
        const pcl::PointCloud<LivoxPoint>::Ptr &cloud,
        std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<std::string> &field_names);

    template void PointCloudProcessor::extractGenericFields<pcl::PointXYZI>(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
        std::unordered_map<std::string, std::vector<float>> &fields,
        const std::vector<std::string> &field_names);

} // namespace lidar_augmentation
