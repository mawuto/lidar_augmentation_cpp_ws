#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "lidar_augmentation/augmentation_methods.h"

using namespace lidar_augmentation;

class AugmentationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create test cloud with 100 points
        cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        for (int i = 0; i < 100; ++i)
        {
            pcl::PointXYZI point;
            point.x = i * 0.1f;
            point.y = i * 0.1f;
            point.z = i * 0.1f;
            point.intensity = 100.0f;
            cloud->push_back(point);
        }

        // ✅ CREATE AN INSTANCE OF LidarAugmenter
        augmenter = std::make_shared<LidarAugmenter>();
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::shared_ptr<LidarAugmenter> augmenter; // ✅ ADD THIS
};

TEST_F(AugmentationTest, RandomDropoutTest)
{
    // ✅ CALL ON INSTANCE, NOT AS STATIC METHOD
    auto [result_cloud, mask] = augmenter->randomDropout<pcl::PointXYZI>(cloud, 0.5f);

    // Should have fewer points
    EXPECT_LT(result_cloud->size(), cloud->size());
    // Should have some points remaining
    EXPECT_GT(result_cloud->size(), 0);
    // Mask should match original size
    EXPECT_EQ(mask.size(), cloud->size());

    std::cout << "Original: " << cloud->size() << " points" << std::endl;
    std::cout << "After dropout: " << result_cloud->size() << " points" << std::endl;
}

TEST_F(AugmentationTest, AddNoiseTest)
{
    // ✅ USE THE PARAMETER NAMES YOUR IMPLEMENTATION EXPECTS
    std::unordered_map<std::string, float> noise_params = {
        {"gaussian_std", 0.01f}, // Your implementation uses this
        {"outlier_rate", 0.1f},  // Your implementation uses this
        {"outlier_std", 0.05f}   // Your implementation uses this
    };

    auto result_cloud = augmenter->addNoise<pcl::PointXYZI>(cloud, noise_params);

    EXPECT_EQ(result_cloud->size(), cloud->size());

    bool points_changed = false;
    for (size_t i = 0; i < std::min(result_cloud->size(), cloud->size()); ++i)
    {
        if (std::abs(result_cloud->at(i).x - cloud->at(i).x) > 1e-6f ||
            std::abs(result_cloud->at(i).y - cloud->at(i).y) > 1e-6f ||
            std::abs(result_cloud->at(i).z - cloud->at(i).z) > 1e-6f)
        {
            points_changed = true;
            break;
        }
    }
    EXPECT_TRUE(points_changed);

    std::cout << "Noise added successfully to " << result_cloud->size() << " points" << std::endl;
}

TEST_F(AugmentationTest, ReduceFOVTest)
{
    std::unordered_map<std::string, float> fov_params = {
        {"horizontal_fov", 180.0f}, // Reduce to 180 degrees
        {"vertical_fov_upper", 10.0f},
        {"vertical_fov_lower", -10.0f}};

    // ✅ CALL ON INSTANCE, NOT AS STATIC METHOD
    auto result_cloud = augmenter->reduceFOV<pcl::PointXYZI>(cloud, fov_params);

    // Should have fewer or equal points
    EXPECT_LE(result_cloud->size(), cloud->size());

    std::cout << "FOV reduction: " << cloud->size() << " -> " << result_cloud->size() << " points" << std::endl;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}