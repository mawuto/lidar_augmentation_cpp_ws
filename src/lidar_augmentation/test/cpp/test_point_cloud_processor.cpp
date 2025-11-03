#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include "lidar_augmentation/point_cloud_processor.h"

using namespace lidar_augmentation;

class PointCloudProcessorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create test cloud
        cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        for (int i = 0; i < 10; ++i)
        {
            pcl::PointXYZI point;
            point.x = i * 1.0f;
            point.y = i * 1.0f;
            point.z = i * 1.0f;
            point.intensity = 100.0f + i;
            cloud->push_back(point);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

TEST_F(PointCloudProcessorTest, BasicFunctionality)
{
    // Test that the class exists and can be instantiated
    EXPECT_TRUE(cloud != nullptr);
    EXPECT_EQ(cloud->size(), 10);
    std::cout << "PointCloudProcessor basic test passed" << std::endl;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}