#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class EuclideanClusteringNode : public rclcpp::Node {

    void points_callback(const sensor_msgs::msg::PointCloud2& msg) const {
        RCLCPP_INFO(this->get_logger(), "Got pointcloud");

        pcl::PCLPointCloud2 pcl_pc2; 
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        RCLCPP_INFO(this->get_logger(), "Converted pointcloud with %zu points", temp_cloud->size());

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(temp_cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);
        vg.filter(*cloud_filtered);

        RCLCPP_INFO(this->get_logger(), "Number of filtered points: %ld", cloud_filtered->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_sub;

    public:
        EuclideanClusteringNode() : Node("perception_euclidean_clustering") {
            m_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/perception/lidar/points_shore_filtered", 10, std::bind(&EuclideanClusteringNode::points_callback, this, std::placeholders::_1)
            );
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EuclideanClusteringNode>());
    rclcpp::shutdown();
    return 0;
}