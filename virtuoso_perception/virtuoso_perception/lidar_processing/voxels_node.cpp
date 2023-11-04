#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct ClusterBounds {
    float x_min = 0.0;
    float x_max = 0.0;
    float y_min = 0.0;
    float y_max = 0.0;
    float z_min = 0.0;
    float z_max = 0.0;
};

class VoxelsNode : public rclcpp::Node {

    void points_callback(const sensor_msgs::msg::PointCloud2& msg) const {

        pcl::PCLPointCloud2 pcl_pc2; 
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        if (this->get_parameter("debug").as_bool()) {
            RCLCPP_INFO(
                this->get_logger(), 
                ("Input size: " + std::to_string(temp_cloud->size())).c_str()
            );
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(temp_cloud);
        vg.setLeafSize(
            this->get_parameter("leaf_size.x").as_double(),
            this->get_parameter("leaf_size.y").as_double(),
            this->get_parameter("leaf_size.z").as_double()
        );
        vg.filter(*cloud_filtered);

        if (this->get_parameter("debug").as_bool()) {
            RCLCPP_INFO(
                this->get_logger(), 
                ("Output size: " + std::to_string(cloud_filtered->size())).c_str()
            );
        }

        sensor_msgs::msg::PointCloud2 pub_msg;
        pcl::toROSMsg(*cloud_filtered.get(), pub_msg);
        pub_msg.header.frame_id = msg.header.frame_id;
        m_voxels_pub->publish(pub_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_voxels_pub;

    public:
        VoxelsNode() : Node("processing_voxels") {

            pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

            this->declare_parameter("debug", false);
            this->declare_parameter("leaf_size.x", 0.0);
            this->declare_parameter("leaf_size.y", 0.0);
            this->declare_parameter("leaf_size.z", 0.0);

            m_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/perception/lidar/points_shore_filtered", 10, std::bind(&VoxelsNode::points_callback, this, std::placeholders::_1)
            );
            m_voxels_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/perception/lidar/voxels", 10
            );
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelsNode>());
    rclcpp::shutdown();
    return 0;
}