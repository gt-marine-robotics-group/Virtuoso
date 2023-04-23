#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class GroundFilterNode : public rclcpp::Node {

    void points_sub_callback(const sensor_msgs::msg::PointCloud2& msg) const {

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            if ((*cloud)[i].z < -1 * this->get_parameter("sensor_height").as_double()) {
                ground_indices->indices.push_back(i);
            }
        }
        extract.setInputCloud(cloud);
        extract.setIndices(ground_indices);
        extract.setNegative(true);
        extract.filter(*cloud);

        sensor_msgs::msg::PointCloud2 pub_msg;
        pcl::toROSMsg(*cloud.get(), pub_msg);
        pub_msg.header.frame_id = msg.header.frame_id;
        m_nonground_pub->publish(pub_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_nonground_pub;

    public:
        GroundFilterNode() : Node("perception_ground_filter") {

            this->declare_parameter("sensor_height", 0.0);

            m_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "input", 10, std::bind(&GroundFilterNode::points_sub_callback, this, std::placeholders::_1)
            );
            m_nonground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/perception/lidar/points_nonground", 10
            );
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundFilterNode>());
    rclcpp::shutdown();
    return 0;
}