#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.hpp>

class SelfFilterNode : public rclcpp::Node {

    void points_sub_callback(const sensor_msgs::msg::PointCloud2& msg) const {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        pcl::ExtractIndices<pcl::PointXYZ> extract;

        pcl::PointIndices::Ptr output_points(new pcl::PointIndices());
        double r2 = this->get_parameter("radius").as_double();
        r2 *= r2;
        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            auto point = (*cloud)[i];
            if ((point.x * point.x) + (point.y * point.y) < r2) continue;
            output_points->indices.push_back(i);
        }
        extract.setInputCloud(cloud);
        extract.setIndices(output_points);
        extract.filter(*cloud);

        sensor_msgs::msg::PointCloud2 pub_msg;
        pcl::toROSMsg(*cloud.get(), pub_msg);
        pub_msg.header.frame_id = msg.header.frame_id;
        m_filtered_pub->publish(pub_msg);

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_filtered_pub;

    public:
        SelfFilterNode() : Node("processing_self_filter") {

            this->declare_parameter("radius", 0.0);

            m_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/perception/lidar/points_nonground", 10, std::bind(&SelfFilterNode::points_sub_callback, this, std::placeholders::_1)
            );

            m_filtered_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/perception/lidar/points_self_filtered", 10
            );
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SelfFilterNode>());
    rclcpp::shutdown();
    return 0;
}