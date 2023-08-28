#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class GroundFilterNode : public rclcpp::Node {

    void points_sub_callback(const sensor_msgs::msg::PointCloud2& msg) const 
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (!this->get_parameter("frame_for_filtering").as_string().empty()) {
            this->transform_to_base_link(cloud, transformed_cloud);
        } else {
            transformed_cloud = cloud;
        }

        pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        for (int i = 0; i < static_cast<int>(transformed_cloud->size()); ++i) {
            if ((*transformed_cloud)[i].z < -1 * this->get_parameter("frame_height").as_double()) {
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

    void transform_to_base_link(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_cloud
    ) const
    {
        geometry_msgs::msg::TransformStamped t;

        try {
            t = m_tf_buffer->lookupTransform(
                this->get_parameter("frame_for_filtering").as_string(),
                cloud->header.frame_id, rclcpp::Time{}
            );
        } catch (const tf2::TransformException& e) {
            RCLCPP_INFO(this->get_logger(), "No transformation to target frame\n");
            return;
        }

        pcl_ros::transformPointCloud(*cloud, *transformed_cloud, t);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_nonground_pub;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    public:
        GroundFilterNode() : Node("perception_ground_filter") {

            this->declare_parameter("frame_for_filtering", "");
            this->declare_parameter("frame_height", 0.0);

            m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

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