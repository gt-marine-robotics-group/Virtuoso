#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

class PclToMapFrameNode : public rclcpp::Node {

    void points_callback(const sensor_msgs::msg::PointCloud2& msg) const {

        geometry_msgs::msg::TransformStamped t;

        try {
            t = m_tf_buffer->lookupTransform(
                std::string("map"), std::string(msg.header.frame_id), rclcpp::Time()
            );
        } catch (const tf2::TransformException& ex) {
            RCLCPP_INFO(this->get_logger(), "Transform failed");
            return;
        }
        
        tf2::Quaternion q(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        );

        tf2::Vector3 p(
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        );

        tf2::Transform transform(q, p);

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        for (int i = 0; i < static_cast<int>(cloud->size()); ++i) {
            auto point = &(*cloud)[i];

            tf2::Vector3 child_coord(point->x, point->y, point->z);

            tf2::Vector3 parent_coord = transform * child_coord;

            point->x = parent_coord.x();
            point->y = parent_coord.y();
            point->z = parent_coord.z();
        }

        sensor_msgs::msg::PointCloud2 pub_msg;
        pcl::toROSMsg(*cloud.get(), pub_msg);
        pub_msg.header.frame_id = "map";
        m_points_pub->publish(pub_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_pub;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    public:
        PclToMapFrameNode() : Node("pcl_to_map_frame") {

            m_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/perception/lidar/voxels", 10, std::bind(&PclToMapFrameNode::points_callback, this, std::placeholders::_1)
            );

            m_points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/mapping/voxels_in_map_frame", 10
            );

            m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

            m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
            
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclToMapFrameNode>());
    rclcpp::shutdown();
    return 0;
}