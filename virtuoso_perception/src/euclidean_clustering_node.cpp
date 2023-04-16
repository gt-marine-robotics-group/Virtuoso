#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

struct ClusterBounds {
    float x_min = 0.0;
    float x_max = 0.0;
    float y_min = 0.0;
    float y_max = 0.0;
    float z_min = 0.0;
    float z_max = 0.0;
};

class EuclideanClusteringNode : public rclcpp::Node {

    void points_callback(const sensor_msgs::msg::PointCloud2& msg) const {
        RCLCPP_INFO(this->get_logger(), "Got pointcloud");

        pcl::PCLPointCloud2 pcl_pc2; 
        pcl_conversions::toPCL(msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        RCLCPP_INFO(this->get_logger(), "Converted pointcloud with %zu points", temp_cloud->size());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(temp_cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud_filtered);

        RCLCPP_INFO(this->get_logger(), "Number of filtered points: %ld", cloud_filtered->size());

        if (cloud_filtered->size() == temp_cloud->size()) {
            RCLCPP_INFO(this->get_logger(), "Voxel filter failed... returning");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Done looping");
        RCLCPP_INFO(this->get_logger(), "Cloud filtered size: %ld", cloud_filtered->size());

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(2);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        RCLCPP_INFO(this->get_logger(), "EC Indices: %zu", cluster_indices.size());

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_marker.ns = "perception_ec";
        visualization_msgs::msg::MarkerArray clear_markers;
        clear_markers.markers.push_back(clear_marker);
        m_clusters_viz_pub->publish(clear_markers);

        visualization_msgs::msg::MarkerArray markers;
        markers.markers.resize(cluster_indices.size());

        int j = 1;
        for (const auto& cluster : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

            for (const auto idx : cluster.indices) {
                cloud_cluster->push_back((*cloud_filtered)[idx]);
            }

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            RCLCPP_INFO(this->get_logger(), "Point cloud for cluster: %ld points", cloud_cluster->size());

            sensor_msgs::msg::PointCloud2 pub_msg;
            pcl::toROSMsg(*cloud_cluster.get(), pub_msg);
            pub_msg.header.frame_id = "wamv/lidar_wamv_link";
            RCLCPP_INFO(this->get_logger(), "ROS PCL Size: %zu", pub_msg.data.size());
            if (j == 1) {
                m_cluster1_pub->publish(pub_msg);
            } else if (j == 2) {
                m_cluster2_pub->publish(pub_msg);
            }

            ClusterBounds bounds;

            for (const auto& point : cloud_cluster->points) {
                if (bounds.x_min == 0.0 || point.x < bounds.x_min) {
                    bounds.x_min = point.x;
                } 
                if (bounds.x_max == 0.0 || point.x > bounds.x_max) {
                    bounds.x_max = point.x;
                }
                if (bounds.y_min == 0.0 || point.y < bounds.y_min) {
                    bounds.y_min = point.y;
                }
                if (bounds.y_max == 0.0 || point.y > bounds.y_max) {
                    bounds.y_max = point.y;
                }
                if (bounds.z_min == 0.0 || point.z < bounds.z_min) {
                    bounds.z_min = point.z;
                }
                if (bounds.z_max == 0.0 || point.z > bounds.z_max) {
                    bounds.z_max = point.z;
                }
            }
            RCLCPP_INFO(this->get_logger(), "X min: %f", bounds.x_min);
            RCLCPP_INFO(this->get_logger(), "X max: %f", bounds.x_max);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "wamv/lidar_wamv_link";
            marker.id = j;
            marker.ns = "perception_ec";
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.scale.x = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            
            // Bottom of the cube
            add_point_to_marker(marker, bounds.x_min, bounds.y_min, bounds.z_min);
            add_point_to_marker(marker, bounds.x_max, bounds.y_min, bounds.z_min);
            add_point_to_marker(marker, bounds.x_max, bounds.y_max, bounds.z_min);
            add_point_to_marker(marker, bounds.x_min, bounds.y_max, bounds.z_min);

            // Top of the cube
            add_point_to_marker(marker, bounds.x_min, bounds.y_max, bounds.z_max);
            add_point_to_marker(marker, bounds.x_min, bounds.y_min, bounds.z_max);
            add_point_to_marker(marker, bounds.x_max, bounds.y_min, bounds.z_max);
            add_point_to_marker(marker, bounds.x_max, bounds.y_max, bounds.z_max);

            markers.markers[j - 1] = marker;

            RCLCPP_INFO(this->get_logger(), "Marker length: %zu", marker.points.size());

            ++j;
        }

        m_clusters_viz_pub->publish(markers);
    }

    void add_point_to_marker(visualization_msgs::msg::Marker& marker, float x, float y, float z) const {
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        marker.points.push_back(point);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_points1_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_points2_pub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cluster1_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cluster2_pub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_clusters_viz_pub;

    public:
        EuclideanClusteringNode() : Node("perception_euclidean_clustering") {
            m_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/perception/lidar/points_shore_filtered", 10, std::bind(&EuclideanClusteringNode::points_callback, this, std::placeholders::_1)
            );
            m_points1_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/planar1", 10
            );
            m_points2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/planar2", 10
            );
            m_cluster1_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/cluster1", 10
            );
            m_cluster2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/cluster2", 10
            );
            m_clusters_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/perception/clusters_viz", 10
            );
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EuclideanClusteringNode>());
    rclcpp::shutdown();
    return 0;
}