#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
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
#include "virtuoso_msgs/msg/bounding_box.hpp"
#include "virtuoso_msgs/msg/bounding_box_array.hpp"

struct ClusterBounds {
    float x_min = 0.0;
    float x_max = 0.0;
    float y_min = 0.0;
    float y_max = 0.0;
    float z_min = 0.0;
    float z_max = 0.0;
};

class EuclideanClusteringNode : public rclcpp::Node {

    void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {

        pcl::PCLPointCloud2 pcl_pc2; 
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(temp_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(this->get_parameter("cluster_tolerance").as_double());
        ec.setMinClusterSize(this->get_parameter("min_cluster_size").as_int());
        ec.setMaxClusterSize(this->get_parameter("max_cluster_size").as_int());
        ec.setSearchMethod(tree);
        ec.setInputCloud(temp_cloud);
        ec.extract(cluster_indices);

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_marker.ns = "perception_ec";
        visualization_msgs::msg::MarkerArray clear_markers;
        clear_markers.markers.push_back(clear_marker);
        m_clusters_viz_pub->publish(clear_markers);

        visualization_msgs::msg::MarkerArray markers;
        markers.markers.resize(cluster_indices.size());

        virtuoso_msgs::msg::BoundingBoxArray boxes;
        boxes.boxes.resize(cluster_indices.size());

        int j = 1;
        for (const auto& cluster : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

            for (const auto idx : cluster.indices) {
                cloud_cluster->push_back((*temp_cloud)[idx]);
            }

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

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

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = msg->header.frame_id;
            marker.id = j;
            marker.ns = "perception_ec";
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.scale.x = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            
            add_point_to_marker(marker, bounds.x_min, bounds.y_min, bounds.z_min);
            add_point_to_marker(marker, bounds.x_min, bounds.y_max, bounds.z_min);

            add_point_to_marker(marker, bounds.x_min, bounds.y_max, bounds.z_max);
            add_point_to_marker(marker, bounds.x_min, bounds.y_min, bounds.z_max);

            markers.markers[j-1] = marker;

            virtuoso_msgs::msg::BoundingBox box;
            
            add_centroid_to_bounding_box(box, *cloud_cluster);

            add_point_to_bounding_box_corners(box, bounds.x_min, bounds.y_min, bounds.z_min);
            add_point_to_bounding_box_corners(box, bounds.x_min, bounds.y_max, bounds.z_min);
            add_point_to_bounding_box_corners(box, bounds.x_min, bounds.y_max, bounds.z_max);
            add_point_to_bounding_box_corners(box, bounds.x_min, bounds.y_min, bounds.z_max);

            boxes.boxes[j-1] = box;

            ++j;
        }

        m_clusters_viz_pub->publish(markers);
        m_boxes_pub->publish(boxes);
    }

    void add_point_to_marker(visualization_msgs::msg::Marker& marker, float x, float y, float z) const {
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        marker.points.push_back(point);
    }

    void add_centroid_to_bounding_box(virtuoso_msgs::msg::BoundingBox& box, 
        const pcl::PointCloud<pcl::PointXYZ>& cloud) const {
        Eigen::Matrix<float, 4, 1> centroid;
        pcl::compute3DCentroid(cloud, centroid);
        box.centroid.x = centroid[0];
        box.centroid.y = centroid[1];
        box.centroid.z = centroid[2];
    }

    void add_point_to_bounding_box_corners(virtuoso_msgs::msg::BoundingBox& box,
        float x, float y, float z) const {
        geometry_msgs::msg::Point32 point;
        point.x = x;
        point.y = y;
        point.z = z;
        box.corners.push_back(point);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_sub;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_clusters_viz_pub;

    rclcpp::Publisher<virtuoso_msgs::msg::BoundingBoxArray>::SharedPtr m_boxes_pub;

    public:
        EuclideanClusteringNode() : Node("perception_euclidean_clustering") {

            pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

            this->declare_parameter("cluster_tolerance", 0.0);
            this->declare_parameter("min_cluster_size", 0);
            this->declare_parameter("max_cluster_size", 0);

            m_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/perception/lidar/voxels", 10, std::bind(&EuclideanClusteringNode::points_callback, this, std::placeholders::_1)
            );
            m_clusters_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/perception/clusters_viz", 10
            );
            m_boxes_pub = this->create_publisher<virtuoso_msgs::msg::BoundingBoxArray>(
                "/perception/lidar/bounding_boxes", 10
            );
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EuclideanClusteringNode>());
    rclcpp::shutdown();
    return 0;
}