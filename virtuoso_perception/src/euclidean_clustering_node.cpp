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

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(temp_cloud);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        vg.filter(*cloud_filtered);

        RCLCPP_INFO(this->get_logger(), "Number of filtered points: %ld", cloud_filtered->size());

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PCDWriter writer;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.01);

        RCLCPP_INFO(this->get_logger(), "Set up segmentation stuff.");

        int nr_points = static_cast<int>(cloud_filtered->size());
        int i = 1;
        while (cloud_filtered->size() > 0.3 * nr_points) {
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0) {
                RCLCPP_INFO(this->get_logger(), "Could not estimate a planar model for the given dataset.");
                break;
            }

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);

            extract.filter(*cloud_plane);
            RCLCPP_INFO(this->get_logger(), "Pointcloud representing the planar component: %ld data points",
                cloud_plane->size());
            
            Eigen::Matrix<float, 4, 1> centroid;
            pcl::compute3DCentroid(*cloud_plane, centroid);
            RCLCPP_INFO(this->get_logger(), "Centroid: (%f, %f, %f)", centroid[0], centroid[1], centroid[2]);

            sensor_msgs::msg::PointCloud2 pub_msg;
            cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::toROSMsg(*cloud_plane.get(), pub_msg);
            pub_msg.header.frame_id = "wamv/lidar_wamv_link";
            if (i == 1) {
                m_points1_pub->publish(pub_msg);
            } else if (i == 2) {
                m_points2_pub->publish(pub_msg);
            }
            
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud_filtered = *cloud_f;

            ++i;
        }

        RCLCPP_INFO(this->get_logger(), "Done looping");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_points_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_points1_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_points2_pub;

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
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EuclideanClusteringNode>());
    rclcpp::shutdown();
    return 0;
}