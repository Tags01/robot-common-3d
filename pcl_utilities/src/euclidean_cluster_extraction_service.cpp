/*
 * Author: Brian Flynn
 * Date: Nov 15, 2022
 * Editors: Christian Tagliamonte
 * Last Modified: Aug 4, 2024
 * Adapted from:
 * https://github.com/uml-robotics/armada_behaviors/blob/main/armada_flexbe_utilities/src/service/pcl_concatenate_pointcloud_service.cpp
 *
 * Description: Starts up a service for concatenating point cloud messages.
 *
 * Input: sensor_msgs/PointCloud2[]
 * Output: sensor_msgs/PointCloud2
 *
 * Usage:
 *    `ros2 launch pcl_utilities concatenate_point_cloud.xml`
 */
#include <cassert>  // assert
#include <functional>  // for std::bind
#include <limits>  // std::numeric_limits
#include <memory>  // std::make_shared
#include <string_view>
#include <type_traits>  // std::common_type
#include <utility>  // std::move
#include <vector>

#include "pcl/impl/point_types.hpp"
#include "pcl/point_cloud.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_utility_msgs/srv/pcl_euclidean_cluster_extraction.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using pcl_utility_msgs::srv::PCLEuclideanClusterExtraction;
constexpr std::string_view g_PARAM_NAMESPACE = "filters.euclidean_cluster_extraction.";

class EuclideanClusterExtractionService : public rclcpp::Node
{
private:
  rclcpp::Service<PCLEuclideanClusterExtraction>::SharedPtr
    euclidean_cluster_extraction_service_;
  rcl_interfaces::msg::ParameterDescriptor param_constraints_;
  double cluster_tolerance_;
  int64_t min_cluster_size_;
  int64_t max_cluster_size_;

public:
  /**
   * Class Constructor.
   *
   * Constructor for EuclideanClusterExtractionService class.
   */
  EuclideanClusterExtractionService()
  : rclcpp::Node("euclidean_cluster_extraction_service")
  {
    std::string param_namespace {g_PARAM_NAMESPACE};

    // cap the parameter value within the range of pcl::uindex_t
    rcl_interfaces::msg::IntegerRange integer_range;
    using RangeType = std::common_type_t<int64_t, pcl::uindex_t>;
    auto value_or_max = std::min<RangeType>(
      std::numeric_limits<int64_t>::max(),
      std::numeric_limits<pcl::uindex_t>::max());

    integer_range.from_value = 0;
    integer_range.to_value = static_cast<int64_t>(value_or_max);
    param_constraints_.integer_range.push_back(integer_range);

    // declare all parameters
    cluster_tolerance_ = declare_parameter<double>(
      param_namespace + "cluster_tolerance");

    min_cluster_size_ = declare_parameter<int64_t>(
      param_namespace + "min_cluster_size", param_constraints_);
    max_cluster_size_ = declare_parameter<int64_t>(
      param_namespace + "max_cluster_size", param_constraints_);

    // create callback and setup service
    auto callback = std::bind(
      &EuclideanClusterExtractionService::euclidean_cluster_extraction,
      this, std::placeholders::_1, std::placeholders::_2);

    euclidean_cluster_extraction_service_ = create_service<PCLEuclideanClusterExtraction>(
      "euclidean_cluster_extraction", std::move(callback));
  }

  /**
   * Segment clusters within a PointCloud into individual cloud objects.
   *
   * BGiven a PointCloud2 message, segment clusters of points into their 
   * own PointCloud2 objects for further processing/handling.
   *
   * @param[in] req sensor_msgs/PointCloud2 A PointCloud2 message.
   * @param[out] res sensor_msgs/PointCloud2 A PointCloud2 message.
   * @return Bool Service completion result.
   */
  bool euclidean_cluster_extraction(
    PCLEuclideanClusterExtraction::Request::SharedPtr req,
    PCLEuclideanClusterExtraction::Response::SharedPtr res)
  {
    std::string param_namespace {g_PARAM_NAMESPACE};

    get_parameter(param_namespace + "cluster_tolerance", cluster_tolerance_);
    get_parameter(param_namespace + "min_cluster_size", min_cluster_size_);
    get_parameter(param_namespace + "max_cluster_size", max_cluster_size_);

    auto input_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    sensor_msgs::msg::PointCloud2 temp_cloud;
    std::vector<sensor_msgs::msg::PointCloud2> obstacle_cloud_list_out;

    pcl::moveFromROSMsg(req->cloud_in, *input_cloud);

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    tree->setInputCloud(input_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    std::vector<pcl::PointIndices> cluster_indices;

    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(static_cast<pcl::uindex_t>(min_cluster_size_));
    ec.setMaxClusterSize(static_cast<pcl::uindex_t>(max_cluster_size_));
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    for (const pcl::PointIndices& indecies : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZRGB> cloud_cluster;

      for (pcl::index_t index : indecies.indices) {
        assert(index >= 0 && index <= std::numeric_limits<size_t>::max());

        cloud_cluster.push_back((*input_cloud)[static_cast<size_t>(index)]);
      }

      assert(cloud_cluster.size() <= std::numeric_limits<pcl::uindex_t>::max());

      cloud_cluster.width = static_cast<pcl::uindex_t>(cloud_cluster.size());
      cloud_cluster.height = 1U;
      cloud_cluster.is_dense = true;
      cloud_cluster.header.frame_id = input_cloud->header.frame_id;

      pcl::toROSMsg(cloud_cluster, temp_cloud);
      obstacle_cloud_list_out.push_back(std::move(temp_cloud));
    }

    if (obstacle_cloud_list_out.empty()) {
      return true;
    }

    res->target_cloud_out = std::move(obstacle_cloud_list_out[0]);
    obstacle_cloud_list_out.erase(obstacle_cloud_list_out.cbegin());

    res->obstacle_cloud_list_out = std::move(obstacle_cloud_list_out);
    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EuclideanClusterExtractionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
