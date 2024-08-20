/*
 * Author: Christian Tagliamonte
 * Date: Aug Aug 4, 2024
 * Editors: N/A
 * Last Modified: Aug 14, 2024
 *
 * Description: A node to test the concatenate_point_cloud_service.
 * This node listens to a series of sequential pointcloud
 *   messages from the topic specified by the parameter, `point_cloud_topic.`
 * Each message is stored within a queue with a
 *   size specified by the parameter, `num_point_clouds` (defaults to 5). When
 *   the queue is full, all messages in are sent
 *   to the `concatenate_point_cloud_service`.
 *
 * The concatenated point cloud is then published to the topic
 *   `concatenate_point_cloud/cloud_concatenated`
 *
 * Usage:
 *    `ros2 launch pcl_utilities test_concatenate_point_cloud.xml point_cloud_topic:=<POINT_CLOUD_TOPIC>`
 */
#include <chrono>   // std::chrono::seconds
#include <cstddef>  // size_t
#include <cstdint>  // int64_t
#include <deque>
#include <functional>  // std::bind, std::placeholders
#include <ios>  // std::fixed, std::setprecision
#include <memory>  // std::make_shared
#include <sstream>  // std::stringstream
#include <string>
#include <utility>  // std::move

#include "rclcpp/executors.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_utility_msgs/srv/pcl_euclidean_cluster_extraction.hpp"

using pcl_utility_msgs::srv::PCLEuclideanClusterExtraction;
using sensor_msgs::msg::PointCloud2;

constexpr std::chrono::seconds MAX_WAIT_TIME {1U};

class TestConcatenatePointCloudNode : public rclcpp::Node
{
private:
  rclcpp::Client<PCLConcatenatePointCloud>::SharedPtr
    concatenate_point_cloud_client_;
  rclcpp::Publisher<PointCloud2>::SharedPtr output_publisher_;
  std::string camera_topic_;
  size_t num_point_clouds_;

public:
  TestConcatenatePointCloudNode()
  : rclcpp::Node("simple_test_concatenate_point_cloud")
  {
    std::string client_topic = declare_parameter<std::string>("node_client_name");
    camera_topic_ = declare_parameter<std::string>("point_cloud_topic");

    // rclcpp uses int64_t internally, use explicit cast to size_t
    rcl_interfaces::msg::ParameterDescriptor param_constraints;
    rcl_interfaces::msg::IntegerRange integer_range;
    integer_range.from_value = 0U;
    integer_range.to_value = MAX_POINT_CLOUDS;
    param_constraints.integer_range.push_back(integer_range);

    num_point_clouds_ = static_cast<size_t>(
      declare_parameter<int64_t>("num_point_clouds", std::move(param_constraints)));

    concatenate_point_cloud_client_ =
      create_client<PCLConcatenatePointCloud>(client_topic);

    output_publisher_ = create_publisher<PointCloud2>(
      "concatenate_point_cloud/cloud_concatenated", 1);
  }

  void spin()
  {
    PointCloud2 point_cloud_message;

    while (rclcpp::ok()) {
      bool was_retrieved = rclcpp::wait_for_message(
        point_cloud_message, shared_from_this(),
        camera_topic_, MAX_WAIT_TIME);

      if (!was_retrieved) {
        RCLCPP_ERROR_STREAM(
          get_logger(),
          "A camera message could not be retrieved within 1 second.");
        continue;
      }

      process_point_cloud(std::move(point_cloud_message));
    }
  }

  void process_point_cloud(PointCloud2 && point_cloud)
  {
    auto request = std::make_shared<PCLConcatenatePointCloud::Request>();
    request->cloud_in = point_cloud;

    auto response_future =
      concatenate_point_cloud_client_->async_send_request(request);

    auto response_code = rclcpp::spin_until_future_complete(
      shared_from_this(), response_future, MAX_WAIT_TIME);

    if (response_code != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to recieve a response from the service");
      return;
    }

    PCLConcatenatePointCloud::Response::SharedPtr response {
      response_future.get()};
    output_publisher_->publish(response->cloud_out);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestConcatenatePointCloudNode>();
  node->spin();

  rclcpp::shutdown();
  return 0;
}
