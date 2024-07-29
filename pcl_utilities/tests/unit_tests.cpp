#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE main

#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pcl_utility_msgs/srv/pcl_voxel_grid_filter.hpp"
#include "./ros2_test_fixture.hpp"
#include <boost/test/included/unit_test.hpp>

using sensor_msgs::msg::PointCloud2;
using pcl_utility_msgs::srv::PCLVoxelGridFilter;

static constexpr size_t TOPIC_COUNT = 10U;
using RequestArray = std::array<
  PCLVoxelGridFilter::Request::SharedPtr, TOPIC_COUNT>;
using ResponseArray = std::array<
  PCLVoxelGridFilter::Response::SharedPtr, TOPIC_COUNT>;

struct Fixture : public robot_common_tests::GlobalFixture {
  static std::string pointcloud_topic;
  static std::string service_topic;

  Fixture() {
    pointcloud_topic = Fixture::get_node()->declare_parameter<std::string>("point_cloud_topic");
    service_topic = Fixture::get_node()->declare_parameter<std::string>("node_client_name");
  }
};

std::string Fixture::pointcloud_topic;
std::string Fixture::service_topic;

BOOST_GLOBAL_FIXTURE(Fixture);

BOOST_AUTO_TEST_SUITE(voxel_grid_filter_suite)

BOOST_AUTO_TEST_CASE(test_output_size_lte) {
  auto messages = Fixture::get_cached_messages<PointCloud2, TOPIC_COUNT>(
    Fixture::pointcloud_topic);

  for (size_t i = 0; i < messages.size(); i++) {
    size_t input_size = messages[i].data.size();
    auto request = std::make_shared<PCLVoxelGridFilter::Request>();
    request->cloud_in = std::move(messages[i]);
    auto response = Fixture::sync_send_request<PCLVoxelGridFilter>(Fixture::service_topic, request, std::chrono::seconds(1));

    BOOST_CHECK_LE(response->cloud_out.data.size(), input_size);
  }
}

BOOST_AUTO_TEST_CASE(test_output_nonzero) {
  auto messages = Fixture::get_cached_messages<PointCloud2, TOPIC_COUNT>(
    Fixture::pointcloud_topic);

  for (PointCloud2& message : messages) {
    size_t input_size = message.data.size();
    auto request = std::make_shared<PCLVoxelGridFilter::Request>();
    request->cloud_in = std::move(message);
    auto response = Fixture::sync_send_request<PCLVoxelGridFilter>
      (Fixture::service_topic, request);

    BOOST_CHECK(response->cloud_out.data.size() && input_size);
  }
}

BOOST_AUTO_TEST_SUITE_END()