#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE main

#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pcl_utility_msgs/srv/pcl_voxel_grid_filter.hpp"
#include "./ros2_service_test_fixture.hpp"
#include <boost/test/included/unit_test.hpp>

using sensor_msgs::msg::PointCloud2;
using pcl_utility_msgs::srv::PCLVoxelGridFilter;

static constexpr size_t TOPIC_COUNT = 30U;
using RequestArray = std::array<
  PCLVoxelGridFilter::Request::SharedPtr, TOPIC_COUNT>;
using ResponseArray = std::array<
  PCLVoxelGridFilter::Response::SharedPtr, TOPIC_COUNT>;

struct Fixture : public ROS2ServiceTestFixture {
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

BOOST_AUTO_TEST_CASE(test_output_nonzero) {
  auto messages = Fixture::get_cached_messages<PointCloud2, 30>(
    Fixture::pointcloud_topic);

  RequestArray requests;
  ResponseArray responses;

  for (size_t i = 0; i < messages.size(); i++) {
    requests[i] = std::make_shared<PCLVoxelGridFilter::Request>();
    requests[i]->cloud_in = std::move(messages[i]);
  }

  Fixture::sync_send_requests_transform<PCLVoxelGridFilter>(
    Fixture::service_topic, requests.cbegin(),
    requests.cend(), responses.begin());

  for (const auto& response : responses) {
    BOOST_ASSERT(response->cloud_out.data.size() > 0);
  }
}

BOOST_AUTO_TEST_SUITE_END()