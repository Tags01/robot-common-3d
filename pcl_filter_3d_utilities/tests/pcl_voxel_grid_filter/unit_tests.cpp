#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/test/included/unit_test.hpp>
#include "pcl_filter_3d_msgs/srv/pcl_voxel_grid_filter.hpp"

#include "../ros2_service_test_fixture.hpp"

using Fixture = ROS2ServiceTestFixture;
using sensor_msgs::msg::PointCloud2;
using pcl_filter_3d_msgs::srv::PCLVoxelGridFilter;

static constexpr size_t TOPIC_COUNT = 30ZU;

struct Fixture : public ROS2ServiceTestFixture {
  static std::string pointcloud_topic;
  static std::string service_topic;

  Fixture() {
    pointcloud_topic = Fixture::get_node()->declare_parameter("camera_topic");
    service_topic = Fixture::get_node()->declare_parameter("service_topic");
  }
};

BOOST_GLOBAL_FIXTURE(Fixture);

BOOST_AUTO_TEST_CASE(test_output_nonzero) {
  auto messages = Fixture::get_cached_messages<PointCloud2, 30>(
    Fixture::pointcloud_topic);

  std::array<PCLVoxelGridFilter::Request::SharedPtr, TOPIC_COUNT> requests;
  std::array<PCLVoxelGridFilter::Response::SharedPtr, TOPIC_COUNT> resonses;

  size_t i;
  for (size_t i = 0; i < ) {
    request = std::make_shared<PCLVoxelGridFilter::Request>();
    request->cloud_in = std::move(messages[i]);
    i++;
  }

  Fixture::ros_service_transform(
    Fixture::service_topic, requests.cbegin(),
    requests.cend(), response.begin());

  for (const auto& response : responses) {
    BOOST_ASSERT(messages.size() > 0);
  }
}