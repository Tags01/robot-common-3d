#include "ros2_service_test_fixture.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/logging.hpp>
#include <rcl_interfaces/msg/list_parameters_result.hpp>

#include <string>
#include <unordered_map>
#include <memory>
#include <cstdint>
#include <sstream>
#include <iostream>

#include <boost/test/unit_test.hpp>

using Self = ROS2ServiceTestFixture;

rclcpp::Executor::SharedPtr Self::executor_;
rclcpp::Node::SharedPtr Self::node_;
std::unordered_map<std::string, std::any> Self::msgs_map_;
std::stringstream Self::output_stream_{};
rclcpp::CallbackGroup::SharedPtr Self::callback_group_;

ROS2ServiceTestFixture::ROS2ServiceTestFixture() {
  boost::unit_test::master_test_suite_t& suite =
    boost::unit_test::framework::master_test_suite();

  rclcpp::init(suite.argc, suite.argv);

  Self::executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  Self::node_ = std::make_shared<rclcpp::Node>("filter_node", "test");
  Self::executor_->add_node(Self::node_);
  Self::callback_group_ = Self::node_->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

  Self::msgs_map_ = std::unordered_map<std::string, std::any>();

  boost::unit_test::unit_test_log.set_stream(output_stream_);
}

ROS2ServiceTestFixture::~ROS2ServiceTestFixture() {
  boost::unit_test::unit_test_log.set_stream(std::cout);
  output_stream_ << std::flush;
  RCLCPP_ERROR_STREAM(Self::node_->get_logger(), Self::output_stream_.str());

  Self::executor_->spin_some();

  Self::executor_ = nullptr;
  Self::node_ = nullptr;
}