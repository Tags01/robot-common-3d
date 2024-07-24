#include "ros2_service_test_fixture.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/logging.hpp>

#include <string>
#include <unordered_map>
#include <memory>

#include <boost/test/unit_test.hpp>

using Self = ROS2ServiceTestFixture;

rclcpp::Executor::SharedPtr Self::executor_;
rclcpp::NodeOptions Self::node_options_;
rclcpp::CallbackGroup::SharedPtr Self::callback_group_;
rclcpp::Node::SharedPtr Self::node_;
std::unordered_map<std::string, std::any> Self::msgs_map_;

ROS2ServiceTestFixture::ROS2ServiceTestFixture()
{
    const auto& master_test_suite = boost::unit_test::framework::master_test_suite();
    rclcpp::init(master_test_suite.argc, master_test_suite.argv);
    Self::node_options_.automatically_declare_parameters_from_overrides(true);
    Self::node_options_.allow_undeclared_parameters(true);

    Self::executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    Self::node_ = std::make_shared<rclcpp::Node>("filter_node", "test", node_options_);
    Self::executor_->add_node(node_);

    Self::callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);

    Self::msgs_map_ = std::unordered_map<std::string, std::any>();
    //boost::unit_test::unit_test_log.set_format(node->get_logger());
}

ROS2ServiceTestFixture::~ROS2ServiceTestFixture() {
//boost::unit_test::unit_test_log::set_strean(std::cout);

  rclcpp::shutdown(); // shutdown node
}