#include "ros2_test_fixture.hpp"

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

namespace robot_common_tests {

rclcpp::Node::SharedPtr GlobalFixture::node_ {nullptr};
std::unordered_map<std::string, std::any> GlobalFixture::msgs_map_{};
std::stringstream GlobalFixture::output_stream_{};

GlobalFixture::GlobalFixture() {
  boost::unit_test::master_test_suite_t& suite =
    boost::unit_test::framework::master_test_suite();

  rclcpp::init(suite.argc, suite.argv);

  node_ = std::make_shared<rclcpp::Node>("test_fixture_node_robot_common");
}

GlobalFixture::~GlobalFixture() {
  // finish up any remaining tasks on node, including RCLCPP_* printing tasks
  rclcpp::spin_some(node_);

  BOOST_TEST_INFO(
    "the static node object must not have a reference count greater than 1 "
    "at program termination. Otherwise the destructor may segfault per "
    "https://github.com/eProsima/Fast-DDS/issues/1906.");
  BOOST_CHECK_EQUAL(node_.use_count(), 1);

  // these values are automatically redirected to ROS
  // output any remaining buffered values, std::cerr no buffered
  std::cout << std::flush;
  std::clog << std::flush;

  rclcpp::shutdown();

  /*
    All nodes stored in a variable with a static lifetime must be explitiely
    freed to avoid a segfault. This is do to the use of other static lifetime variables within eProsima FastDDS (a dependent of rclcpp::Node) which lead to internal objects being freed before the destructor of the parent object is called.

    The following bug reports describe the issue in more detail:
      - https://github.com/eProsima/Fast-DDS/issues/1906
      - https://github.com/ros2/rclcpp/issues/1766

    The workaround is to set the std::shared_ptr to NULL
    assuming there are no other owthers, will force object destruction.
  */

  node_ = nullptr;
}
};  // namespace robot_common_tests