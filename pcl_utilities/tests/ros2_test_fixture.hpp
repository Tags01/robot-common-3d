#ifndef SERVICE_TEST_FIXTURE_HEADER_ID_7632763864872641
#define SERVICE_TEST_FIXTURE_HEADER_ID_7632763864872641

#include <rclcpp/wait_for_message.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/logging.hpp>
#include <iostream>

#include <any>
#include <array>
#include <optional>
#include <cstdint>
#include <random>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <future>
#include <utility>
#include <sstream>
#include <ratio>

#include <boost/test/unit_test.hpp>

// need this because type information may be in an unreadable form otherwise
#include <boost/core/demangle.hpp>

namespace robot_common_tests {

namespace internal_ns {
  static const std::unordered_map<rclcpp::FutureReturnCode, std::string> RCLCPP_FUTURE_STATUS_MAP {
    {rclcpp::FutureReturnCode::SUCCESS, "SUCCESS"},
    {rclcpp::FutureReturnCode::TIMEOUT, "TIMEOUT"},
    {rclcpp::FutureReturnCode::INTERRUPTED, "INTERRUPTED"}};

  /**
   * @template-param RatioT
   *  an instance of std::ratio<>
   * @returns
   *  printable string of the unit of time
   *
   * @description
   *  This is a helper function meant to convert a time ratio type to
   *  its corresponding unit name
   *
   * @purpose
   *  This is meant to increase the quality of error messages
   *  by providing the associated unit of time
   */
  template <typename RatioT>
  constexpr std::string_view time_to_unit_string() {
    if constexpr (std::ratio_equal<RatioT, std::chrono::hours::period>())
      return "hours";
    else if (std::ratio_equal<RatioT, std::chrono::minutes::period>())
      return "minutes";
    else if (std::ratio_equal<RatioT, std::chrono::seconds::period>())
      return "seconds";
    else if (std::ratio_equal<RatioT, std::chrono::milliseconds::period>())
      return "milliseconds";
    else if (std::ratio_equal<RatioT, std::chrono::microseconds::period>())
      return "microseconds";
    else if (std::ratio_equal<RatioT, std::chrono::nanoseconds::period>())
      return "nanoseconds";
    return "unknown time units";
  }
};  // namespace internal

class GlobalFixture {
    private:
      static rclcpp::Node::SharedPtr node_;
      static std::unordered_map<std::string, std::any> msgs_map_;
      static std::stringstream output_stream_;
    public:
      GlobalFixture();

      virtual ~GlobalFixture();

      /**
       *
       * @decription:
       *  This function supports recording a consistant fixed number of messages that
       *  will remain the same for the input combination.
       *
       * The NUMBER size type if fixed since the number of messages retrieved each
       * time should be fixed. If a variable number of messages were allowed and a
       * first test requested 5 messages and another requested 10. The state of the message
       * cache would change in a way that would introduce side-effects.
       *
       * RTTI is used to detect issues with types at runtime since no C++ facility exists
       * to detect errors at compile time.
       * boost::core::demangle is used to extract the type in a human readable format.
       *
       * @purpose
       *  This function is used to minimize boilerplate in testing code
       *
       */
      template<
        typename Message, size_t NUMBER,
        typename TimeRep = uint64_t, typename RatioT = std::milli
      >
      static const std::array<Message, NUMBER>& get_cached_messages(
        std::string topic,
        std::chrono::duration<TimeRep, RatioT> wait = std::chrono::milliseconds(200)
      ) {
          namespace internal_ns = ::robot_common_tests::internal_ns;
          using MsgArray = std::array<Message, NUMBER>;

          if (msgs_map_.count(topic) == 0) {
              MsgArray msgs;

              for (auto& msg: msgs) {
                BOOST_TEST_INFO(
                  "The message could not be recieved on the current topic within the duration: "
                  << wait.count() << " " << internal_ns::time_to_unit_string<RatioT>()
                  << ". Double check the topic name and status of the publisher.");
                BOOST_REQUIRE(
                  rclcpp::wait_for_message<Message>(msg, node_, topic, wait));
              }

              msgs_map_[topic] = std::any(std::move(msgs));
              return std::any_cast<MsgArray&>(msgs_map_[topic]);
          }

          // allow case of bad cast to fallthrough
          try {
            return std::any_cast<MsgArray&>(msgs_map_[topic]);
          } catch (std::bad_any_cast& error) {
            using std::string_literals::operator""s;

            std::string incorrect_type = boost::core::demangle(typeid(MsgArray).name());
            std::string actual_type = boost::core::demangle(msgs_map_[topic].type().name());

            BOOST_TEST_MESSAGE(
              "the passed type does not match the cached type\n\t"
              << "Passed Parameter Type:  `" << incorrect_type << "`"
              << "Old Cached Message Type:  `" << std::quoted(actual_type) << "`"
              << "\nCheck that the `TYPE` and `LENGTH` template "
              << "arguments are the same for all other invocations with this "
              << "topic\n");

            BOOST_REQUIRE(false);
            throw error;
          }
      }

    template<
      typename ServiceType,
      typename TimeRep1 = uint64_t, typename RatioT1 = std::milli,
      typename TimeRep2 = uint64_t, typename RatioT2 = std::milli>
    static typename ServiceType::Response::SharedPtr sync_send_request(
      std::string service_name,
      typename ServiceType::Request::SharedPtr request,
      std::chrono::duration<TimeRep1, RatioT1> service_wait = std::chrono::milliseconds(200),
      std::chrono::duration<TimeRep2, RatioT2> call_wait = std::chrono::milliseconds(200))
    {
      namespace internal_ns = ::robot_common_tests::internal_ns;
      typename rclcpp::Client<ServiceType>::SharedPtr client =
        node_->create_client<ServiceType>(service_name);

      client->wait_for_service(service_wait);
      BOOST_TEST_INFO("The service " << std::quoted(service_name)
        << " could not be found within the expected duration: "
        << service_wait.count() << " "
        << internal_ns::time_to_unit_string<RatioT2>());

      BOOST_REQUIRE(client->service_is_ready());

      typename rclcpp::Client<ServiceType>::FutureAndRequestId future =
        client->async_send_request(request);
      rclcpp::FutureReturnCode future_code =
        rclcpp::spin_until_future_complete(node_, future, call_wait);

        BOOST_TEST_INFO(
          "Did not recieve a result message from " << std::quoted(service_name)
          << " within the expected duration: " << call_wait.count() << " "
          << internal_ns::time_to_unit_string<RatioT2>);

        BOOST_REQUIRE_EQUAL(
          internal_ns::RCLCPP_FUTURE_STATUS_MAP.at(future_code),
          "SUCCESS");

      return future.get();
    }

    static rclcpp::Node::SharedPtr get_node() {
      return node_;
    }
};

}; // namespace internal

#endif
