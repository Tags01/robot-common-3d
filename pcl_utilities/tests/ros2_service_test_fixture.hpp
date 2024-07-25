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

#include <boost/test/unit_test.hpp>

// need this because type information may be in an unreadable form otherwise
#include <boost/core/demangle.hpp>

static const std::unordered_map<rclcpp::FutureReturnCode, std::string> RCLCPP_FUTURE_STATUS_MAP {
  {rclcpp::FutureReturnCode::SUCCESS, "SUCCESS"},
  {rclcpp::FutureReturnCode::TIMEOUT, "TIMEOUT"},
  {rclcpp::FutureReturnCode::INTERRUPTED, "INTERRUPTED"}};

class ROS2ServiceTestFixture {
    private:
      static rclcpp::Executor::SharedPtr executor_;
      static rclcpp::Node::SharedPtr node_;
      static std::unordered_map<std::string, std::any> msgs_map_;
      static std::stringstream output_stream_;
      static rclcpp::CallbackGroup::SharedPtr callback_group_;
    public:
      ROS2ServiceTestFixture();

      virtual ~ROS2ServiceTestFixture();

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
      template<typename Message, size_t NUMBER>
      static const std::array<Message, NUMBER>& get_cached_messages(
        std::string topic,
        std::chrono::seconds wait = std::chrono::seconds(-1)
      ) {
          using MsgArray = std::array<Message, NUMBER>;

          if (msgs_map_.count(topic) == 0) {
              MsgArray msgs;

              for (auto& msg: msgs) {
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

            BOOST_ERROR("the passed type "s + incorrect_type + " does not match " + actual_type + "\n" + error.what());

            throw std::logic_error("unreachable statement");
          }
      }

    template<typename ServiceType>
    static typename ServiceType::Response::SharedPtr sync_send_request(
      std::string service_name,
      typename ServiceType::Request::SharedPtr request,
      std::chrono::seconds service_wait = std::chrono::seconds(-1),
      std::chrono::seconds call_wait = std::chrono::seconds(-1))
    {
      typename rclcpp::Client<ServiceType>::SharedPtr client =
        node_->create_client<ServiceType>(service_name, rmw_qos_profile_services_default, callback_group_);
      client->wait_for_service(service_wait);
      BOOST_ASSERT(client->service_is_ready());

      std::shared_future future = client->async_send_request(request);
      rclcpp::FutureReturnCode future_code =
        executor_->spin_until_future_complete(future, std::chrono::milliseconds(100));
      BOOST_REQUIRE_EQUAL(RCLCPP_FUTURE_STATUS_MAP.at(future_code), "SUCCESS");
      return future.get();
    }

    template<typename ServiceType, typename InputIterator, typename OutputIterator>
    static OutputIterator sync_send_requests_transform(
        const std::string& service_name,
        InputIterator begin, InputIterator end,
        OutputIterator dest_start,
        std::chrono::seconds service_wait = std::chrono::seconds(-1),
        std::chrono::seconds call_wait = std::chrono::seconds(-1))
    {
      typename rclcpp::Client<ServiceType>::SharedPtr
        client = node_->create_client<ServiceType>(service_name, rmw_qos_profile_services_default, callback_group_);
      client->wait_for_service(service_wait);
      BOOST_ASSERT(client->service_is_ready());

      std::vector<typename rclcpp::Client<ServiceType>::FutureAndRequestId>
        awaitable_responses;

      for (InputIterator request_iter = begin; request_iter != end; request_iter++) {
        awaitable_responses.push_back(client->async_send_request(*request_iter));
      }

      auto future_awaiter = [&call_wait](auto&& future) {
          rclcpp::FutureReturnCode result = executor_->spin_until_future_complete(future, std::chrono::milliseconds(10000));
          BOOST_REQUIRE_EQUAL(RCLCPP_FUTURE_STATUS_MAP.at(result), "SUCCESS");

          return future.get();
      };

      return std::transform(
        awaitable_responses.begin(), awaitable_responses.end(),
        dest_start, future_awaiter);
    }

    // template<Typename FutureLike>
    // static poll_until_future_complete(
    //   FutureLike future,
    //   std::chrono::nanoseconds poll_rate = std::chrono::nanoseconds(50)
    //   std::chrono::nanoseconds wait = std::chrono::nanoseconds(-1)) {
    //     if rclcpp::spin


    //   }

    static rclcpp::Node::SharedPtr get_node() {
      return node_;
    }

    static rclcpp::Executor::SharedPtr get_executor() {
      return executor_;
    }
};

#endif
