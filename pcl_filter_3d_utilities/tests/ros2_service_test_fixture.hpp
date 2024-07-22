#ifndef SERVICE_TEST_FIXTURE_HEADER_ID_7632763864872641
#define SERVICE_TEST_FIXTURE_HEADER_ID_7632763864872641

#include <rclcpp/wait_for_message.hpp>
#include <rclcpp/lifecycle_node.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/logging.hpp>

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
#include <mutex>
#include <atomic>
#include <condition_variable>

#include <boost/test/included/unit_test.hpp>
#include <boost/test/included/unit_test_framework.hpp>
// need this because type information may be in an unreadable form otherwise
#include <boost/core/demangle.hpp>

template <typename MutexType = std::mutex>
class FIFOMutex {
  private:
    MutexType mutex_;
    std::atomic<std::optional<thread::id>> owner_;
    std::queue<thread::id> thread_queue_;
  public:
    FIFOMutex(): mutex_(), owner_(std::nullopt), thread_queue_() {}
    void lock() {
      mutex_.lock();
      thread_queue_.push_back(
        std::this_thread::thread_id());
    }

    void unlock() {
      if (thread_queue_.size() > 0) {
        owner_.store(std::make_optional(thread_queue.front()));
        thread_queue.pop_front();
      }

      mutex_.unlock();
    }

    bool try_lock() {
      std::optional<std::thread_id> thread_id_;
      thread_id_ = _atomic.get();
      if (thread_id_ == std::this_thread::get_id())
        return mutex_.try_lock();
    }
}

class ROS2ServiceTestFixture {
    private:
      static rclcpp::NodeOptions node_options_ {};
      static rclcpp::Node::SharedPtr node_ {nullptr};
      static std::unordered_map<std::string, std::any> msgs_map_ {};
      static std::thread node_thread_ {};
      static std::mutex global_lock_ {};
    public:
        ROS2ServiceTestFixture()
        {
            rclcpp::init(master_test_suite.argc, master_test_suite.argv);
            node_options_.automatically_declare_parameters_from_overrides(true);
            node_options_.allow_undeclared_parameters(true);

            node_ = std::make_shared<rclcpp::Node>("filter_node", "test", node_options_);

            const auto& master_test_suite = boost::unit_test::framework::master_test_suite();
            //boost::unit_test::unit_test_log.set_format(node->get_logger());

            //spin the node in the background while the tests run
            node_thread_ = std::thread(std::bind(
              run_node_main, master_test_suite.argc, master_test_suite.argv));
        }

        static void run_node_main(int argc, char ** argv) {
          using std::chrono_literals::operator""ms;
          while (rclcpp::ok()) {
            global_lock_.lock();
            rclcpp::spin_some(lock);
            global_lock_.unlock();
            std::this_thread::sleep_for(10ms);
          };
        }

        virtual ~ROS2ServiceTestFixture() {
          //boost::unit_test::unit_test_log::set_strean(std::cout);
          std::lock_guard _guard(global_lock_);

          rclcpp::shutdown(); // shutdown node
          node_thread_.join(); // wait for thread to finish
        }


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
                MsgArray msgs{};

                for (auto& msg: msgs) {
                  rclcpp::wait_for_message<Message>(msg, node_, topic, wait);
                }
                msgs_map_[topic] = std::make_any(std::move(msgs));
                return msgs_map_[topic];
            }

            // allow case of bad cast to fallthrough
            try {
              return std::any_cast<MsgArray>(msgs_map_[topic]);
            } catch (std::bad_any_cast& error) {
              using std::string_literals::operator""s;

              std::string incorrect_type = std::quoted(boost::core::demangle(typeid(MsgArray).name()));
              std::string actual_type = std::quoted(boost::core::demangle(msgs_map_[topic].name()));

              BOOST_ERROR(node->get_logger_(), "the passed type "s + incorrect_type + " does not match " + actual_type)
            }
        }

    template<typename ServiceType, typename InputIterator, typename OutputIterator>
    static void ros_service_transform(
        const std::string& service_name,
        InputIterator begin, InputIterator end,
        OutputIterator dest_start,
        std::chrono::seconds service_wait = std::chrono::seconds(-1),
        std::chrono::seconds call_wait = std::chrono::seconds(-1))
    {
      rclcpp::Client::SharedPtr client = node_->create_client<ServiceType>(service_name);
      //client->wait_for_service(service_wait);

      std::vector<std::future<ResponseType>> awaitable_responses;

      for (InputIterator request_iter = begin; request_iter != end; request_iter++) {
        awaitable_responses.push_back(client->call_async(request_iter));
      }

      return std::transform(
        awaitable_responses.begin(), awaitable_responses.end(),
        dest_start,
        [&call_wait](auto& future) {
            return future.wait_for(call_wait);
        });
    }

    static rclcpp::Node::SharedPtr get_node() {
      return node_;
    }
};

#endif
