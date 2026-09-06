// SPDX-License-Identifier: MIT

#include <scx_slam_executor/freshness_executor.hpp>
#include <scx_slam_executor/freshqos.h>
#include <scx_slam_executor/application_stages.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{

class RecordingHintSink final : public scx_slam_executor::HintSink
{
public:
  std::function<void(const fresh_task_hint &)> after_publish;
  std::function<void()> after_clear;
  void publish(uint64_t worker_pid_tgid, const fresh_task_hint & hint) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      worker_ids_.push_back(worker_pid_tgid);
      hints_.push_back(hint);
      operations_.push_back(hint.job_id);
      published_.store(true);
    }
    if (after_publish) {after_publish(hint);}
  }

  void clear(uint64_t worker_pid_tgid) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cleared_worker_ids_.push_back(worker_pid_tgid);
      operations_.push_back(0);
    }
    if (after_clear) {after_clear();}
  }

  bool published() const
  {
    return published_.load();
  }

  std::vector<uint64_t> worker_ids() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return worker_ids_;
  }

  std::vector<uint64_t> cleared_worker_ids() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cleared_worker_ids_;
  }

  std::vector<fresh_task_hint> hints() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return hints_;
  }

  std::vector<uint64_t> operations() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return operations_;
  }

private:
  mutable std::mutex mutex_;
  std::atomic<bool> published_{false};
  std::vector<uint64_t> worker_ids_;
  std::vector<uint64_t> cleared_worker_ids_;
  std::vector<fresh_task_hint> hints_;
  std::vector<uint64_t> operations_;
};

}  // namespace

TEST(FreshnessExecutor, PublishesAssignedJobBeforeCallbackAndClearsAfterward)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("freshness_executor_test");
  auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sink = std::make_shared<RecordingHintSink>();
  scx_slam_executor::FreshnessExecutor executor(sink);

  scx_slam_executor::CallbackProfile profile;
  profile.stage_id = SLAM_STAGE_VISION_FE;
  profile.class_id = FRESH_CLASS_DEADLINE;
  profile.relative_deadline_ns = 33000000ULL;
  profile.stale_ns = 66000000ULL;
  profile.budget_ns = 12000000ULL;
  executor.add_callback_group_with_profile(group, node->get_node_base_interface(), profile);

  std::atomic<bool> callback_ran{false};
  std::atomic<uint64_t> callback_worker{0};
  const uint64_t dispatcher = freshqos_pid_tgid_self();
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1),
    [&]() {
      EXPECT_TRUE(sink->published());
      callback_worker.store(freshqos_pid_tgid_self());
      callback_ran.store(true);
      executor.cancel();
    },
    group);

  executor.spin();
  rclcpp::shutdown();

  ASSERT_TRUE(callback_ran.load());
  ASSERT_NE(executor.worker_pid_tgid(), 0U);
  EXPECT_EQ(callback_worker.load(), executor.worker_pid_tgid());
  EXPECT_NE(callback_worker.load(), dispatcher);
  const auto hints = sink->hints();
  const auto workers = sink->worker_ids();
  const auto cleared_workers = sink->cleared_worker_ids();
  ASSERT_EQ(hints.size(), 1U);
  ASSERT_EQ(workers.size(), 1U);
  ASSERT_EQ(cleared_workers.size(), 1U);
  EXPECT_EQ(workers.front(), executor.worker_pid_tgid());
  EXPECT_EQ(cleared_workers.front(), executor.worker_pid_tgid());
  EXPECT_EQ(hints.front().api_version, FRESH_API_VERSION);
  EXPECT_EQ(hints.front().stage_id, SLAM_STAGE_VISION_FE);
  EXPECT_EQ(hints.front().class_id, FRESH_CLASS_DEADLINE);
  EXPECT_EQ(hints.front().job_id, 1U);
  EXPECT_EQ(
    hints.front().deadline_ts_ns - hints.front().release_ts_ns,
    profile.relative_deadline_ns);
  EXPECT_EQ(hints.front().stale_ns, profile.stale_ns);
  EXPECT_EQ(hints.front().budget_ns, profile.budget_ns);
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{1, 0}));
}

TEST(FreshnessExecutor, ClearsAssignedJobWhenCallbackThrows)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("freshness_executor_throw_test");
  auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sink = std::make_shared<RecordingHintSink>();
  scx_slam_executor::FreshnessExecutor executor(sink);
  executor.add_callback_group_with_profile(
    group, node->get_node_base_interface(), scx_slam_executor::CallbackProfile{});

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1),
    []() {throw std::runtime_error("expected callback failure");},
    group);

  EXPECT_THROW(executor.spin(), std::runtime_error);
  rclcpp::shutdown();

  EXPECT_EQ(sink->hints().size(), 1U);
  EXPECT_EQ(sink->cleared_worker_ids().size(), 1U);
}

TEST(FreshnessExecutor, ReplacesOnlyCompletedJobsWhileDraining)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("freshness_executor_drain_test");
  auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sink = std::make_shared<RecordingHintSink>();
  scx_slam_executor::FreshnessExecutor executor(sink);
  executor.add_callback_group_with_profile(
    group, node->get_node_base_interface(), scx_slam_executor::CallbackProfile{});

  std::atomic<unsigned int> callback_count{0};
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1),
    [&]() {
      if (++callback_count == 3) {
        executor.cancel();
      }
    },
    group);

  executor.spin();
  rclcpp::shutdown();

  EXPECT_EQ(callback_count.load(), 3U);
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{1, 2, 3, 0}));
}

TEST(FreshnessExecutor, TakesMessageMetadataBeforeWakingSubscriptionWorker)
{
  using Message = std_msgs::msg::UInt64MultiArray;

  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("freshness_executor_message_test");
  auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sink = std::make_shared<RecordingHintSink>();
  scx_slam_executor::FreshnessExecutor executor(sink);

  std::atomic<uint64_t> callback_thread{0};
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  auto subscription = node->create_subscription<Message>(
    "freshness_executor_message", rclcpp::QoS(10),
    [&](const Message::SharedPtr message) {
      EXPECT_EQ(message->data.at(0), 77U);
      callback_thread.store(freshqos_pid_tgid_self());
      executor.cancel();
    },
    options);

  scx_slam_executor::CallbackProfile profile;
  profile.stage_id = SLAM_STAGE_STATE_EST;
  profile.class_id = FRESH_CLASS_DEADLINE;
  profile.relative_deadline_ns = 20000000ULL;
  std::atomic<uint64_t> extractor_thread{0};
  executor.add_subscription_callback_group_with_profile<Message>(
    group, node->get_node_base_interface(), profile,
    [&](const Message & message) {
      extractor_thread.store(freshqos_pid_tgid_self());
      return scx_slam_executor::MessageMetadata{message.data.at(0), message.data.at(1)};
    });

  auto publisher = node->create_publisher<Message>("freshness_executor_message", rclcpp::QoS(10));
  Message message;
  message.data = {77, 123456789ULL};
  std::thread publisher_thread([&]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      publisher->publish(message);
    });

  executor.spin();
  publisher_thread.join();
  rclcpp::shutdown();

  ASSERT_EQ(sink->hints().size(), 1U);
  const auto hint = sink->hints().front();
  EXPECT_EQ(hint.job_id, 77U);
  EXPECT_EQ(hint.release_ts_ns, 123456789ULL);
  EXPECT_EQ(hint.deadline_ts_ns, 143456789ULL);
  EXPECT_EQ(hint.stage_id, SLAM_STAGE_STATE_EST);
  EXPECT_EQ(callback_thread.load(), executor.worker_pid_tgid());
  EXPECT_NE(extractor_thread.load(), callback_thread.load());
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{77, 0}));
}

namespace
{
uint64_t now_ns()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

// Real DDS take/return and mutually exclusive callback-group ownership. The
// timeout is only a failure escape, never the success condition of these tests.
class AdmissionTest : public ::testing::Test
{
protected:
  using Message = std_msgs::msg::UInt64MultiArray;
  void SetUp() override
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
    node = std::make_shared<rclcpp::Node>("admission_test");
    group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    sink = std::make_shared<RecordingHintSink>();
    executor = std::make_unique<scx_slam_executor::FreshnessExecutor>(sink);
    profile.stage_id = SLAM_STAGE_STATE_EST;
    profile.class_id = FRESH_CLASS_DEADLINE;
    profile.relative_deadline_ns = 20000000ULL;
    profile.stale_ns = 40000000ULL;
    profile.budget_ns = 10000000ULL;
    profile.reject_expired = true;
  }
  void TearDown() override {rclcpp::shutdown();}
  void run(std::function<void(Message::SharedPtr)> callback,
    scx_slam_executor::MessageObserver observer, unsigned count, bool all_fresh = false)
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;
    auto subscription = node->create_subscription<Message>("admission", rclcpp::QoS(10),
      std::move(callback), options);
    executor->add_subscription_callback_group_with_profile<Message>(
      group, node->get_node_base_interface(), profile,
      [](const Message & message) {
        return scx_slam_executor::MessageMetadata{
          message.data[0], message.data[1] ? now_ns() : 1ULL};
      }, true, std::move(observer));
    auto publisher = node->create_publisher<Message>("admission", rclcpp::QoS(10));
    std::atomic<bool> done{false};
    std::thread producer([&]() {
        for (int i = 0; i < 200 && publisher->get_subscription_count() == 0; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        for (unsigned i = 1; i <= count; ++i) {
          Message message;
          message.data = {i, all_fresh || i == count ? 1ULL : 0ULL};
          publisher->publish(message);
        }
        for (int i = 0; i < 400 && !done; ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!done) {executor->cancel();}
      });
    try {
      executor->spin();
    } catch (...) {
      done = true;
      producer.join();
      throw;
    }
    done = true;
    producer.join();
  }
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::CallbackGroup::SharedPtr group;
  std::shared_ptr<RecordingHintSink> sink;
  std::unique_ptr<scx_slam_executor::FreshnessExecutor> executor;
  scx_slam_executor::CallbackProfile profile;
};
}

TEST_F(AdmissionTest, RejectsStaleBacklogBeforePublicationAndRecovers)
{
  // The former sensor-special ID must obey this Deadline profile's admission.
  profile.stage_id = 0;
  unsigned selected = 0, dropped = 0, completed = 0;
  std::weak_ptr<void> dropped_message;
  run([&](Message::SharedPtr message) {
      EXPECT_EQ(message->data[0], 4U);
      EXPECT_EQ(dropped, 3U);
      EXPECT_TRUE(dropped_message.expired());
      completed++;
      executor->cancel();
    }, [&](const std::shared_ptr<void> & message, scx_slam_executor::MessageEvent event) {
      EXPECT_FALSE(group->can_be_taken_from().load());
      if (event == scx_slam_executor::MessageEvent::Selected) {selected++;}
      else {
        dropped++;
        dropped_message = message;
        EXPECT_TRUE(sink->hints().empty());
      }
    }, 4);
  EXPECT_EQ(selected, 4U);
  EXPECT_EQ(selected, completed + dropped);
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{4, 0}));
  EXPECT_TRUE(group->can_be_taken_from().load());
}

TEST_F(AdmissionTest, ReleasesMessageAndGroupWhenDropObserverThrows)
{
  std::weak_ptr<void> dropped_message;
  EXPECT_THROW(run([](Message::SharedPtr) {ADD_FAILURE();},
    [&](const std::shared_ptr<void> & message, scx_slam_executor::MessageEvent event) {
      if (event == scx_slam_executor::MessageEvent::DroppedBeforeStart) {
        dropped_message = message;
        throw std::runtime_error("drop observer failure");
      }
    }, 2), std::runtime_error);
  EXPECT_TRUE(dropped_message.expired());
  EXPECT_TRUE(group->can_be_taken_from().load());
  EXPECT_TRUE(sink->operations().empty());
}

TEST_F(AdmissionTest, RechecksAfterHandoffDelayWithoutRunningExpiredCallback)
{
  sink->after_publish = [](const fresh_task_hint & hint) {
      EXPECT_EQ(hint.flags, 0U);  // Admission does not require a kernel flag.
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    };
  unsigned selected = 0, dropped = 0;
  run([](Message::SharedPtr) {ADD_FAILURE() << "expired callback ran";},
    [&](const std::shared_ptr<void> &, scx_slam_executor::MessageEvent event) {
      if (event == scx_slam_executor::MessageEvent::Selected) {selected++;}
      else {dropped++; executor->cancel();}
    }, 1);
  EXPECT_EQ(selected, 1U);
  EXPECT_EQ(dropped, 1U);
  EXPECT_TRUE(group->can_be_taken_from().load());
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{1, 0}));
}

TEST_F(AdmissionTest, KeepsInFlightIdentityAndBudgetThroughLateCallbackTail)
{
  unsigned completed = 0;
  run([&](Message::SharedPtr message) {
      const auto before = sink->hints().back();
      EXPECT_FALSE(group->can_be_taken_from().load());
      EXPECT_EQ(before.flags, 0U);
      EXPECT_EQ(before.budget_ns, profile.budget_ns);
      if (message->data[0] == 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        EXPECT_GT(now_ns(), before.deadline_ts_ns);
        EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{1}));
        EXPECT_EQ(sink->hints().back().job_id, before.job_id);
      }
      completed++;
      if (completed == 2) {executor->cancel();}
    }, {}, 2, true);
  EXPECT_EQ(completed, 2U);
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{1, 2, 0}));
}

TEST_F(AdmissionTest, ApplicationCanRetainExpiredWork)
{
  profile.stage_id = 12345;
  profile.reject_expired = false;
  unsigned completed = 0;
  run([&](Message::SharedPtr) {
      EXPECT_EQ(sink->hints().back().flags, 0U);
      if (++completed == 2) {executor->cancel();}
    }, [](const std::shared_ptr<void> &, scx_slam_executor::MessageEvent event) {
      EXPECT_EQ(event, scx_slam_executor::MessageEvent::Selected);
    }, 2);
  EXPECT_EQ(completed, 2U);
}

TEST_F(AdmissionTest, ApplicationCanExpireUrgentWork)
{
  profile.class_id = FRESH_CLASS_URGENT;
  unsigned dropped = 0, completed = 0;
  run([&](Message::SharedPtr message) {
      EXPECT_EQ(message->data[0], 2U);
      EXPECT_EQ(dropped, 1U);
      completed++;
      executor->cancel();
    }, [&](const std::shared_ptr<void> &, scx_slam_executor::MessageEvent event) {
      if (event == scx_slam_executor::MessageEvent::DroppedBeforeStart) {
        dropped++;
        EXPECT_TRUE(sink->hints().empty());
      }
    }, 2);
  EXPECT_EQ(completed + dropped, 2U);
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{2, 0}));
  EXPECT_TRUE(group->can_be_taken_from().load());
}

TEST_F(AdmissionTest, ReplacesCompletedSlotDirectlyAndClearsOnShutdown)
{
  const auto dispatcher = freshqos_pid_tgid_self();
  std::atomic<unsigned> completed{0};
  unsigned cleared = 0;
  std::weak_ptr<void> message_reference;
  sink->after_publish = [&](const fresh_task_hint & hint) {
      EXPECT_EQ(freshqos_pid_tgid_self(), dispatcher);
      EXPECT_EQ(completed.load(), hint.job_id - 1);
      EXPECT_TRUE(message_reference.expired());
      EXPECT_EQ(cleared, 0U);
    };
  sink->after_clear = [&]() {
      EXPECT_EQ(freshqos_pid_tgid_self(), dispatcher);
      EXPECT_EQ(completed.load(), 2U);
      ++cleared;
      EXPECT_TRUE(message_reference.expired());
      EXPECT_TRUE(group->can_be_taken_from().load());
    };
  run([&](Message::SharedPtr message) {
      message_reference = message;
      if (++completed == 2) {executor->cancel();}
    }, {}, 2, true);
  EXPECT_EQ(cleared, 1U);
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{1, 2, 0}));
}
