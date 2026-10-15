#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/msg/log.hpp>

#include "pf_driver/pf/pf_interface.h"
#include "pf_driver/ros/drop_diagnostics.h"

namespace
{

using namespace std::chrono_literals;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

std::string value(const DiagnosticStatus& status, const std::string& key)
{
  for (const auto& entry : status.values)
  {
    if (entry.key == key)
    {
      return entry.value;
    }
  }
  return "missing";
}

TEST(DiagnosticStatus, ReportsExactTotalsAndOnlyNewDropDeltas)
{
  DropSnapshot before;
  before.drops = { 2, 3, 5 };
  DropSnapshot after;
  after.drops = { 4, 4, 9 };
  after.scans_published = 9007199254740993ULL;
  after.packet_queue_capacity = 127;
  auto status = dropDiagnosticStatus(after, before, "driver: dropped scans and packets", "R2000 127.0.0.1");
  EXPECT_EQ(status.name, "driver: dropped scans and packets");
  EXPECT_EQ(status.hardware_id, "R2000 127.0.0.1");
  EXPECT_EQ(status.level, DiagnosticStatus::WARN);
  EXPECT_EQ(status.message, "3 scans and 4 packets affected by drops since the previous update");
  ASSERT_EQ(status.values.size(), 5u);
  EXPECT_EQ(value(status, "scans_published"), "9007199254740993");
  EXPECT_EQ(value(status, "scans_dropped_missing_first_packet"), "4");
  EXPECT_EQ(value(status, "scans_dropped_incomplete"), "4");
  EXPECT_EQ(value(status, "packets_dropped_queue_full"), "9");
  EXPECT_EQ(value(status, "packet_queue_capacity"), "127");
  status = dropDiagnosticStatus(after, after, "driver", "ip");
  EXPECT_EQ(status.level, DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "0 scans and 0 packets affected by drops since the previous update");
}

class DropDiagnosticsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    static std::atomic<unsigned> serial{ 0 };
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions initialization;
    initialization.set_domain_id(213);
    context_->init(0, nullptr, initialization);
    rclcpp::NodeOptions options;
    options.context(context_).use_global_arguments(false);
    options.parameter_overrides(
        { rclcpp::Parameter("use_sim_time", true), rclcpp::Parameter("diagnostics.period", 0.02) });
    node_ = std::make_shared<rclcpp::Node>("pf_test_" + std::to_string(serial.fetch_add(1)), options);
    node_->declare_parameter<std::string>("scanner_ip", "127.0.0.1");
    peer_ = std::make_shared<rclcpp::Node>("diagnostic_reader", rclcpp::NodeOptions().context(context_));
    status_subscription_ = peer_->create_subscription<DiagnosticArray>("/diagnostics", rclcpp::QoS(100).reliable(),
                                                                       [this](const DiagnosticArray& message) {
                                                                         std::lock_guard<std::mutex> lock(mutex_);
                                                                         messages_.push_back(message);
                                                                         changed_.notify_all();
                                                                       });
    log_subscription_ = peer_->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", rclcpp::QoS(100).reliable().transient_local(), [this](const rcl_interfaces::msg::Log& message) {
          if (message.name == node_->get_logger().get_name())
          {
            std::lock_guard<std::mutex> lock(mutex_);
            logs_.push_back(message.msg);
            changed_.notify_all();
          }
        });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(executor_options, 3);
    executor_->add_node(node_);
    executor_->add_node(peer_);
  }

  void TearDown() override
  {
    stop();
    executor_.reset();
    blocked_timer_.reset();
    owner_.reset();
    diagnostics_.reset();
    status_subscription_.reset();
    log_subscription_.reset();
    peer_.reset();
    node_.reset();
    context_->shutdown("diagnostics test completed");
    context_.reset();
  }

  void start()
  {
    spin_ = std::thread([this] { executor_->spin(); });
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(pause_mutex_);
      release_ = true;
      pause_changed_.notify_all();
    }
    stop_metadata_ = true;
    if (metadata_.joinable())
    {
      metadata_.join();
    }
    if (executor_)
    {
      executor_->cancel();
    }
    if (spin_.joinable())
    {
      spin_.join();
    }
  }

  template <typename Predicate>
  bool waitFor(Predicate predicate)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, 5s, predicate);
  }

  bool hasLog(const std::string& text) const
  {
    return std::find(logs_.begin(), logs_.end(), text) != logs_.end();
  }

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> node_, peer_;
  std::unique_ptr<DropDiagnostics> diagnostics_;
  std::unique_ptr<PFInterface> owner_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr status_subscription_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_subscription_;
  rclcpp::TimerBase::SharedPtr blocked_timer_;
  std::thread spin_, metadata_;
  std::atomic<bool> stop_metadata_{ false };
  std::mutex mutex_;
  std::condition_variable changed_;
  std::vector<DiagnosticArray> messages_;
  std::vector<std::string> logs_;
  std::mutex pause_mutex_;
  std::condition_variable pause_changed_;
  bool entered_ = false;
  bool release_ = false;
  bool pause_timed_out_ = false;
};

TEST_F(DropDiagnosticsTest, PFInterfacePublishesBeforeDeviceInitializationWithStoppedRosTime)
{
  owner_ = std::make_unique<PFInterface>(node_);
  ASSERT_EQ(node_->now().nanoseconds(), 0);
  start();
  ASSERT_TRUE(waitFor([&] { return messages_.size() >= 3; }));
  stop();
  for (const auto& message : messages_)
  {
    ASSERT_EQ(message.status.size(), 1u);
    EXPECT_EQ(message.header.stamp.sec, 0);
    EXPECT_EQ(message.header.stamp.nanosec, 0u);
    EXPECT_EQ(message.status.front().hardware_id, "127.0.0.1");
    EXPECT_EQ(message.status.front().level, DiagnosticStatus::OK);
    EXPECT_EQ(value(message.status.front(), "scans_published"), "0");
    EXPECT_EQ(value(message.status.front(), "packet_queue_capacity"), "0");
  }
}

TEST_F(DropDiagnosticsTest, SummariesFlushAfterInputStopsDespiteBlockedDefaultCallbacks)
{
  diagnostics_ = std::make_unique<DropDiagnostics>(node_);
  auto counters = diagnostics_->counters();
  // Before spinning, the first report and four pending drops are deterministic.
  for (int i = 0; i < 5; ++i)
  {
    counters->recordDrop(DropReason::MISSING_FIRST_PACKET, { 9, 2 });
  }
  counters->recordPublication();
  counters->recordPublication();
  counters->setQueueCapacity(127);
  blocked_timer_ = node_->create_wall_timer(1ms, [this] {
    std::unique_lock<std::mutex> lock(pause_mutex_);
    entered_ = true;
    pause_changed_.notify_all();
    if (!pause_changed_.wait_for(lock, 5s, [this] { return release_; }))
    {
      pause_timed_out_ = true;
    }
  });
  start();
  {
    std::unique_lock<std::mutex> lock(pause_mutex_);
    ASSERT_TRUE(pause_changed_.wait_for(lock, 5s, [this] { return entered_; }));
  }
  const std::string first = "Discarded packet 2 of scan 9: its first packet has not started a scan (total 1)";
  const std::string summary = "scans_dropped_missing_first_packet: 4 additional drops since the previous summary; "
                              "total 5";
  ASSERT_TRUE(waitFor([&] { return hasLog(first) && hasLog(summary) && messages_.size() >= 3; }));
  const auto after_flush = std::chrono::steady_clock::now();
  // Observe another wall-timer interval without further input.
  ASSERT_TRUE(waitFor([&] { return std::chrono::steady_clock::now() - after_flush >= 1100ms; }));
  stop();
  EXPECT_FALSE(pause_timed_out_);
  EXPECT_EQ(std::count(logs_.begin(), logs_.end(), first), 1);
  EXPECT_EQ(std::count(logs_.begin(), logs_.end(), summary), 1);
  EXPECT_EQ(logs_.size(), 2u);
  ASSERT_FALSE(messages_.empty());
  for (const auto& message : messages_)
  {
    ASSERT_EQ(message.status.size(), 1u);
    EXPECT_EQ(message.header.stamp.sec, 0);
    EXPECT_EQ(message.header.stamp.nanosec, 0u);
    EXPECT_EQ(value(message.status.front(), "scans_published"), "2");
    EXPECT_EQ(value(message.status.front(), "scans_dropped_missing_first_packet"), "5");
    EXPECT_EQ(value(message.status.front(), "packet_queue_capacity"), "127");
  }
  EXPECT_EQ(messages_.back().status.front().level, DiagnosticStatus::OK);
}

TEST_F(DropDiagnosticsTest, ConcurrentHardwareMetadataStaysCoherent)
{
  diagnostics_ = std::make_unique<DropDiagnostics>(node_);
  const std::string a = "R2000 " + std::string(1024, 'a');
  const std::string b = "R2300 " + std::string(1024, 'b');
  diagnostics_->setHardwareId(a);
  metadata_ = std::thread([this, a, b] {
    while (!stop_metadata_.load())
    {
      diagnostics_->setHardwareId(a);
      diagnostics_->setHardwareId(b);
      std::this_thread::yield();
    }
  });
  start();
  ASSERT_TRUE(waitFor([&] { return messages_.size() >= 20; }));
  stop();
  for (const auto& message : messages_)
  {
    ASSERT_EQ(message.status.size(), 1u);
    const auto& hardware = message.status.front().hardware_id;
    EXPECT_TRUE(hardware == a || hardware == b);
  }
}

TEST_F(DropDiagnosticsTest, DiagnosticsPeriodIsReadOnlyAndInvalidPeriodsFail)
{
  diagnostics_ = std::make_unique<DropDiagnostics>(node_);
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("diagnostics.period", 0.5)).successful);
  for (double period :
       { 0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN(), 1e-12, 1e12 })
  {
    rclcpp::NodeOptions options;
    options.context(context_).parameter_overrides({ rclcpp::Parameter("diagnostics.period", period) });
    auto invalid = std::make_shared<rclcpp::Node>("invalid_diagnostics_period", options);
    EXPECT_THROW({ DropDiagnostics diagnostics(invalid); }, std::invalid_argument);
  }
}

}  // namespace
