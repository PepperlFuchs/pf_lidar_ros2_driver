#pragma once

#include <deque>
#include <mutex>
//#include <chrono>

#include <rclcpp/rclcpp.hpp>

struct TimeSync_Sample
{
  /* The std::chrono::steady_clock time on the host when the request was sent */
  rclcpp::Time pc_time;

  /* The time reported by the scanner */
  rclcpp::Time sensor_time;

  /* The time it took since request was sent until response was received */
  unsigned req_duration_us;
};

/* timesync_method */
#define TIMESYNC_NONE 0
#define TIMESYNC_ADHOC 1
#define TIMESYNC_PACKET_AVERAGE 2
#define TIMESYNC_POLL_AVERAGE 3

class TimeSync
{
private:
  std::deque<TimeSync_Sample> samples_;

  uint64_t sum_req_duration_us_;
  double off_usec_;
  double base_time_;
  double scale_time_;
  int period_;
  bool linear_regression_;
  rclcpp::Time sensor_base_;
  rclcpp::Time pc_base_;

  std::mutex access_;

public:
  TimeSync();
  void init(int period, int off_usec, bool linear_regression);
  void reset(double since);
  void update(uint64_t sensor_time_raw, unsigned req_duration, rclcpp::Time pc_time);
  bool valid(void);
  void raw_to_rclcpp(uint64_t raw, rclcpp::Time& cppt, rcl_clock_type_t clock_type);
  void sensor_to_pc(uint64_t raw, rclcpp::Time& cppt);
  long time_to_full_sensor_second(rclcpp::Time&);
};
