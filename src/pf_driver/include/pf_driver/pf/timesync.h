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
#define NUM_TIMESYNC_METHODS 4
#define TIMESYNC_METHOD_OFF         0
#define TIMESYNC_METHOD_SIMPLE      1
#define TIMESYNC_METHOD_AVERAGE     2
#define TIMESYNC_METHOD_REQUESTS    3

/* timesync_averaging */
#define NUM_TIMESYNC_AVERAGING 2
#define TIMESYNC_AVERAGING_MEAN       0
#define TIMESYNC_AVERAGING_REGRESSION 1

class TimeSync
{
private:
  std::deque<TimeSync_Sample> samples_;

  uint64_t sum_req_duration_us_;
  double off_usec_;
  double base_time_;
  double scale_time_;
  int period_;
  int averaging_;
  rclcpp::Time sensor_base_;
  rclcpp::Time pc_base_;

  std::mutex access_;

public:
  static const char* timesync_method_name[NUM_TIMESYNC_METHODS];
  static const char* timesync_averaging_name[NUM_TIMESYNC_AVERAGING];

  TimeSync();
  void init(int period, int off_usec, int averaging);
  void reset(double since);
  void update(uint64_t sensor_time_raw, unsigned req_duration, rclcpp::Time pc_time);
  bool valid(void);
  void raw_to_rclcpp(uint64_t raw, rclcpp::Time& cppt, rcl_clock_type_t clock_type);
  void sensor_to_pc(uint64_t raw, rclcpp::Time& cppt);
  long time_to_full_sensor_second(rclcpp::Time&);

  static int timesync_method_name_to_int(std::string& value);
  static int timesync_averaging_name_to_int(std::string& value);


};
