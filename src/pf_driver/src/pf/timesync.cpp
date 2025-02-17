#include <iostream>
#include <iomanip>
#include "pf_driver/pf/timesync.h"

#include <cmath>

TimeSync::TimeSync()
  : sensor_base_(0), pc_base_(0), base_time_(0), scale_time_(0), period_(0), linear_regression_(false)
{
}

void TimeSync::reset(double since)
{
  samples_.clear();
  sensor_base_ = rclcpp::Time(0); /* to be subtracted from sensor time before conversion */
  pc_base_ = rclcpp::Time(0);     /* to be added to converted sensor time result */
  base_time_ = 0.0;
  scale_time_ = 0.0; /* implicit + 1.0 */
  sum_req_duration_us_ = 0;

  RCLCPP_INFO(rclcpp::get_logger("timesync"), "reset %f", since);
}

void TimeSync::init(int period, int off_usec, bool linear_regression)
{
  reset(0.0);
  period_ = period;
  off_usec_ = 1.0E-6 * (double)off_usec;
  linear_regression_ = linear_regression;
}

bool TimeSync::valid(void)
{
  return (samples_.size() > 0);
}

void TimeSync::raw_to_rclcpp(uint64_t raw, rclcpp::Time& cppt, rcl_clock_type_t clock_type)
{
  int32_t s = (int32_t)((raw >> 32) & 0x7ffffffful);
  uint32_t ns = ((uint64_t)(raw & 0xfffffffful) * 1000000000ull) >> 32;

  cppt = rclcpp::Time(s, ns, clock_type);
}

void TimeSync::sensor_to_pc(uint64_t sensor_time_raw, rclcpp::Time& pc_time)
{
  rclcpp::Time sensor_time;
  raw_to_rclcpp(sensor_time_raw, sensor_time, pc_base_.get_clock_type());

  {
    std::lock_guard<std::mutex> guard(access_);

    auto reduced_sensor_time(sensor_time - sensor_base_);

    double sensor_seconds = reduced_sensor_time.seconds();
    std::chrono::duration<double> conv_time(base_time_ + off_usec_ + sensor_seconds + scale_time_ * sensor_seconds);

    pc_time = pc_base_ + rclcpp::Duration(conv_time);
  }

  RCLCPP_INFO(rclcpp::get_logger("timesync"), "calc: %f %f", sensor_time.seconds(), pc_time.seconds());
}

/* Compute the nanoseconds remaining until the system_time_raw reaches a full second (for HW timesync) */
long TimeSync::time_to_full_sensor_second(rclcpp::Time& pc_time)
{
  std::lock_guard<std::mutex> guard(access_);

  std::chrono::duration<double> conv_time(((pc_time - pc_base_).seconds() - off_usec_ - base_time_) /
                                          (1.0 + scale_time_));

  rclcpp::Time sensor_time = sensor_base_ + rclcpp::Duration(conv_time);

  /* Return remainder until full second on sensor */
  return 1000000000l - (sensor_time.nanoseconds() % 1000000000);
}

void TimeSync::update(uint64_t sensor_time_raw, unsigned req_duration_us, rclcpp::Time pc_time)
{
  TimeSync_Sample sample;

  if (req_duration_us > 50000)
  {
    return;
  }

  if (!samples_.empty())
  {
    /* Reset stats if there was a "large" jump in pc time, e.g. due to daylight savings?? */
    /* TBD: What is "large"? 15sec? 15min? */

    double since_last_update = (pc_time - samples_.back().pc_time).seconds();
    if (fabs(since_last_update) > 15.0)
    {
      reset(since_last_update);
    }
  }

  {
    double period_secs = 0.001 * (double)period_;

    raw_to_rclcpp(sensor_time_raw, sample.sensor_time, pc_time.get_clock_type());

    if (!samples_.empty())
    {
      double since_last_update = (sample.sensor_time - samples_.back().sensor_time).seconds();

      if (fabs(since_last_update) > 1.0)
      {
        /* Reset stats if there was a long time since last packet, probably some kind of restart */

        reset(since_last_update);
      }
      else
      {
        /* Do not blindly collect all samples, as that might cause the
         * collection to grow very large if user configured to observe input
         * over a long time at high scan rate.  Rather ignore new samples that
         * come in at a higher rate than necessary to collect just 200 samples
         * evenly distributed during the whole period.  TBD: 200 was chosen
         * somewhat arbitrarily.
         */

        if (since_last_update < period_secs / 200)
        {
          return;
        }
      }
    }

    /* Discard samples older than period_ */
    while (!samples_.empty() && ((sample.sensor_time - samples_.front().sensor_time).seconds()) > period_secs)
    {
      sum_req_duration_us_ -= samples_.front().req_duration_us;
      samples_.pop_front();
    }

    /* Record new sample */
    sample.req_duration_us = req_duration_us;
    sum_req_duration_us_ += req_duration_us;
    sample.pc_time = pc_time;
    samples_.push_back(sample);
  }

  /* Offsets to keep values small in calculation below */

  /* These have to be considered in addition to the computed
    offset and scale, see sensor_to_pc() method above */

  rclcpp::Time sensor_base(samples_.back().sensor_time);
  rclcpp::Time pc_base(samples_.back().pc_time);

  {
    double time_factor = 1.0;
    double base_time = 0.0;

    if (linear_regression_)
    {
      /* Linear regression */

      /* Compute sums */
      double sum_x = 0, sum_xx = 0, sum_y = 0, sum_xy = 0;
      for (auto it = samples_.begin(); it != samples_.end(); ++it)
      {
        double x = (it->sensor_time - sensor_base).seconds();
        double y = (it->pc_time - pc_base).seconds();

        sum_x += x;
        sum_xx += x * x;
        sum_y += y;
        sum_xy += x * y;
      }

      /* Compute base and coefficent */
      double n = (double)samples_.size();
      double den = (n * sum_xx - sum_x * sum_x);
      time_factor = (den == 0.0) ? 1.0 : ((n * sum_xy - sum_x * sum_y) / den);
      base_time = (sum_y - time_factor * sum_x) / n;

      RCLCPP_INFO(rclcpp::get_logger("timesync"), "regress: %f %f %f %f %f %f %f %f", sum_x, sum_xx, sum_y, sum_xy, den,
                  time_factor, base_time, n);
    }
    else
    {
      /* Average */

      base_time = 0.0;
      for (auto it = samples_.begin(); it != samples_.end(); ++it)
      {
        base_time += ((it->pc_time - pc_base) - (it->sensor_time - sensor_base)).seconds();
      }

      /* Compute linearity error simply from periods between last and first timestamp pairs. */
      if (samples_.size() > 1)
      {
        base_time /= (double)(samples_.size());

        double pc_period = (samples_.back().pc_time - samples_.front().pc_time).seconds();
        double sensor_period = (samples_.back().sensor_time - samples_.front().sensor_time).seconds();

        if (pc_period > 0.0)
        {
          time_factor = pc_period / sensor_period;
        }
      }
    }

    /* Update state, protected by access_ mutex */
    {
      std::lock_guard<std::mutex> guard(access_);

      sensor_base_ = sensor_base;
      pc_base_ = pc_base;
      base_time_ = base_time;
      scale_time_ = time_factor - 1.0;
    }
  }

  unsigned mean_req_duration_us = sum_req_duration_us_ / samples_.size();

  RCLCPP_INFO(rclcpp::get_logger("timesync"), "update: %f %f %f %f %f %f %u %u", sample.sensor_time.seconds(),
              sample.pc_time.seconds(), sensor_base.seconds(), pc_base.seconds(), base_time_, scale_time_,
              (unsigned)req_duration_us, (unsigned)samples_.size());
}
