#include <iostream>
#include <iomanip>
#include "pf_driver/pf/timesync.h"

#include <cmath>

/* Sensor to ROS time synchronization in PF driver

Class `TimeSync` (in `src/pf_driver/src/pf/timesync.cpp` and `src/pf_driver/include/pf/timesync.h`)
holds an array of `TimeSync_Sample`, each relating a particular `pc_time` to a `sensor_time` timestamp.

New timestamp pairs can be fed into an instance of `TimeSync` by calling its `update()` method. Using
the samples collected in this array, coefficients are computed for conversion between sensor timestamps
and PC timestamps, either using just averaging the offset and slope or using linear regression
(determined by driver parameter `timesync_regression`).

The class method `sensor_to_rclcpp()` is meant for conversion of sensor to ROS timestamps, using
these coefficients.

There are currently two instances of `TimeSync`.

The first, `passive_timesync` is updated each time when a scan data packet is
evaluated, using the sensor timestamp in the packet and the ROS time when
evaluation starts.  At that time of evaluation, an unknown duration has passed
since physical reception. The update frequency currently is determined
implicitly by the rate of scan data packets and a hardcoded upper limit of 10
Hz (packets within 100 ms after an update are ignored).

The second instance of `TimeSync`, `active_timesync`, is fed from a timer callback
that is set up in `pf_interface` to regularly trigger HTTP requests for sensor time
 and update the `TimeSync` instance with the results. Its frequency is
determined by the `timesync_interval` driver parameter (milliseconds). If the
request took longer than 50 ms, the result is ignored.

For ease of implementation, those `TimeSync` instances are part of the
`params_` `ScanParameter` object, because this object is within reach for both
the PF interface timer callback and during packet evaluation, although they
strictly aren't parameters but dynamic state.

The list of recorded `TimeSync_Sample` is reset whenever a non-zero
`scan_status` in a packet header is seen, indicating a change in sample rate, a
significant deviation from nominal `scan_frequency` or other problems. Also an
update later than one second after the previous one would cause a reset, or a
difference of more than 15s between current and previous ROS time. In general,
the time span covered by the collected samples can be configured in driver
parameter `timesync_period`.

### Changes to previous implementation

In short, previously, the driver put the time of evaluation of the first
packet of a scan into LaserScan.header.stamp.

Now it represents the time when the first sample was taken. It is converted
from sensor time into ROS time using offset and coefficient computed from
data obtained during preceding observation of the time relationship.

More precisely, in previous driver versions, the LaserScan `header.stamp` was
set to the `rclcpp::Clock().now() - scan_time` at the time when the first
contributing packet was parsed. Before `bugfix/laserscan-metadata`, the
`scan_time` was mistakenly always zero so effectively this represented the time
when the first packet of the scan was received (ie. a packet length later than
when the first sample of the first packet was taken). After
`bugfix/laserscan-metadata`, it becomes more complicated, because `scan_time`
is calculated correctly but the actual time correction should have been the
duration of the measurements in the packet, not a whole scan.



*/

TimeSync::TimeSync() : sensor_base_(0), pc_base_(0), base_time_(0), scale_time_(0), period_(0), averaging_(0)
{
}

const char* TimeSync::timesync_method_name[NUM_TIMESYNC_METHODS] = { "off", "simple", "average", "requests" };

const char* TimeSync::timesync_averaging_name[NUM_TIMESYNC_AVERAGING] = { "mean", "regression" };

int TimeSync::timesync_method_name_to_int(std::string& value)
{
  for (int i = 0; i < NUM_TIMESYNC_METHODS; ++i)
  {
    if (value.compare(TimeSync::timesync_method_name[i]) == 0)
    {
      return i;
    }
  }
  return -1;
}

int TimeSync::timesync_averaging_name_to_int(std::string& value)
{
  for (int i = 0; i < NUM_TIMESYNC_AVERAGING; ++i)
  {
    if (value.compare(TimeSync::timesync_averaging_name[i]) == 0)
    {
      return i;
    }
  }
  return -1;
}

void TimeSync::reset(double since)
{
  samples_.clear();
  sensor_base_ = rclcpp::Time(0); /* to be subtracted from sensor time before conversion */
  pc_base_ = rclcpp::Time(0);     /* to be added to converted sensor time result */
  base_time_ = 0.0;
  scale_time_ = 0.0; /* implicit + 1.0 */
  sum_req_duration_us_ = 0;

  RCLCPP_DEBUG(rclcpp::get_logger("timesync"), "reset %f", since);
}

void TimeSync::init(int period, int off_usec, int averaging)
{
  reset(0.0);
  period_ = period;
  off_usec_ = 1.0E-6 * (double)off_usec;
  averaging_ = averaging;
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

  RCLCPP_DEBUG(rclcpp::get_logger("timesync"), "calc: %f %f", sensor_time.seconds(), pc_time.seconds());
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

    if (averaging_ = TIMESYNC_AVERAGING_REGRESSION)
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

      RCLCPP_DEBUG(rclcpp::get_logger("timesync"), "regress: %f %f %f %f %f %f %f %f", sum_x, sum_xx, sum_y, sum_xy,
                   den, time_factor, base_time, n);
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

  RCLCPP_DEBUG(rclcpp::get_logger("timesync"), "update: %f %f %f %f %f %f %u %u", sample.sensor_time.seconds(),
               sample.pc_time.seconds(), sensor_base.seconds(), pc_base.seconds(), base_time_, scale_time_,
               (unsigned)req_duration_us, (unsigned)samples_.size());
}
