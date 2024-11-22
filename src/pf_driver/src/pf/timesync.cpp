#include <iostream>
#include <iomanip>
#include "pf_driver/pf/timesync.h"

TimeSync::TimeSync() :
    sensor_base_(0), pc_base_(0),
    base_time_(0), scale_time_(0)
{
}

void TimeSync::reset(double since)
{
    samples_.clear();
    sensor_base_ = rclcpp::Time(0); /* to be subtracted from sensor time before conversion */
    pc_base_ = rclcpp::Time(0); /* to be added to converted sensor time result */
    base_time_ = 0.0;
    scale_time_ = 0.0; /* implicit + 1.0 */
    sum_req_duration_us_ = 0;

    RCLCPP_INFO(rclcpp::get_logger("timesync"), "reset %f", since);
}

void TimeSync::raw_to_rclcpp(uint64_t raw, rclcpp::Time& cppt)
{
    int32_t s = (int32_t)((raw >> 32) & 0x7ffffffful);
    uint32_t ns = ((uint64_t)(raw & 0xfffffffful) * 1000000000ull) >> 32;
    cppt = rclcpp::Time(s, ns, RCL_STEADY_TIME);
}

void TimeSync::sensor_to_pc(uint64_t sensor_time_raw, rclcpp::Time& pc_time)
{
    rclcpp::Time sensor_time;
    raw_to_rclcpp(sensor_time_raw, sensor_time);

    {
        std::lock_guard<std::mutex> guard(access_);

        auto reduced_sensor_time(sensor_time - sensor_base_);

        double sensor_seconds = reduced_sensor_time.seconds();
        std::chrono::duration<double> conv_time(base_time_ + sensor_seconds + scale_time_ * sensor_seconds);

        pc_time = pc_base_ + rclcpp::Duration(conv_time);
    }

    RCLCPP_INFO(rclcpp::get_logger("timesync"), "calc: %f %f",
        sensor_time.seconds(),
        pc_time.seconds());
}

void TimeSync::update(uint64_t sensor_time_raw, unsigned req_duration_us, rclcpp::Time pc_time)
{
    TimeSync_Sample sample;

    if (req_duration_us > 50000)
    {
        return;
    }

    {
        raw_to_rclcpp(sensor_time_raw, sample.sensor_time);

        if (samples_.size() > 0)
        {
            double since_last_update = (sample.sensor_time - samples_.back().sensor_time).seconds();
            if (since_last_update < 0.1)
            {
                return;
            }
            if (since_last_update > 1.0)
            {
                reset(since_last_update);
            }
        }

        sample.req_duration_us = req_duration_us;
        sum_req_duration_us_ += req_duration_us;
        sample.pc_time = pc_time;
        samples_.push_back(sample);
    }

    if (samples_.size() > 200)
    {
        sum_req_duration_us_ -= samples_.front().req_duration_us;
        samples_.pop_front();
    }

#if 0
    sensor_base = rclcpp::Time((uint64_t)0, samples_.begin()->sensor_time.get_clock_type());
    pc_base = rclcpp::Time((uint64_t)0, samples_.begin()->pc_time.get_clock_type());
#else

    /* Offsets to keep values small in calculation below */
    rclcpp::Time sensor_base = samples_.begin()->sensor_time;
    rclcpp::Time pc_base = samples_.begin()->pc_time;

#if 0
    for(auto it=samples_.begin(); it!=samples_.end(); ++it)
    {
        if (it->sensor_time < sensor_base)
        {
            sensor_base = it->sensor_time;
        }
        if (it->pc_time < pc_base)
        {
            pc_base = it->pc_time;
        }
    }
#endif
#endif

    {
        /* Compute sums for linear regression */
        double sum_x=0, sum_xx=0, sum_y=0, sum_xy=0;
        for(auto it=samples_.begin(); it!=samples_.end(); ++it)
        {
            double x = (it->sensor_time - sensor_base).seconds();
            double y = (it->pc_time - pc_base).seconds();

            sum_x  += x;
            sum_xx += x * x;
            sum_y  += y;
            sum_xy += x * y;
        }

        /* Compute base and coefficent using linear regression */
        double n = (double)samples_.size();
        double den = (n*sum_xx - sum_x*sum_x);
        double time_factor = (den==0.0)?1.0 : ((n*sum_xy - sum_x*sum_y) / den);
        double base_time = (sum_y - time_factor*sum_x)/n;

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

    RCLCPP_INFO(rclcpp::get_logger("timesync"), "update: %f %f %f %f %f %f %u %lu",
        sample.sensor_time.seconds(),
        sample.pc_time.seconds(),
        sensor_base.seconds(),
        pc_base.seconds(),
        base_time_,
        scale_time_,
        req_duration_us,
        mean_req_duration_us/samples_.size());

}

