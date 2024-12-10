## Time synchronization

The timestamps in scan data are read from a independent clock in the sensor.

In `feature/timesync` branch, two methods are implemented that can relate the
sensor clock to ROS time momentarily. Further methods (e.g. with extra hardware
in place or support of a separate time server) may be added later.

Used regularly, their results can be fed into a common algorithm to compute and
update sensor time versus ROS time offset and linearity error.  With this
information, all sensor timestamps can be converted into ROS timestamps.

### Scan data packet reception time


### Continuous requests for sensor time


### Implementation in PF driver

Class `TimeSync` (in `src/pf_driver/src/pf/timesync.cpp` and `src/pf_driver/include/pf/timesync.h`)
holds an array of `TimeSync_Sample`, each relating a particular `pc_time` to a `sensor_time` timestamp.
New timestamp pairs can be fed into an instance of `TimeSync` by calling its `update()` method, and
`sensor_to_rclcpp()` is meant for conversion of sensor to ROS timestamps on the basis of the accumulated
data.

In `pf_interface`, a timer is set up to regularly trigger HTTP requests for sensor time and update the
`TimeSync` instance `active_timesync` with the results.

Another instance, `passive_timesync`, is updated each time when a scan data packet is evaluated, using
the sensor timestamp in the packet and the ROS time when evaluation starts.  At that time of evaluation, an unknown duration has passed since physical reception.

Unfortunately, the exact time when the packet was actually received is only
available with extra low level OS-dependent functions and out of reach in this
implementation.

For ease of implementation, the `TimeSync` instances are part of the `params_` `ScanParameter` object,
because this object is within reach for both the PF interface timer callback and during packet evaluation,
although they strictly aren't parameters but dynamic state.


### Usage

Set `timesync_interval` to some non-zero milliseconds value if extra HTTP requests for sensor time shall
be made. Otherwise, the packet reception is used to determine the timestamp relation.

The recorded data is reset whenever scan parameters such as frequency and angular resolution are changed.
The timestamps presented in the output just after changes should not be taken with care, as they might be
inaccurate and jittering.



