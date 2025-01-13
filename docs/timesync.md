## Time synchronization

The timestamps in scan data are read from a independent clock in the sensor.

In `feature/timesync` branch, two methods are implemented that can relate the
sensor clock to ROS time momentarily. Further methods (e.g. with extra hardware
in place or support of a separate time server) may be added later.

Used regularly, their results can be fed into a common algorithm to compute and
update sensor time versus ROS time offset and linearity error.  With this
information, all sensor timestamps can be converted into ROS timestamps.

### Scan data packet reception time

The sensor timestamp of the packets received from the sensor can be related to the
ROS time when the packets are evaluated.

At that time of evaluation, an unknown duration has passed since physical reception.

Several delays have to be taken into account:

 - the timestamp in the packet describes the time when the first included measurement
    was made. Since then, `num_points_packet` further measurements took place at the
    current sample rate, until the packet was constructed and passed to the
    network driver at the sensor.
 - the actual transmission (preparation in network stack on sensor, eventual route
    passing more network devices like routers, and reception and routing to application
    on the computer running the ROS driver node) takes some time. The amount of time
    depends on the transmission protocol (realtime UDP for first UDP handle takes almost
    no time, TCP however could even be slowed down by the ROS node) and is unknown.

Unfortunately, the exact time when the packet was actually received is only
available for UDP data only with extra low level OS-dependent functions and out
of reach in this implementation.

Typically however it is only very few milliseconds with noticable some exceptions.


### Continuous requests for sensor time

A HTTP request is made by ROS driver to request the `timestamp_raw` from sensor. The time when
the request is made and the time when the response is ready give absolute bounds for the ROS time
that was current at the time when the reported `timestamp_raw` was read on the sensor side.

However, the duration of the request can be rather long (ie. tens of milliseconds) and it is a
little uncertain when the clock was read within the time frame - more towards the beginning or end?

### Common time

The sensor and ROS could synchronize their time to a common time source. The R2000 and R2300 yet
do not support such synchronization.

### Implementation in PF driver

Class `TimeSync` (in `src/pf_driver/src/pf/timesync.cpp` and `src/pf_driver/include/pf/timesync.h`)
holds an array of `TimeSync_Sample`, each relating a particular `pc_time` to a `sensor_time` timestamp.
New timestamp pairs can be fed into an instance of `TimeSync` by calling its `update()` method, and
`sensor_to_rclcpp()` is meant for conversion of sensor to ROS timestamps on the basis of the accumulated
data.

In `pf_interface`, a timer is set up to regularly trigger HTTP requests for sensor time and update the
`TimeSync` instance `active_timesync` with the results. Its frequency is determined by the
`timesync_interval` driver parameter (milliseconds). If the request took longer than 50 ms, the
result is ignored.

Another instance, `passive_timesync`, is updated each time when a scan data packet is evaluated, using
the sensor timestamp in the packet and the ROS time when evaluation starts.  At
that time of evaluation, an unknown duration has passed since physical reception. The update frequency
currently is determined implicitly by the rate of scan data packets and a hardcoded
upper limit of 10 Hz (packets within 100 ms after an update are ignored).

For ease of implementation, the `TimeSync` instances are part of the `params_` `ScanParameter` object,
because this object is within reach for both the PF interface timer callback and during packet evaluation,
although they strictly aren't parameters but dynamic state.

The list of recorded `TimeSync_Sample` is reset whenever a non-zero
`scan_status` in a packet header is seen, indicating a change in sample rate, a
significant deviation from nominal `scan_frequency` or other problems. Also an
update later than one second after the previous one would cause a reset (for
details see `timesync.cpp`).

### Changes to previous implementation

In short, previously, the driver put the time of evaluation of the final
packet of a scan into LaserScan.header.stamp.

Now it represents the time when the first sample was taken. It is converted
from sensor time into ROS time using offset and coefficient computed from
data obtained during preceding observation of the time relationship.

More precisely, in previous driver versions, the LaserScan `header.stamp` was
set to the `rclcpp::Clock().now() - scan_time` at the time when the last
contributing packet was parsed. Before `bugfix/laserscan-metadata`, the
`scan_time` was mistakenly always zero so effectively this represented the time
when the final packet of the scan was received. After
`bugfix/laserscan-metadata`, it becomes more complicated, because `scan_time`
is calculated correctly but doesn't necessarily represent the time between
first sample and reception of the final packet (except with full 360° scans
from R2000).

### Usage

Set node parameter `timesync_interval` to some non-zero milliseconds value if
extra HTTP requests for sensor time shall be made. The node will perform those
request regularly each time the interval passed. Otherwise, the packet
evaluation time is used to determine the timestamp relation.

The parameter `timesync_period` determines the period of time over which data
is kept for averaging and `timesync_regression` may be set to `true` if 
linear regression is preferred over simple averaging for computing the PC
time from sensor time.

The recorded data is reset whenever scan parameters such as frequency and
angular resolution are changed. Immediately after such changes, the ROS
timestamps in scan data should be taken with care, while conversion coefficient
and offset are still settling.


