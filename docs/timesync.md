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

Attention: When using TCP, the relationship of physical reception time and
packet evaluation time becomes even more questionable because buffering and
other optimization methods for more reliable communication can introduce more
delays. For example, the Ethernet receiver logic might automagically combine
two incoming frames into a large one, unknown to all processing afterwards,
effectively delaying the evaluation of the first frame until the second frame;
the two frames then reach the application (ROS driver) at the same time.

Therefore, using UDP is recommended over TCP if timing info is a priority.


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

For ease of implementation, those `TimeSync` instances are part of the `params_` `ScanParameter` object,
because this object is within reach for both the PF interface timer callback and during packet evaluation,
although they strictly aren't parameters but dynamic state.

The list of recorded `TimeSync_Sample` is reset whenever a non-zero
`scan_status` in a packet header is seen, indicating a change in sample rate, a
significant deviation from nominal `scan_frequency` or other problems. Also an
update later than one second after the previous one would cause a reset (for
details see `timesync.cpp`). In general, the time span covered by the
collected samples can be configued in driver parameter `timesync_period`.



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

### Usage

Set node parameter `timesync_interval` to some non-zero milliseconds value if
extra HTTP requests for sensor time shall be made. The node will perform those
request regularly each time the interval passed. Otherwise, the packet
evaluation time is used to determine the timestamp relation.

The parameter `timesync_period` determines the period of time over which data
is kept for averaging and `timesync_regression` may be set to `true` if 
linear regression is preferred over simple averaging for computing the ROS
time from sensor time.

Typical setups are described below. The currently recommended setup is to leave
both parameters at 0 if timestamp accuracy and jitter is not a concern,
otherwise `transport udp`, `timesync_interval 0` and `timesync_period 10000`.

Feedback is welcome!


#### No averaging

Just set timestamp in ROS LaserScan to reception time of first scan packet
minus the time needed to measure the points contained in that packet, so it
is somewhat near the moment when the first point was acquired.

Uncertainties: All delays and jitter described in "Scan data packet reception
time" directly affect the timestamp in output. Expect several milliseconds of
jitter in the resulting LaserScan timestamps.

    timesync_interval: 0
    timesync_period: 0

#### Smoothed from packet reception time

The packet reception time vs. sensor time information is collected and used to
compute average offset and slope of PC time compared to sensor time. The result
is used to convert timestamps from sensor to ROS time.

Pro: This gives much more consistent and stable timestamps in the LaserScan
output header with less jitter.

Unvertainty: The average latency from sensor to evaluation on PC is not known
and thus cannot be accounted for. It probably is quite stable for a given setup,
in a range of only a few milliseconds, but in theory, if there was some network
device delaying the transmission or some time consuming processing before each
evaluation, the driver can't notice.

    timesync_interval: 0
    timesync_period: 10000

#### Smoothed from extra HTTP requests for `system_time_raw`

Making extra HTTP requests to the sensor for `system_time_raw` every
`timesync_interval` milliseconds, this is another way to acquire input for
computing sensor time vs. PC time offset and slope.

Pro: The time for making the request and receiving the answer is a known upper
bound for possible error in computed time offset.

Con: This needs extra communication which might be a burden to the sensor at
high sample rates.

Note that `timesync_period` must be larger than `timesync_interval`.

    timesync_interval: 250
    timesync_period: 10000


### Caveats

The recorded data is reset whenever scan parameters such as frequency and
angular resolution are changed. Immediately after such changes, the ROS
timestamps in scan data should be taken with care, while conversion coefficient
and offset are still settling.

The current implementation is not prepared for jumps in ROS time. It probably
should attach a Clock callback to reset the recorded data in such situations.
