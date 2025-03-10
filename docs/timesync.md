## Time synchronization

The timestamps in scan data are read from a clock in the sensor, independent of
ROS time. For output of scan data as ROS topics, the timestamps need to be
converted into ROS timestamps.

The information required for this conversion can be gathered from different
sources, described in the following sections.  Used regularly, their results
can be fed into a common algorithm to compute and update sensor time versus ROS
time offset and linearity error.


### Scan data packet evaluation time

The sensor timestamp of the packets received from the sensor can be related to the
ROS time when the packets are evaluated.

At that time of evaluation, an unknown duration has passed since physical reception.

Several delays have to be taken into account:

 - the timestamp in the packet describes the time when first scan point in this packet was made
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


### PPS (pulses per second)

R2000 can be configured to generate a pulse on a switching output related to
its internal time, e.g. whenever its internal clock reaches a full second.
Knowing exactly at which ROS time the full sensor time second is reached can
help to improve the accuracy of the offset that has been determined with other
methods.  The ROS driver might get support for this in the future, e.g. with
sensor output connected to a GPIO or serial (RS232) control line input, but
currently this is not possible.


## Usage

Select a method to compute a ROS timestamp from sensor timestamp by setting
`timesync_method` to one of the following values (defaults to 'average' if not set):

 - `off` (sensor time is converted without adding any offset, not conformant to `msgs/LaserScan` spec)
 - `simple` (ROS timestamp is set to packet evalution ROS time minus packet duration)
 - `average` (ROS timestamp is set to sensor timestamp plus offset to ROS time, computed from packet evaluation times)
 - `requests` (ROS timestamp is set to sensor timestamp plus offset to ROS time, determined with HTTP requests to sensor)

The node parameter `timesync_interval` determines the interval (in milliseconds)
between requests made for sensor time if `timesync_method` is `requests`.

The parameter `timesync_period` (in milliseconds) determines the period of time
over which data is recorded for averaging.

Additionally, `timesync_offset_usec` allows to specify a fixed duration (in microseconds)
which will be added to the PC time computed from sensor time, to compensate for
a known extra offset between computed and actual time.

When using the packet evaluation timestamps (`timesync_method: average`) for
estimation, you typically have to compensate for a time that is a little behind
(larger) with a negative value, e.g. -3000.  When using extra HTTP requests,
the computed time instead appears to be ahead of the actual time so the value
typically must be positive, e.g. 10000, often dependent on the load caused by
current amount of data to handle (scan frequency X scan size).

Typical setups are described below. The currently recommended setup is to leave
`timesync_method` at its default (`average`), choose `transport: udp` and
`timesync_period: 10000`.

Feedback is welcome!
 

#### Method 'off': No conversion

The timestamp from sensor is just numerically converted into ROS time. That is,
for example, if the sensor was just powered on, you'd expect just a few seconds
here. This does not conform to the `msgs/LaserScan` specification and therefore
should be chosen only if you only do your own custom processing of the messages.

    timesync_method:   "off"


#### Method 'simple': Just packet evaluation time, no averaging

Just set timestamp in ROS LaserScan to evaluation time of first scan packet
minus the time needed to measure the points contained in that packet, so it
is somewhat near the moment when the first point was acquired.

Uncertainties: All delays and jitter described in "Scan data packet evalution
time" directly affect the timestamp in output. Expect several milliseconds of
jitter in the resulting LaserScan timestamps.

    timesync_method:   "simple"
    timesync_offset_usec: 0


#### Method 'average': Smoothed from packet evaluation time

The packet evaluation time vs. sensor time information is collected and used to
compute average offset and slope of PC time compared to sensor time. The result
is used to convert timestamps from sensor to ROS time.

Pro: This gives much more consistent and stable timestamps in the LaserScan
output header with less jitter. 3ms of stable latency can be compensated for by
decrementing the PC time computed from sensor time by this amount (adding a
negative `timesync_offset_usec`).

Uncertainty: The average latency from sensor to evaluation on PC is not known
and thus cannot be accounted for automatically. It probably is quite stable for
a given setup, in a range of only a few milliseconds, but in theory, if there
was some network device delaying the transmission or some time consuming
processing before each evaluation, the driver can't notice.

    timesync_method:   "average"
    timesync_period:   10000
    timesync_offset_usec: -3000


#### Method 'requests': Smoothed from extra HTTP requests for `system_time_raw`

Making extra HTTP requests to the sensor for `system_time_raw` every
`timesync_interval` milliseconds, this is another way to acquire input for
computing sensor time vs. PC time offset and slope.

Pro: The duration from sending the request until the answer is received 
from the sensor is a known upper bound for possible error in computed time
offset.

Con: This needs extra communication which might be a burden to the sensor 
especially at high sample rates, and increase risk of gaps in scan data.

Note that `timesync_period` must be larger than `timesync_interval`.

    timesync_method:   "requests"
    timesync_interval: 250
    timesync_period:   10000
    timesync_offset_usec: 7000


### Caveats

The recorded data is reset whenever scan parameters such as frequency and
angular resolution are changed. Immediately after such changes, the ROS
timestamps in scan data should be taken with care, while conversion coefficient
and offset are still settling.

Jumps in ROS time larger than 15s, e.g. due to changes in daylight savings,
also will cause the driver to reset the recorded data, as will jumps in sensor
time that exceed 1s.


### Changes to previous driver versions

Previous driver versions supported only the 'simple' method and no finetuning.
Latest versions were out by as much as one scan duration when estimating the
timestamp of the acquisition of the first point.


