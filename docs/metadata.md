## Scan metadata

The calculation of metadata in LaserScan and PointCloud2 messages such as timestamps and
resolution information has been reworked in `bugfix/laserscan-metadata` branch.

The following `sensor_msgs/msg/LaserScan` fields are affected:

                                 # in frame frame_id, angles are measured around
                                 # the positive Z axis (counterclockwise, if Z is up)
                                 # with zero angle being forward along the x axis

    float32 angle_min            # start angle of the scan [rad]
    float32 angle_max            # end angle of the scan [rad]
    float32 angle_increment      # angular distance between measurements [rad]

    float32 time_increment       # time between measurements [seconds] - if your scanner
                                 # is moving, this will be used in interpolating position
                                 # of 3d points
    float32 scan_time            # time between scans [seconds]

    float32 range_min            # minimum range value [m]
    float32 range_max            # maximum range value [m]

### Changes

#### `scan_time`

`scan_time` previously always was set to zero due to a data type mismatch in
code.  It is now derived from the `scan_frequency` as reported by the sensor in
the packet header. It tells the *configured* scan frequency (PFSDP parameter
`scan_frequency`), not the actual physical speed (`scan_frequency_measured`).

For a 4-layer R2300 the driver distributes `LaserScan` messages on four
separate topics, one per each layer, so that subscribers can differentiate
between layers (there is no elevation information in `LaserScan`). In this
configuration, each `scan_time` is calculated per layer, thus appears to be 
four times longer than for 1-layer R2300.

#### `angle_min`, `angle_max`, `angle_increment`

`angle_min` specifies the angle at which the very first measurement of the
`LaserScan` was taken, as reported in the header of the first packet of a scan.

Previously it told the *configured* start angle. Consequently, the `angle_max`,
which was previously computed from configured and dynamic information, is now
set from data received in the packet headers:

    angle_max := angle_min + (num_points_scan+1) * angle_increment

Note that a scanner which is configured to rotate *clockwise*, which differs
from the default (`ccw`) behaviour, outputs negative `angle_increment` which
might come unexpected:

    angle_increment < 0

    angle_max < angle_min

Decimation caused by optional scan data filtering is taken into account. A
`filter_width=4` would cause a proportionally larger `angle_increment`. 

#### `time_increment` 

`time_increment` previously was derived from `scan_time` and thus
always zero (see above) but now will give reasonable information.

For R2000 it is still computed from the `scan_time`, which is computed the
from *configured* `scan_frequency` value, simply by dividing:

    R2000 time_increment := scan_time / (360° / angular_increment)

If rotational speed is slightly off, especially when target speed was just
reconfigured or or due to external physical impacts, also the `time_increment`
also might be a little off the truth.

The R2300 sample rate is not as tightly coupled to the current speed as in
R2000. For both `scan_frequency` options, the nominal sample rate is 90 kHz.

    R2300 time_increment := 1/90 kHz

Again the decimation caused by optional scan data filtering is taken into account.
If four actual measurements are averaged into one point, the `time_increment` is
accordingly four times longer than without filterung.

Note that `time_increment` in many situations is *not* 

    time_increment NOT= scan_time / (angle_max - angle_min)


