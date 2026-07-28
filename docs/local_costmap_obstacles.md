# Local costmap obstacle observations

The local costmap consumes `/scan` directly as `sensor_msgs/LaserScan`.
Marking and clearing are enabled. Finite returns from 0.05 m through 1.0 m
mark obstacles; clearing raytraces span 0.0 m through 1.2 m. Infinite returns
are valid clearing observations. Observations are not persisted, and no
expected-update-rate timeout is enforced.

Both the layer and the `scan` observation source accept obstacle heights from
0.0 m through 2.0 m. The source-level maximum must remain explicit. Its
Nav2 default is 0.0 m, which rejects every scan point after projection from
the laser's 0.1135 m mounting height even when the layer-level maximum is
2.0 m.

The scan plane is 0.1135 m above the floor. Objects below approximately
0.11 m cannot intersect it and are structurally undetectable by this sensor
layout. A shoe, low box, cable, or similar low object must not be treated as
part of the supported autonomous operating envelope. This limitation cannot
be corrected in costmap software.
