# Dynamic global obstacles

The global costmap loads `obstacle_layer`, but the configured default is
disabled. Enable or disable it in a running Nav2 process with:

```bash
ros2 param set /global_costmap/global_costmap obstacle_layer.enabled true
ros2 param set /global_costmap/global_costmap obstacle_layer.enabled false
```

The installed Jazzy `nav2_costmap_2d` 1.3.12 package accepts `enabled` through
the obstacle layer's dynamic-parameter callback. This was verified against a
configured instance of the installed binary by changing the parameter
`false -> true -> false`; no relaunch is required.

The layer uses `combination_method: 0` (Overwrite). Live observations therefore
replace static-map costs only inside the obstacle layer's current update
bounds. This allows a later raytrace to clear a transient mark; the static
layer continues to supply the map outside those bounds.

## Known low-obstacle hazard

The laser scan plane is 0.1135 m above the floor. A shelf edge, threshold, or
other object entirely below that plane can be raytraced through. With
Overwrite, such a feature would be cleared from the combined global costmap
inside the observation bounds even if it were occupied in the static map.

No usable map bundle is currently committed. Assess this hazard against every
new map, especially if its occupancy image is manually edited or generated
from a sensor at a different height.

## A/B measurement

After enabling the layer, sample the raw layer every 10 seconds during a
two-minute driving session:

```bash
ros2 topic echo /global_costmap/obstacle_layer_raw \
  nav2_msgs/msg/Costmap --field data --once
```

This is a `nav2_msgs/msg/Costmap`, so count cells whose cost value is `254`
(`LETHAL_OBSTACLE`), not occupancy-grid value 100. The acceptance target is a
bounded or stabilising series, rather than the prior monotonic
`308 -> 1078 -> 1539` growth over 112 seconds.
