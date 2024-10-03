gz service --timeout 10000 -s /world/kinetic_energy_monitor/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 8
}
plugins {
  name: "gz::sim::systems::TrackController"
  filename: "gz-sim-track-controller-system"
  innerxml: "<link>rear_left_flipper</link>\n                \n<min_velocity>-1.0</min_velocity>\n                \n<max_velocity>-7151.6</max_velocity>"
}
'