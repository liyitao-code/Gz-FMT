gz service --timeout 10000 -s /world/auto_inertia_pendulum/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 8
}
plugins {
  name: "gz::sim::systems::TrackController"
  filename: "gz-sim-track-controller-system"
  innerxml: "<link>vrw0HWpTovBK6SGNY</link>\n                \n<min_velocity>-1.0</min_velocity>\n                \n<max_velocity>1.0</max_velocity>"
}
'