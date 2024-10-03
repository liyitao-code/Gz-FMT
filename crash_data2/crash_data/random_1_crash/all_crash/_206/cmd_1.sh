gz service --timeout 10000 -s /world/default/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 8
}
plugins {
  name: "gz::sim::systems::VelocityControl"
  filename: "gz-sim-velocity-control-system"
  innerxml: "<initial_linear>4797.2 -9147 -495592</initial_linear>\n        \n<initial_angular>0 0 -0.1</initial_angular>"
}
'