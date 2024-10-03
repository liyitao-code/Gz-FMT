gz service --timeout 10000 -s /world/multi_lrauv/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 23
}
plugins {
  name: "gz::sim::systems::VelocityControl"
  filename: "gz-sim-velocity-control-system"
  innerxml: "<initial_linear>-6530.4 -506596 -5039</initial_linear>\n        \n<initial_angular>0 0 -0.1</initial_angular>"
}
'