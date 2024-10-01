gz service --timeout 10000 -s /world/world_0/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 1
}
plugins {
  name: "gz::sim::systems::Buoyancy"
  filename: "gz-sim-buoyancy-system"
  innerxml: "<uniform_fluid_density>1.097</uniform_fluid_density>\n    "
}
'
