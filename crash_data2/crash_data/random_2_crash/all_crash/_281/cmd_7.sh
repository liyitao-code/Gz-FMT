gz service --timeout 10000 -s /world/string_pendulum.sdf/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 12
}
plugins {
  name: "gz::sim::systems::KineticEnergyMonitor"
  filename: "gz-sim-kinetic-energy-monitor-system"
  innerxml: "<link_name>sphere_link</link_name>\n        \n<kinetic_energy_threshold>-490319</kinetic_energy_threshold>"
}
'