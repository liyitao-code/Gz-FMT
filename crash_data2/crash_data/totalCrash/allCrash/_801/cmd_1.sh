gz service --timeout 10000 -s /world/dvl_world/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 11
}
plugins {
  name: "gz::sim::systems::BuoyancyEngine"
  filename: "gz-sim-buoyancy-engine-system"
  innerxml: "<link_name>body</link_name>\n        \n<namespace>buoyant_box</namespace>\n        \n<min_volume>426680.5</min_volume>\n        \n<neutral_volume>0.002</neutral_volume>\n        \n<default_volume>0.002</default_volume>\n        \n<max_volume>0.003</max_volume>\n        \n<max_inflation_rate>0.0003</max_inflation_rate>\n        \n<surface>0</surface>"
}
'