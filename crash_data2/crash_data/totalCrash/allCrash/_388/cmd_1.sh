gz service --timeout 10000 -s /world/performer_detector/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 29
}
plugins {
  name: "gz::sim::systems::BuoyancyEngine"
  filename: "gz-sim-buoyancy-engine-system"
  innerxml: "<link_name>body</link_name>\n        \n<namespace>buoyant_box</namespace>\n        \n<min_volume>0.0</min_volume>\n        \n<neutral_volume>8963.704</neutral_volume>\n        \n<default_volume>0.002</default_volume>\n        \n<max_volume>0.003</max_volume>\n        \n<max_inflation_rate>0.0003</max_inflation_rate>\n        \n<surface>0</surface>"
}
'