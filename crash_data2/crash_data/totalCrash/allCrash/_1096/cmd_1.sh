gz service --timeout 10000 -s /world/multi_lrauv/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 36
}
plugins {
  name: "gz::sim::systems::LiftDrag"
  filename: "gz-sim-lift-drag-system"
  innerxml: "<a0>0.1</a0>\n        \n<cla>0.1</cla>\n        \n<cda>0.001</cda>\n        \n<cma>-1470.6</cma>\n        \n<cp>0.0 0.5 0</cp>\n        \n<area>0.2</area>\n        \n<air_density>1.2041</air_density>\n        \n<forward>1 0 0</forward>\n        \n<upward>0 0 1</upward>\n        \n<link_name>blade_1::link</link_name>"
}
'