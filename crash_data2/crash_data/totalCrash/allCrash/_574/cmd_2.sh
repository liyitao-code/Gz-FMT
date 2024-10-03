gz service --timeout 10000 -s /world/actors/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 16
}
plugins {
  name: "gz::sim::systems::LiftDrag"
  filename: "gz-sim-lift-drag-system"
  innerxml: "<!-- force turn -->\n        \n<a0>0.00</a0>\n        \n<cla>2.000</cla>\n        \n<cda>0.001</cda>\n        \n<cma>0.00</cma>\n        \n<alpha_stall>1.0</alpha_stall>\n        \n<cla_stall>-1.0</cla_stall>\n        \n<cda_stall>0.0</cda_stall>\n        \n<cma_stall>0.0</cma_stall>\n        \n<cp>0.0 0 1</cp>\n        \n<area>3</area>\n        \n<air_density>1.2041</air_density>\n        \n<forward>-1 0 0</forward>\n        \n<upward>6742 -10 0</upward>\n        \n<link_name>rudder</link_name>"
}
'