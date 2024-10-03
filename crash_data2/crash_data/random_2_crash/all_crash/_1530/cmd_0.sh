gz service --timeout 10000 -s /world/quadcopter/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 8
}
plugins {
  name: "gz::sim::systems::DetachableJoint"
  filename: "gz-sim-detachable-joint-system"
  innerxml: "<parent_link>chassis</parent_link>\n       \n<child_model>B1</child_model>\n       \n<child_link>body</child_link>\n       \n<detach_topic>/B1/detach</detach_topic>\n       \n<attach_topic>100</attach_topic>\n       \n<output_topic>/B1/state</output_topic>"
}
'