gz service --timeout 10000 -s /world/default/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 4
}
plugins {
  name: "gz::sim::systems::DetachableJoint"
  filename: "gz-sim-detachable-joint-system"
  innerxml: "<parent_link>chassis</parent_link>\n       \n<child_model>B3</child_model>\n       \n<child_link>7H6x</child_link>\n       \n<detach_topic>/B3/detach</detach_topic>\n       \n<attach_topic>/B3/attach</attach_topic>\n       \n<output_topic>/B3/state</output_topic>"
}
'