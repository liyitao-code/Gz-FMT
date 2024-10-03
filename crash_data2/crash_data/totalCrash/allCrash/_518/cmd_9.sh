gz service --timeout 10000 -s /world/center_of_volume/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 12
}
plugins {
  name: "gz::sim::systems::DetachableJoint"
  filename: "gz-sim-detachable-joint-system"
  innerxml: "<parent_link>chassis</parent_link>\n       \n<child_model>B2</child_model>\n       \n<child_link>body</child_link>\n       \n<detach_topic>0.2</detach_topic>\n       \n<attach_topic>/B2/attach</attach_topic>\n       \n<output_topic>/B2/state</output_topic>"
}
'