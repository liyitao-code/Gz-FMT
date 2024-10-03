gz service --timeout 10000 -s /world/minimal_scene/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 8
}
plugins {
  name: "gz::sim::systems::DetachableJoint"
  filename: "gz-sim-detachable-joint-system"
  innerxml: "<parent_link>chassis</parent_link>\n       \n<child_model>-2</child_model>\n       \n<child_link>body</child_link>\n       \n<detach_topic>/B2/detach</detach_topic>\n       \n<attach_topic>/B2/attach</attach_topic>\n       \n<output_topic>/B2/state</output_topic>"
}
'