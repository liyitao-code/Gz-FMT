gz service --timeout 10000 -s /world/default/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 526
}
plugins {
  name: "gz::sim::systems::DetachableJoint"
  filename: "gz-sim-detachable-joint-system"
  innerxml: "<parent_link>body</parent_link>\n        \n<child_model>box1</child_model>\n        \n<child_link>box_body</child_link>\n        \n<detach_topic>-503357</detach_topic>"
}
'