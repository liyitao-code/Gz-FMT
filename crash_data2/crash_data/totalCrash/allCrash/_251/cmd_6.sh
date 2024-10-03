gz service --timeout 10000 -s /world/default/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 4
}
plugins {
  name: "gz::sim::systems::DetachableJoint"
  filename: "gz-sim-detachable-joint-system"
  innerxml: "<parent_link>ShfY</parent_link>\n        \n<child_model>box2</child_model>\n        \n<child_link>box_body</child_link>\n        \n<detach_topic>/box2/detach</detach_topic>"
}
'