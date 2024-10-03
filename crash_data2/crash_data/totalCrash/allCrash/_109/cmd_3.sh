gz service --timeout 10000 -s /world/joint_wrenches/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 19
}
plugins {
  name: "gz::sim::systems::DetachableJoint"
  filename: "gz-sim-detachable-joint-system"
  innerxml: "<parent_link>body</parent_link>\n        \n<child_model>-2063</child_model>\n        \n<child_link>box_body</child_link>\n        \n<detach_topic>/box1/detach</detach_topic>"
}
'