gz service --timeout 10000 -s /world/default/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 4
}
plugins {
  name: "gz::sim::systems::JointTrajectoryController"
  filename: "gz-sim-joint-trajectory-controller-system"
  innerxml: "<!-- Note: You can also specify a custom topic for the joint trajectory commands -->\n        \n<topic>Sh6QrNVRSBIuVUI4uyA5XoXy617</topic>"
}
'