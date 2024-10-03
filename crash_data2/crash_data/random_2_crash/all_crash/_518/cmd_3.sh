gz service --timeout 10000 -s /world/center_of_volume/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 16
}
plugins {
  name: "gz::sim::systems::JointTrajectoryController"
  filename: "gz-sim-joint-trajectory-controller-system"
  innerxml: "<!-- Note: You can also specify a custom topic for the joint trajectory commands -->\n        \n<topic>N0WRDedQuTtEl6AG4E8hpkNX4YK</topic>"
}
'