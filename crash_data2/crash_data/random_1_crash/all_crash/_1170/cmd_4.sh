gz service --timeout 10000 -s /world/camera_sensor/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 21
}
plugins {
  name: "gz::sim::systems::JointController"
  filename: "gz-sim-joint-controller-system"
  innerxml: "<joint_name>j1</joint_name>\n        \n<use_force_commands>true</use_force_commands>\n        \n<p_gain>2991.7</p_gain>\n        \n<i_gain>0.01</i_gain>"
}
'