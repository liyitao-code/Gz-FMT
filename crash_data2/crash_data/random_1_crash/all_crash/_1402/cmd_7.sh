gz service --timeout 10000 -s /world/video_record_pendulum/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 8
}
plugins {
  name: "gz::sim::systems::JointController"
  filename: "gz-sim-joint-controller-system"
  innerxml: "<joint_name>j1</joint_name>\n        \n<use_force_commands>iZvN</use_force_commands>\n        \n<p_gain>0.2</p_gain>\n        \n<i_gain>0.01</i_gain>"
}
'