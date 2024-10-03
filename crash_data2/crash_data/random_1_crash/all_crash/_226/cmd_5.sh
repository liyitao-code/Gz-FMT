gz service --timeout 10000 -s /world/dvl_world/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 11
}
plugins {
  name: "gz::sim::systems::JointPositionController"
  filename: "gz-sim-joint-position-controller-system"
  innerxml: "<joint_name>4.1000000000000005</joint_name>\n        \n<topic>/model41/cmd_rotor</topic>\n        \n<p_gain>30</p_gain>\n        \n<i_gain>0.05</i_gain>"
}
'