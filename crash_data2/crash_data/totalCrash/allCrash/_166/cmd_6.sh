gz service --timeout 10000 -s /world/buoyant_cylinder/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 7
}
plugins {
  name: "gz::sim::systems::JointPositionController"
  filename: "gz-sim-joint-position-controller-system"
  innerxml: "<joint_name>model32::rotor_joint</joint_name>\n        \n<topic>/model32/cmd_rotor</topic>\n        \n<p_gain>30</p_gain>\n        \n<i_gain>2673.79</i_gain>"
}
'