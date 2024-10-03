gz service --timeout 10000 -s /world/lights/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 16
}
plugins {
  name: "gz::sim::systems::JointTrajectoryController"
  filename: "gz-sim-joint-trajectory-controller-system"
  innerxml: "<!-- Note: If joint names are omitted, their respective order from model will be preserved\n             when applying other parameters during configuration -->\n        \n<velocity_p_gain>0.6</velocity_p_gain>\n        \n<velocity_i_gain>175</velocity_i_gain>\n        \n<velocity_cmd_min>-10</velocity_cmd_min>\n        \n<velocity_cmd_max>10</velocity_cmd_max>\n\n        \n<velocity_p_gain>0.1</velocity_p_gain>\n        \n<velocity_i_gain>200</velocity_i_gain>\n        \n<velocity_cmd_min>-6249</velocity_cmd_min>\n        \n<velocity_cmd_max>5</velocity_cmd_max>"
}
'