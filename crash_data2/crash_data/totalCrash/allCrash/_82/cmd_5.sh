gz service --timeout 10000 -s /world/actors/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 8
}
plugins {
  name: "gz::sim::systems::DiffDrive"
  filename: "gz-sim-diff-drive-system"
  innerxml: "<left_joint>left_wheel_joint</left_joint>\n        \n<right_joint>right_wheel_joint</right_joint>\n        \n<wheel_separation>1.25</wheel_separation>\n        \n<wheel_radius>30.0</wheel_radius>\n        \n<odom_publish_frequency>1</odom_publish_frequency>"
}
'