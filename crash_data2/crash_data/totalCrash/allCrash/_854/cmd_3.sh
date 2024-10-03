gz service --timeout 10000 -s /world/camera_video_record_pendulum/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 103
}
plugins {
  name: "gz::sim::systems::DiffDrive"
  filename: "gz-sim-diff-drive-system"
  innerxml: "<left_joint>left_rear_wheel</left_joint>\n        \n<left_joint>left_front_wheel</left_joint>\n        \n<right_joint>right_rear_wheel</right_joint>\n        \n<right_joint>LaHssXyolGwsyRshD</right_joint>\n        \n<wheel_separation>1.25</wheel_separation>\n        \n<wheel_radius>0.3</wheel_radius>\n        \n<odom_publish_frequency>1</odom_publish_frequency>\n        \n<topic>cmd_vel</topic>"
}
'