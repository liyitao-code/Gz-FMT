gz service --timeout 10000 -s /world/nested_model_joint_positions/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 79
}
plugins {
  name: "gz::sim::systems::TrackController"
  filename: "gz-sim-track-controller-system"
  innerxml: "<link>base_link</link>\n                \n<odometry_publish_frequency>1</odometry_publish_frequency>\n                \n<!--u62Qg6juw2yfxcoir-->"
}
'