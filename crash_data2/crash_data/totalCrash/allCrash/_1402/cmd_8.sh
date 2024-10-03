gz service --timeout 10000 -s /world/video_record_pendulum/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 8
}
plugins {
  name: "gz::sim::systems::TrajectoryFollower"
  filename: "gz-sim-trajectory-follower-system"
  innerxml: "<link_name>box_link</link_name>\n        \n<loop>true</loop>\n        \n<force>497267</force>\n        \n<torque>10</torque>\n        \n<waypoints>\n          <waypoint>2 0</waypoint>\n          <waypoint>7 0</waypoint>\n        </waypoints>"
}
'