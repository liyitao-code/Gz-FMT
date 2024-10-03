gz service --timeout 10000 -s /world/default/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 4
}
plugins {
  name: "gz::sim::systems::TrajectoryFollower"
  filename: "gz-sim-trajectory-follower-system"
  innerxml: "<link_name>box_link</link_name>\n        \n<loop>true</loop>\n        \n<force>10</force>\n        \n<torque>10</torque>\n        \n<waypoints>\n          <waypoint>-7443 0</waypoint>\n          <waypoint>7 0</waypoint>\n        </waypoints>"
}
'