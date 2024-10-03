gz service --timeout 10000 -s /world/acoustic_comms/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 12
}
plugins {
  name: "gz::sim::systems::TrajectoryFollower"
  filename: "gz-sim-trajectory-follower-system"
  innerxml: "<link_name>box_link</link_name>\n        \n<loop>true</loop>\n        \n<force>10</force>\n        \n<torque>10</torque>\n        \n<line>\n          <direction>0</direction>\n          <length>0.5</length>\n        </line>"
}
'