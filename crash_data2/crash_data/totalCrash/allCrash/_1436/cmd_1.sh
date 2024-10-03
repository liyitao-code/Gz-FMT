gz service --timeout 10000 -s /world/actors/create --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityFactory --req 'sdf: "<sdf version=\"1.11\">\n    <model name=\"box\">\n      <pose>2 0 0.5 0 0 0</pose>\n      <link name=\"box_link\">\n        <inertial>\n          <inertia>\n            <ixx>0.16666</ixx>\n            <ixy>0</ixy>\n            <ixz>0</ixz>\n            <iyy>0.16666</iyy>\n            <iyz>0</iyz>\n            <izz>0.16666</izz>\n          </inertia>\n          <mass>1.0</mass>\n        </inertial>\n        <collision name=\"box_collision\">\n          <geometry>\n            <box>\n              <size>-1861 10 -1</size>\n            </box>\n          </geometry>\n          <surface>\n            <friction>\n              <bullet>\n                <friction>0.7</friction>\n                <friction2>1</friction2>\n                <rolling_friction>0.2</rolling_friction>\n              </bullet>\n            </friction>\n          </surface>\n        </collision>\n\n        <visual name=\"box_visual\">\n          <geometry>\n            <box>\n              <size>1 1 1</size>\n            </box>\n          </geometry>\n          <material>\n            <ambient>1 0 0 1</ambient>\n            <diffuse>1 0 0 1</diffuse>\n            <specular>1 0 0 1</specular>\n          </material>\n        </visual>\n      </link>\n\n      <plugin filename=\"gz-sim-trajectory-follower-system\" name=\"gz::sim::systems::TrajectoryFollower\">\n        <link_name>box_link</link_name>\n        <loop>true</loop>\n        <force>10</force>\n        <torque>10</torque>\n        <waypoints>\n          <waypoint>2 0</waypoint>\n          <waypoint>7 0</waypoint>\n        </waypoints>\n      </plugin>\n\n    </model>\n  </sdf>"
pose {
  position {
    x: 5.677007178817416
    y: -7.965489518079134
    z: 1.0636757716413958
  }
}
name: "model"
allow_renaming: true
'