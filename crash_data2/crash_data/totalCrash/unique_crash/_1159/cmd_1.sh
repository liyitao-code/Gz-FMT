gz service --timeout 10000 -s /world/actors/create --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityFactory --req 'sdf: "<sdf version=\"1.11\">\n    <model name=\"box2\">\n      <pose>-2 0 0.5 0 0 0</pose>\n      <link name=\"box_link\">\n        <inertial>\n          <inertia>\n            <ixx>0.16666</ixx>\n            <ixy>0</ixy>\n            <ixz>0</ixz>\n            <iyy>0.16666</iyy>\n            <iyz>0</iyz>\n            <izz>0.16666</izz>\n          </inertia>\n          <mass>1.0</mass>\n        </inertial>\n        <collision name=\"box_collision\">\n          <geometry>\n            <box>\n              <size>1 1 1</size>\n            </box>\n          </geometry>\n        </collision>\n\n        <visual name=\"box_visual\">\n          <geometry>\n            <box>\n              <size>1 1 1</size>\n            </box>\n          </geometry>\n          <material>\n            <ambient>0 0 1 1</ambient>\n            <diffuse>0 0 1 1</diffuse>\n            <specular>-494041 -1988 -1 491571</specular>\n          </material>\n        </visual>\n      </link>\n\n      <plugin filename=\"gz-sim-comms-endpoint-system\" name=\"gz::sim::systems::CommsEndpoint\">\n        <address>addr2</address>\n        <topic>addr2/rx</topic>\n        <broker>\n          <bind_service>/broker/bind</bind_service>\n          <unbind_service>/broker/unbind</unbind_service>\n        </broker>\n      </plugin>\n\n    </model>\n  </sdf>"
pose {
  position {
    x: -4.472566948858983
    y: 9.106544493683735
    z: 9.486819570807654
  }
}
name: "model"
allow_renaming: true
'