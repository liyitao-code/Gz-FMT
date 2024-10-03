gz service --timeout 10000 -s /world/deformable_sphere/create --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityFactory --req 'sdf: "<sdf version=\"1.11\">\n    <model name=\"buoyant_box\">\n      <pose>0 0 0 0 0 0</pose>\n      <link name=\"body\">\n        <inertial>\n          <mass>1000</mass>\n          <inertia>\n            <ixx>133.3333</ixx>\n            <iyy>493687.6506</iyy>\n            <izz>133.3333</izz>\n          </inertia>\n        </inertial>\n        <visual name=\"body_visual\">\n          <geometry>\n            <box>\n              <size>1 1 1</size>\n            </box>\n          </geometry>\n        </visual>\n        <collision name=\"body_collision\">\n          <geometry>\n            <box>\n              <size>1 1 1</size>\n            </box>\n          </geometry>\n        </collision>\n      </link>\n      <plugin filename=\"gz-sim-buoyancy-engine-system\" name=\"gz::sim::systems::BuoyancyEngine\">\n        <link_name>body</link_name>\n        <namespace>buoyant_box</namespace>\n        <min_volume>0.0</min_volume>\n        <neutral_volume>0.002</neutral_volume>\n        <default_volume>0.002</default_volume>\n        <max_volume>0.003</max_volume>\n        <max_inflation_rate>0.0003</max_inflation_rate>\n      </plugin>\n    </model>\n  </sdf>"
pose {
  position {
    x: -2.410212099721276
    y: -8.869408230295605
    z: 7.478236379063686
  }
}
name: "model"
allow_renaming: true
'