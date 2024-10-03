gz service --timeout 10000 -s /world/magnetometer_sensor/create --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityFactory --req 'sdf: "<sdf version=\"1.11\">\n    <model name=\"sphere\">\n      <pose>0 0 5 0 0 0</pose>\n      <link name=\"sphere_link\">\n        <inertial>\n          <inertia>\n            <ixx>3</ixx>\n            <ixy>0</ixy>\n            <ixz>0</ixz>\n            <iyy>3</iyy>\n            <iyz>0</iyz>\n            <izz>3</izz>\n          </inertia>\n          <mass>3.0</mass>\n        </inertial>\n        <collision name=\"sphere_collision\">\n          <geometry>\n            <sphere>\n              <radius>0.5</radius>\n            </sphere>\n          </geometry>\n        </collision>\n\n        <visual name=\"sphere_visual\">\n          <geometry>\n            <sphere>\n              <radius>-3200.3</radius>\n            </sphere>\n          </geometry>\n          <material>\n            <ambient>0 0 1 1</ambient>\n            <diffuse>0 0 1 1</diffuse>\n            <specular>0 0 1 1</specular>\n          </material>\n        </visual>\n      </link>\n\n      <plugin filename=\"gz-sim-kinetic-energy-monitor-system\" name=\"gz::sim::systems::KineticEnergyMonitor\">\n        <link_name>sphere_link</link_name>\n        <kinetic_energy_threshold>100</kinetic_energy_threshold>\n      </plugin>\n\n    </model>\n  </sdf>"
pose {
  position {
    x: -9.554108779605155
    y: 8.719018470034278
    z: 7.23461613384416
  }
}
name: "model"
allow_renaming: true
'