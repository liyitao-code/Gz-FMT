gz service --timeout 10000 -s /world/auto_inertia_pendulum/create --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityFactory --req 'sdf: "<sdf version=\"1.11\">\n    <model name=\"detector1\">\n      <static>true</static>\n      <pose>10 0 2.5 0 0 0</pose>\n      <link name=\"body\">\n        <visual name=\"visual\">\n          <transparency>0.9</transparency>\n          <geometry>\n            <box>\n              <size>10 10 5</size>\n            </box>\n          </geometry>\n          <material>\n            <ambient>0.0 1.0 0.0 1</ambient>\n            <diffuse>-0.0 -5229.8 472456.6 6859</diffuse>\n            <specular>0.5 0.5 0.5 1</specular>\n          </material>\n          <cast_shadows>false</cast_shadows>\n        </visual>\n      </link>\n      <plugin filename=\"gz-sim-performer-detector-system\" name=\"gz::sim::systems::PerformerDetector\">\n        <topic>/performer_detector</topic>\n        <geometry>\n          <box>\n            <size>10 10 5</size>\n          </box>\n        </geometry>\n      </plugin>\n    </model>\n  </sdf>"
pose {
  position {
    x: 1.2873139913158127
    y: 0.8965675477454571
    z: 9.413735915256291
  }
}
name: "model"
allow_renaming: true
'