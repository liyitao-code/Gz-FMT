gz sim a.sdf -r &
sleep 2


gz service --timeout 10000 -s /world/world_0/create --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityFactory --req 'sdf: "<sdf version=\"1.11\"><model name=\"detector1\">\n      <static>true</static>\n      <pose>10 0 2.5 0 0 0</pose>\n      <link name=\"body\">\n        <visual name=\"visual\">\n          <transparency>0.9</transparency>\n          <geometry>\n            <box>\n              <size>10 10 5</size>\n            </box>\n          </geometry>\n          <material>\n            <ambient>0.0 1.0 0.0 1</ambient>\n            <diffuse>0.0 1.0 0.0 1</diffuse>\n            <specular>0.5 0.5 0.5 1</specular>\n          </material>\n          <cast_shadows>false</cast_shadows>\n        </visual>\n      </link>\n      <plugin filename=\"gz-sim-performer-detector-system\" name=\"gz::sim::systems::PerformerDetector\">\n        <topic>/performer_detector</topic>\n        <geometry>\n          <box>\n            <size>10 10 5</size>\n          </box>\n        </geometry>\n      </plugin>\n    </model></sdf>"
pose {
  position {
    x: -7.547238581888573
    y: 5.759460310071722
    z: 1.1831807505648284
  }
}
name: "model"
allow_renaming: true'


# gz service --timeout 10000 -s /world/world_0/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
#   id: 47
# }
# plugins {
#   name: "gz::sim::systems::LiftDrag"
#   filename: "gz-sim-lift-drag-system"
#   innerxml: "<a0>0.1</a0>\n        \n<cla>0.1</cla>\n        \n<cda>0.001</cda>\n        \n<cma>0.0</cma>\n        \n<cp>0.0 0.5 0</cp>\n        \n<area>0.2</area>\n        \n<air_density>1.2041</air_density>\n        \n<forward>1 0 0</forward>\n        \n<upward>0 0 1</upward>\n        \n<link_name>blade_2::link</link_name>\n      "
# }'
