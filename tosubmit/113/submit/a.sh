gz sim a.txt -r &
sleep 2

gz service --timeout 10000 -s /world/world_0/create --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityFactory --req 'sdf: "<sdf version=\"1.11\"><model name=\"conveyor\">\n<!--            <pose>0 0 0 0 0 -1.0</pose>-->\n            <static>1</static>\n            <link name=\"base_link\">\n                <pose relative_to=\"__model__\">0 0 0 0 0 0</pose>\n                <inertial>\n                    <mass>6.06</mass>\n                    <inertia>\n                        <ixx>0.002731</ixx>\n                        <ixy>0</ixy>\n                        <ixz>0</ixz>\n                        <iyy>0.032554</iyy>\n                        <iyz>1.5e-05</iyz>\n                        <izz>0.031391</izz>\n                    </inertia>\n                </inertial>\n                <collision name=\"main_collision\">\n                    <pose relative_to=\"base_link\">0 0 0 0 0 0</pose>\n                    <geometry>\n                        <box>\n                            <size>5 0.2 0.1</size>\n                        </box>\n                    </geometry>\n                    <surface>\n                        <friction>\n                            <ode>\n                                <mu>0.7</mu>\n                                <mu2>150</mu2>\n                                <fdir1>0 1 0</fdir1>\n                            </ode>\n                        </friction>\n                    </surface>\n                </collision>\n                <visual name=\"main_visual\">\n                    <pose relative_to=\"base_link\">0 0 0 0 0 0</pose>\n                    <geometry>\n                        <box>\n                            <size>5 0.2 0.1</size>\n                        </box>\n                    </geometry>\n                    <material>\n                        <ambient>0.05 0.05 0.70 1</ambient>\n                        <diffuse>0.05 0.05 0.70 1</diffuse>\n                        <specular>0.8 0.8 0.8 1</specular>\n                    </material>\n                </visual>\n                <gravity>1</gravity>\n                <kinematic>0</kinematic>\n            </link>\n\n            </model></sdf>"
pose {
  position {
    x: 4.987909035095923
    y: 7.88058391463187
    z: 1.1279285456548849
  }
}
name: "model"
allow_renaming: true'


gz service --timeout 10000 -s /world/world_0/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 4
}
plugins {
  name: "gz::sim::systems::AckermannSteering"
  filename: "gz-sim-ackermann-steering-system"
  innerxml: "<left_joint>front_left_wheel_joint</left_joint>\n        \n<left_joint>rear_left_wheel_joint</left_joint>\n        \n<right_joint>front_right_wheel_joint</right_joint>\n        \n<right_joint>rear_right_wheel_joint</right_joint>\n        \n<left_steering_joint>front_left_wheel_steering_joint</left_steering_joint>\n        \n<right_steering_joint>front_right_wheel_steering_joint</right_steering_joint>\n        \n<kingpin_width>1.0</kingpin_width>\n        \n<steering_limit>0.5</steering_limit>\n        \n<wheel_base>1.0</wheel_base>\n        \n<wheel_separation>1.25</wheel_separation>\n        \n<wheel_radius>0.3</wheel_radius>\n        \n<min_velocity>-1</min_velocity>\n        \n<max_velocity>1</max_velocity>\n        \n<min_acceleration>-3</min_acceleration>\n        \n<max_acceleration>3</max_acceleration>\n      "
}'
