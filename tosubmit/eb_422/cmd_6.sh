gz service --timeout 10000 -s /world/optical_tactile_plugin/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 17
}
plugins {
  name: "gz::sim::systems::TriggeredPublisher"
  filename: "gz-sim-triggered-publisher-system"
  innerxml: "<input type=\"gz.msgs.Int32\" topic=\"/keyboard/keypress\">\n                    <match field=\"data\">65</match>\n                </input>\n                \n<output type=\"gz.msgs.Twist\" topic=\"/model/simple_tracked/cmd_vel\">\n                    linear: {x: 0.0}, angular: {z: 1.0}\n                </output>\n            "
}
'