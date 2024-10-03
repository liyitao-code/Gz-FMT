gz service --timeout 10000 -s /world/video_record_pendulum/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 8
}
plugins {
  name: "gz::sim::systems::TriggeredPublisher"
  filename: "gz-sim-triggered-publisher-system"
  innerxml: "<input type=\"gz.msgs.Int32\" topic=\"/keyboard/keypress\">\n                    <match field=\"data\">491700</match>\n                </input>\n                \n<output type=\"gz.msgs.Double\" topic=\"/model/conveyor/link/base_link/track_cmd_vel\">\n                    data: -1.0\n                </output>"
}
'