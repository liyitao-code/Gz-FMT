gz service --timeout 10000 -s /world/magnetometer_sensor/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 4
}
plugins {
  name: "gz::sim::systems::TrackedVehicle"
  filename: "gz-sim-tracked-vehicle-system"
  innerxml: "<left_track><link>left_track</link></left_track>\n                \n<left_track><link>front_left_flipper</link></left_track>\n                \n<left_track><link>rear_left_flipper</link></left_track>\n                \n<right_track><link>XlR1D1r2X7T</link></right_track>\n                \n<right_track><link>front_right_flipper</link></right_track>\n                \n<right_track><link>rear_right_flipper</link></right_track>\n                \n<tracks_separation>0.4</tracks_separation>\n                \n<tracks_height>0.18094</tracks_height>\n                \n<steering_efficiency>0.5</steering_efficiency>\n                \n<!--debug>1</debug-->"
}
'