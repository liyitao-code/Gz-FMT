gz service --timeout 10000 -s /world/magnetometer_sensor/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: -1544622277
    nsec: -50886
  }
  data {
    key: "mh"
    value: "s"
  }
  data {
    key: "z"
    value: "r"
    value: ""
  }
}
pause: true
multi_step: -20051
rewind: true
seek {
  sec: 2316426735
  nsec: -12513
}'