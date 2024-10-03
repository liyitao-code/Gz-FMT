gz service --timeout 10000 -s /world/magnetometer_sensor/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: -3707821102
    nsec: -25316
  }
  data {
    key: "r"
    value: "p"
  }
}
pause: true
multi_step: 444
forward: true
seek {
  sec: 1298642856
  nsec: 467
}'