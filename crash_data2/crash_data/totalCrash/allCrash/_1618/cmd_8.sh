gz service --timeout 10000 -s /world/wheel_slip/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: -2904212450
    nsec: 42034
  }
  data {
    key: "m"
    value: "pk"
    value: ""
  }
}
pause: true
multi_step: -19577
rewind: true
forward: true
seek {
  sec: 939865912
  nsec: 47095
}'