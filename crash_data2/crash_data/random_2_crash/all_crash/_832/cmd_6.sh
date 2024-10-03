gz service --timeout 10000 -s /world/quadrotor/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: 14106099
    nsec: -13859
  }
  data {
    key: "oe"
    value: "r"
  }
}
pause: true
multi_step: -4612
forward: true
seek {
  sec: 1377895341
  nsec: 31365
}'