gz service --timeout 10000 -s /world/multicopter/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: 2792688494
    nsec: -2805
  }
  data {
    key: "hc"
    value: "y"
    value: "l"
  }
}
pause: true
multi_step: 31043
rewind: true
forward: true
seek {
  sec: 2536573078
  nsec: 60497
}'