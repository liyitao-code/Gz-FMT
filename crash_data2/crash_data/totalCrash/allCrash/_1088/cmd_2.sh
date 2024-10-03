gz service --timeout 10000 -s /world/lift_drag/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: 1385991454
    nsec: 62400
  }
  data {
    key: "cu"
  }
  data {
    key: "gy"
  }
}
pause: true
multi_step: 40103
rewind: true
forward: true
seek {
  sec: 1821900634
  nsec: -20031
}'