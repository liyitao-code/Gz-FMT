gz service --timeout 10000 -s /world/contact_sensor/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: -10145398
    nsec: 10637
  }
  data {
    key: "ow"
  }
  data {
    key: "iy"
  }
}
multi_step: 30086
forward: true
seek {
  sec: 1685832690
  nsec: 32217
}'