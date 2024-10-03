gz service --timeout 10000 -s /world/auto_inertia_pendulum/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: -580612708
    nsec: 25966
  }
  data {
    key: "b"
  }
  data {
    key: "q"
  }
}
pause: true
multi_step: 13770
rewind: true
seek {
  sec: 2766932071
  nsec: -26072
}'