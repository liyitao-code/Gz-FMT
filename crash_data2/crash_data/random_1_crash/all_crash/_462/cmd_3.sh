gz service --timeout 10000 -s /world/trajectory_follower/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: 3755453413
    nsec: -30853
  }
  data {
    value: "a"
  }
  data {
    value: "n"
  }
}
multi_step: 4670
seek {
  sec: 328295324
  nsec: 1570
}'