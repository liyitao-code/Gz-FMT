gz service --timeout 10000 -s /world/pose_publisher/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: 3195959616
    nsec: 23825
  }
  data {
    value: "z"
    value: "wj"
  }
}
pause: true
multi_step: -25296
seek {
  sec: 3365462042
  nsec: -55985
}'