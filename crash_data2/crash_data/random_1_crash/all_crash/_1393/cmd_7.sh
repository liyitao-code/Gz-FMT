gz service --timeout 10000 -s /world/buoyant_tethys/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: -3455434523
    nsec: -26422
  }
  data {
    value: "rs"
  }
  data {
    value: "t"
  }
}
pause: true
multi_step: 13422
rewind: true
seek {
  sec: 2953825646
  nsec: -57077
}'