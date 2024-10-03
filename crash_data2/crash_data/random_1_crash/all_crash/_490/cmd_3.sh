gz service --timeout 10000 -s /world/multi_lrauv/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: 2467209538
    nsec: -46539
  }
  data {
    key: "sr"
    value: "w"
  }
  data {
    key: "br"
    value: ""
  }
}
pause: true
multi_step: -20500
rewind: true
seek {
  sec: 2120475611
  nsec: -17638
}'