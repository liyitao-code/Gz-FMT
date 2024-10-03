gz service --timeout 10000 -s /world/default/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: 2858621453
    nsec: -16914
  }
  data {
    value: "hk"
    value: "q"
  }
}
multi_step: -32816
rewind: true
forward: true
seek {
  sec: 4206127295
  nsec: 43358
}'