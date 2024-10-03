gz service --timeout 10000 -s /world/track_drive/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: 3198468735
    nsec: 35265
  }
  data {
    key: "ow"
  }
  data {
  }
}
pause: true
multi_step: -13487
rewind: true
forward: true
seek {
  sec: 1802628912
  nsec: 46790
}'