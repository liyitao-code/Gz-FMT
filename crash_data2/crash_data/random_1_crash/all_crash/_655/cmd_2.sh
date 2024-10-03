gz service --timeout 10000 -s /world/default/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: 2848127379
    nsec: -5304
  }
  data {
    value: "ch"
  }
  data {
    key: "h"
    value: "b"
  }
}
multi_step: -50539
rewind: true
seek {
  sec: 786779308
  nsec: -3000
}'