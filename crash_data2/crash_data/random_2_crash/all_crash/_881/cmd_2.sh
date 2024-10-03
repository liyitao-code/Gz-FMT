gz service --timeout 10000 -s /world/odometer_world/playback/control --reptype gz.msgs.Boolean --reqtype gz.msgs.LogPlaybackControl --req 'header {
  stamp {
    sec: -2003169250
    nsec: 2277
  }
  data {
    value: "h"
    value: "lz"
  }
}
multi_step: -57630
rewind: true
forward: true
seek {
  sec: 3023301944
  nsec: -45872
}'