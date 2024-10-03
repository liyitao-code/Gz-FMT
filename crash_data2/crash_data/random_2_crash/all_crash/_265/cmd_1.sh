gz service --timeout 10000 -s /world/default/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: -1106201506
    nsec: 40834
  }
}
world_control {
  header {
    stamp {
      sec: -616394377
      nsec: 2748
    }
    data {
      key: "ro"
      value: ""
    }
    data {
    }
  }
  multi_step: 3992999204
  reset {
    header {
      stamp {
        sec: -442290784
        nsec: -30634
      }
      data {
        key: "h"
        value: ""
        value: "qm"
      }
    }
    time_only: true
    model_only: true
  }
  seed: 2514096112
  run_to_sim_time {
    sec: 1350561243
    nsec: 60598
  }
}
state {
  header {
    stamp {
      sec: 96473832
      nsec: 10337
    }
    data {
      key: "bu"
      value: "m"
    }
  }
}'