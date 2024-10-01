gz service --timeout 10000 -s /world/optical_tactile_plugin/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: 3988472702
    nsec: -7213
  }
  data {
    key: "u"
    value: ""
    value: ""
  }
}
world_control {
  header {
    stamp {
      sec: 434229994
      nsec: -12520
    }
  }
  pause: true
  step: true
  multi_step: 833568079
  reset {
    header {
      stamp {
        sec: 158391178
        nsec: 29971
      }
      data {
        key: "c"
        value: ""
        value: "p"
      }
      data {
        key: "xn"
        value: ""
        value: "g"
      }
    }
    all: true
    time_only: true
    model_only: true
  }
  seed: 3080137427
  run_to_sim_time {
    sec: 10074272
    nsec: 15159
  }
}
state {
  header {
    stamp {
      sec: 2898107380
      nsec: -7019
    }
    data {
      key: "g"
      value: "g"
    }
  }
}'