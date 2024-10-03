gz service --timeout 10000 -s /world/optical_tactile_plugin/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: 3067724396
    nsec: -62803
  }
}
world_control {
  header {
    stamp {
      sec: -3074432582
      nsec: 22195
    }
    data {
      key: "t"
    }
    data {
      key: "z"
    }
  }
  multi_step: 1115103884
  reset {
    header {
      stamp {
        sec: 2541475519
        nsec: -45060
      }
      data {
        key: "y"
        value: "im"
      }
      data {
        key: "yn"
      }
    }
    all: true
  }
  seed: 3768056978
  run_to_sim_time {
    sec: 3195752565
    nsec: -16013
  }
}
state {
  header {
    stamp {
      sec: -2163048736
      nsec: -42550
    }
    data {
      key: "lw"
      value: ""
    }
    data {
      value: "x"
      value: ""
    }
  }
  entities {
    id: 14797711240781981686
    remove: true
  }
  entities {
    id: 5019293988613660528
    components {
      type: 14989293473826698134
      remove: true
    }
    remove: true
  }
}'