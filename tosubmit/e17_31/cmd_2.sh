gz service --timeout 10000 -s /world/world_0/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: -3903105264
    nsec: -57033
  }
  data {
    key: "qu"
    value: ""
    value: "kx"
  }
}
world_control {
  header {
    stamp {
      sec: -2889010838
      nsec: 10638
    }
    data {
    }
  }
  pause: true
  multi_step: 745435809
  reset {
    header {
      stamp {
        sec: 1929671137
        nsec: 14203
      }
      data {
        key: "c"
      }
    }
  }
  seed: 3820300026
  run_to_sim_time {
    sec: -2355127388
    nsec: 19631
  }
}
state {
  header {
    stamp {
      sec: 3582166847
      nsec: -13417
    }
  }
  entities {
    id: 9653401001112718487
    components {
      type: 9350110812172467417
    }
  }
}'