gz service --timeout 10000 -s /world/optical_tactile_plugin/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: 1824118375
    nsec: -29848
  }
}
world_control {
  header {
    stamp {
      sec: 1371961111
      nsec: -50039
    }
  }
  multi_step: 1243834979
  reset {
    header {
      stamp {
        sec: -1636800788
        nsec: -53831
      }
      data {
        key: "yr"
        value: "tx"
        value: "nu"
      }
      data {
      }
    }
    all: true
    model_only: true
  }
  seed: 1976813034
  run_to_sim_time {
    sec: -2984341925
    nsec: -13697
  }
}
state {
  header {
    stamp {
      sec: 3519499728
      nsec: -51166
    }
    data {
    }
    data {
      key: "jj"
    }
  }
  entities {
    id: 227129474721437073
    components {
      type: 10701323162408112325
      remove: true
    }
    components {
      type: 11043500331793972622
      component: "ju"
      remove: true
    }
    remove: true
  }
}'