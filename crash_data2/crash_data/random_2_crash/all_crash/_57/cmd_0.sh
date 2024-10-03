gz service --timeout 10000 -s /world/elevator_world/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: -2281947256
    nsec: 22823
  }
}
world_control {
  header {
    stamp {
      sec: -2005899801
      nsec: 16890
    }
    data {
      key: "z"
    }
  }
  step: true
  multi_step: 705449455
  reset {
    header {
      stamp {
        sec: -342327905
        nsec: 53438
      }
      data {
        key: "n"
      }
      data {
      }
    }
    all: true
    time_only: true
  }
  seed: 664978897
  run_to_sim_time {
    sec: -3048015883
    nsec: -36135
  }
}
state {
  header {
    stamp {
      sec: -2389450027
      nsec: -367
    }
    data {
    }
    data {
      key: "fm"
    }
  }
  entities {
    id: 14428191598868755071
    components {
      type: 8796657918424788791
      component: "im"
    }
    components {
      type: 6226233932943297730
      component: "dx"
    }
    remove: true
  }
  entities {
    id: 3638890783305280256
    components {
      type: 4220379421994952131
      remove: true
    }
    remove: true
  }
}'