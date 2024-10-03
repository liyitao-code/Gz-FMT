gz service --timeout 10000 -s /world/magnetometer_sensor/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: 941520218
    nsec: -26385
  }
  data {
    value: "i"
    value: "c"
  }
  data {
    value: ""
    value: ""
  }
}
world_control {
  header {
    stamp {
      sec: 2554600132
      nsec: -13968
    }
    data {
      key: "s"
      value: "pw"
    }
    data {
      key: "r"
      value: "ew"
    }
  }
  multi_step: 800578196
  reset {
    header {
      stamp {
        sec: -2485486537
        nsec: 20059
      }
      data {
        value: "d"
        value: "hb"
      }
      data {
        key: "dw"
        value: ""
      }
    }
    model_only: true
  }
  seed: 4174362349
  run_to_sim_time {
    sec: 3292046523
    nsec: -13450
  }
}
state {
  header {
    stamp {
      sec: -166103358
      nsec: 6427
    }
    data {
      key: "gv"
      value: ""
      value: ""
    }
  }
  entities {
    id: 1535308487235066634
    components {
      type: 14675646959611844883
      component: "vt"
    }
    components {
      type: 3058764801089753921
      remove: true
    }
  }
}'