gz service --timeout 10000 -s /world/lift_drag/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: -2214363871
    nsec: -35346
  }
  data {
    value: ""
  }
  data {
    value: "q"
  }
}
world_control {
  header {
    stamp {
      sec: -3867850698
      nsec: -47887
    }
    data {
      value: "j"
      value: ""
    }
    data {
      key: "c"
    }
  }
  multi_step: 891910872
  reset {
    header {
      stamp {
        sec: 419503848
        nsec: -28734
      }
    }
    all: true
    time_only: true
    model_only: true
  }
  seed: 3297435956
  run_to_sim_time {
    sec: 2807035679
    nsec: -46108
  }
}
state {
  header {
    stamp {
      sec: 958674381
      nsec: -60036
    }
    data {
      key: "q"
      value: "r"
    }
  }
  entities {
    id: 10351543832191236486
    remove: true
  }
}'