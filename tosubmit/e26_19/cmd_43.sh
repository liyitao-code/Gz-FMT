gz service --timeout 10000 -s /world/world_0/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: 3015194556
    nsec: -45808
  }
  data {
    key: "ka"
  }
}
world_control {
  header {
    stamp {
      sec: 3918315372
      nsec: -27493
    }
  }
  pause: true
  multi_step: 883680933
  reset {
    header {
      stamp {
        sec: 3670402974
        nsec: 44092
      }
      data {
        key: "xy"
      }
    }
    all: true
    model_only: true
  }
  seed: 3922869885
  run_to_sim_time {
    sec: -3052809341
    nsec: 30747
  }
}
state {
  header {
    stamp {
      sec: -1187820693
      nsec: -24968
    }
    data {
      key: "kg"
      value: "lq"
      value: "xp"
    }
  }
}'