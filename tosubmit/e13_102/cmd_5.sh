gz service --timeout 10000 -s /world/world_0/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: -2545817146
    nsec: 33208
  }
}
world_control {
  header {
    stamp {
      sec: 442963531
      nsec: 18815
    }
    data {
      key: "po"
    }
  }
  step: true
  multi_step: 2428035624
  reset {
    header {
      stamp {
        sec: -3828368081
        nsec: -13371
      }
    }
    all: true
    time_only: true
  }
  seed: 1706175810
  run_to_sim_time {
    sec: 1406654426
    nsec: 2047
  }
}
state {
  header {
    stamp {
      sec: 631710104
      nsec: 51047
    }
  }
}'