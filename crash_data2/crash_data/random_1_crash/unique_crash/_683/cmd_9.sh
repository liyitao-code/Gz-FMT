gz service --timeout 10000 -s /world/shapes/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: 2952195538
    nsec: 52383
  }
  data {
  }
}
world_control {
  header {
    stamp {
      sec: 696482789
      nsec: -22737
    }
    data {
      value: ""
      value: "sr"
    }
    data {
      key: "f"
      value: ""
    }
  }
  multi_step: 3272484965
  reset {
    header {
      stamp {
        sec: 257319675
        nsec: 45797
      }
      data {
        key: "n"
        value: "en"
        value: "c"
      }
      data {
        key: "f"
        value: "sw"
      }
    }
    time_only: true
  }
  seed: 3976686413
  run_to_sim_time {
    sec: -3224982330
    nsec: -11287
  }
}
state {
  header {
    stamp {
      sec: 2432148606
      nsec: 43298
    }
    data {
    }
  }
}'