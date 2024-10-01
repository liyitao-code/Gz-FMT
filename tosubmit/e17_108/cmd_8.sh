gz service --timeout 10000 -s /world/world_0/control --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControl --req 'header {
  stamp {
    sec: 3938205359
    nsec: -63100
  }
  data {
    key: "m"
  }
}
pause: true
step: true
multi_step: 189938597
reset {
  header {
    stamp {
      sec: 540864211
      nsec: -39283
    }
    data {
      value: "um"
    }
    data {
      key: "p"
      value: "r"
    }
  }
  all: true
  time_only: true
}
seed: 866702842
run_to_sim_time {
  sec: 1160507740
  nsec: 23815
}'