gz service --timeout 10000 -s /world/auto_inertia_rolling_shapes/control --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControl --req 'header {
  stamp {
    sec: 4253968596
    nsec: 5817
  }
}
multi_step: 746044717
reset {
  header {
    stamp {
      sec: 3061063116
      nsec: -9240
    }
    data {
    }
  }
  all: true
  model_only: true
}
seed: 2334200483
run_to_sim_time {
  sec: 3682622369
  nsec: 40663
}'