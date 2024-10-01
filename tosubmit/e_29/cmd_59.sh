gz service --timeout 10000 -s /world/world_0/control --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControl --req 'header {
  stamp {
    sec: -3446440492
    nsec: -21338
  }
  data {
    value: ""
    value: "s"
  }
}
pause: true
step: true
multi_step: 2243887140
reset {
  header {
    stamp {
      sec: -3781190652
      nsec: 6454
    }
  }
  all: true
  model_only: true
}
seed: 2500585059
run_to_sim_time {
  sec: -3575437745
  nsec: 53543
}'