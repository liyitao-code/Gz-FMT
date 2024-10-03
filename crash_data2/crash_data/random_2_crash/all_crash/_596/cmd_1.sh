gz service --timeout 10000 -s /world/default/control --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControl --req 'header {
  stamp {
    sec: 3056720444
    nsec: -60526
  }
}
pause: true
step: true
multi_step: 2676679076
reset {
  header {
    stamp {
      sec: -2503698444
      nsec: 37650
    }
  }
  time_only: true
  model_only: true
}
seed: 4160882023
run_to_sim_time {
  sec: 2742545140
  nsec: 25951
}'