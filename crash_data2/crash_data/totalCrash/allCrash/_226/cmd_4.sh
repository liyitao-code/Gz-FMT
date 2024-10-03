gz service --timeout 10000 -s /world/dvl_world/control --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControl --req 'header {
  stamp {
    sec: -3027649146
    nsec: -13189
  }
}
pause: true
multi_step: 220322653
reset {
  header {
    stamp {
      sec: 1911863465
      nsec: -33248
    }
  }
  all: true
  model_only: true
}
seed: 1119563627
run_to_sim_time {
  sec: 4235032014
  nsec: 7866
}'