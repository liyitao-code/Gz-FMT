gz service --timeout 10000 -s /world/default/control --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControl --req 'header {
  stamp {
    sec: 3497720081
    nsec: -58659
  }
  data {
    key: "k"
  }
}
pause: true
multi_step: 1600768289
reset {
  header {
    stamp {
      sec: -1821793761
      nsec: 5580
    }
  }
  all: true
  model_only: true
}
seed: 1602991481
run_to_sim_time {
  sec: -267344809
  nsec: 4289
}'