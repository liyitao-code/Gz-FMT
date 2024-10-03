gz service --timeout 10000 -s /world/detachable_joint/control --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControl --req 'header {
  stamp {
    sec: -1281458689
    nsec: 50322
  }
}
multi_step: 3067269715
reset {
  header {
    stamp {
      sec: 729012607
      nsec: 1847
    }
    data {
      value: "dy"
    }
    data {
      value: "ob"
      value: "pb"
    }
  }
  time_only: true
}
seed: 838287520
run_to_sim_time {
  sec: -1075851181
  nsec: -51018
}'