gz service --timeout 10000 -s /world/contact_sensor/control/state --reptype gz.msgs.Boolean --reqtype gz.msgs.WorldControlState --req 'header {
  stamp {
    sec: 2950201956
    nsec: -59503
  }
}
world_control {
  header {
    stamp {
      sec: -3606368148
      nsec: -21877
    }
  }
  step: true
  multi_step: 3223645962
  reset {
    header {
      stamp {
        sec: 2413271679
        nsec: -1781
      }
    }
    all: true
  }
  seed: 3108088678
  run_to_sim_time {
    sec: -4230593901
    nsec: -37963
  }
}
state {
  header {
    stamp {
      sec: -1346594986
      nsec: -33206
    }
    data {
    }
  }
  entities {
    id: 16466922207360023430
    components {
      type: 9127288713655301703
      component: "k"
    }
    remove: true
  }
}'