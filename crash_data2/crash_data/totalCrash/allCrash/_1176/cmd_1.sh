gz service --timeout 10000 -s /world/default/set_physics --reptype gz.msgs.Boolean --reqtype gz.msgs.Physics --req 'header {
  stamp {
    sec: 3056403485
    nsec: -29006
  }
  data {
    key: "j"
  }
}
type: SIMBODY
solver_type: "c"
min_step_size: -0.656620593628529
precon_iters: -53706
iters: -46232
sor: 0.12762074875973028
cfm: 0.719025150789266
erp: 0.8707302526818779
contact_max_correcting_vel: -0.23031744201555138
contact_surface_layer: -0.7564362040404673
gravity {
  header {
    stamp {
      sec: -2497382989
      nsec: 19824
    }
    data {
      key: "u"
    }
  }
  x: 0.9970758870717429
  y: -0.6315326102072816
  z: 0.45837195102702766
}
real_time_factor: -0.7095645784095668
real_time_update_rate: 0.9361603666232312
max_step_size: -0.5258570802751827
magnetic_field {
  header {
    stamp {
      sec: 3362409038
      nsec: 5600
    }
    data {
      key: "q"
      value: "v"
      value: "tm"
    }
    data {
      key: "pb"
      value: "d"
    }
  }
  x: -0.9754844507039371
  y: -0.8771749349655518
  z: 0.9120259352305069
}'