gz service --timeout 10000 -s /world/diff_drive/set_physics --reptype gz.msgs.Boolean --reqtype gz.msgs.Physics --req 'header {
  stamp {
    sec: -1149875504
    nsec: 24008
  }
  data {
    key: "e"
    value: "qk"
  }
}
type: BULLET
solver_type: "w"
min_step_size: 0.448214122794943
precon_iters: -36351
iters: -17033
sor: -0.09149165081132082
cfm: 0.8685849500876663
erp: -0.04539639914468574
contact_max_correcting_vel: -0.7536824046388857
contact_surface_layer: -0.46839929380497747
gravity {
  header {
    stamp {
      sec: -3327563633
      nsec: -38110
    }
    data {
      key: "mo"
      value: "v"
    }
    data {
      key: "o"
      value: "w"
    }
  }
  x: -0.9032230719189029
  y: -0.49632656490794
  z: 0.5013733431486704
}
enable_physics: true
real_time_factor: -0.6964766367778747
real_time_update_rate: -0.765121374842413
max_step_size: 0.1287993489424144
profile_name: "hy"
magnetic_field {
  header {
    stamp {
      sec: -663266145
      nsec: 27309
    }
    data {
      key: "nd"
      value: "mj"
    }
  }
  x: 0.02185796086610159
  y: 0.5052919294863745
  z: -0.8646231491508334
}'