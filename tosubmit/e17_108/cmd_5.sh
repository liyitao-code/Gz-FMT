gz service --timeout 10000 -s /world/world_0/set_physics --reptype gz.msgs.Boolean --reqtype gz.msgs.Physics --req 'header {
  stamp {
    sec: 3638332969
    nsec: -22661
  }
  data {
    key: "it"
  }
}
solver_type: "g"
min_step_size: -0.944065885377491
precon_iters: -16313
iters: -32320
sor: -0.5699017842131129
cfm: 0.8039719373758509
erp: 0.7216826539847947
contact_max_correcting_vel: 0.7347593745802405
contact_surface_layer: -0.012760873495838654
gravity {
  header {
    stamp {
      sec: -2761626002
      nsec: -53864
    }
  }
  x: -0.298927353198853
  y: 0.23545036166611588
  z: 0.1268710038589267
}
enable_physics: true
real_time_factor: 0.04110548925736257
real_time_update_rate: -0.2739499423912075
max_step_size: -0.8910461609807379
profile_name: "kl"
magnetic_field {
  header {
    stamp {
      sec: -2951990000
      nsec: -47208
    }
    data {
      value: ""
    }
    data {
      key: "v"
    }
  }
  x: -0.7304609294650386
  y: -0.5929536496702783
  z: 0.5955039465274043
}'