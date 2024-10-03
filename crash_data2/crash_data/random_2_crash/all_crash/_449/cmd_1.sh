gz service --timeout 10000 -s /world/advanced_lift_drag/set_physics --reptype gz.msgs.Boolean --reqtype gz.msgs.Physics --req 'header {
  stamp {
    sec: 1766618015
    nsec: 35919
  }
  data {
    value: ""
  }
}
type: SIMBODY
min_step_size: -0.5365066119011743
precon_iters: -14298
iters: 25893
sor: -0.3922428554042634
cfm: 0.8602680737329831
erp: 0.3509692568269549
contact_max_correcting_vel: 0.3914448574824778
contact_surface_layer: -0.789029586646337
gravity {
  header {
    stamp {
      sec: 942485553
      nsec: -48610
    }
    data {
      key: "g"
      value: ""
    }
  }
  x: -0.5122811242037129
  y: -0.1779560229374908
  z: -0.20744917971032995
}
enable_physics: true
real_time_factor: -0.9702215699279535
real_time_update_rate: -0.5894897770885839
max_step_size: 0.7090007066535591
profile_name: "x"
magnetic_field {
  header {
    stamp {
      sec: 1829057108
      nsec: -7699
    }
    data {
      key: "di"
      value: "f"
      value: ""
    }
  }
  x: 0.936130711078166
  y: -0.9143740260960933
  z: 0.7991480413866392
}'