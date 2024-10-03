gz service --timeout 10000 -s /world/shapes/set_physics --reptype gz.msgs.Boolean --reqtype gz.msgs.Physics --req 'header {
  stamp {
    sec: 1710664369
    nsec: -22140
  }
}
type: SIMBODY
solver_type: "p"
min_step_size: 0.46608906544080697
precon_iters: -15312
iters: -19834
sor: 0.0553264926479351
cfm: 0.27846674747661404
erp: -0.3620897465295281
contact_max_correcting_vel: 0.6913353870376682
contact_surface_layer: 0.6149365247412351
gravity {
  header {
    stamp {
      sec: 1102504554
      nsec: 33902
    }
    data {
      key: "yr"
    }
    data {
      key: "y"
    }
  }
  x: -0.29376004833335223
  y: 0.28526816746886374
  z: 0.6745299134805125
}
enable_physics: true
real_time_factor: -0.9750897926765791
real_time_update_rate: -0.3316507620082352
max_step_size: 0.48902327619049824
profile_name: "gw"
magnetic_field {
  header {
    stamp {
      sec: 2407887342
      nsec: -51649
    }
    data {
      value: "t"
      value: "ro"
    }
  }
  x: 0.6146538543054703
  y: -0.15033586411332678
  z: -0.5657639953970297
}'