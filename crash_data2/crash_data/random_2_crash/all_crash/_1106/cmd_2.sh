gz service --timeout 10000 -s /world/default/set_physics --reptype gz.msgs.Boolean --reqtype gz.msgs.Physics --req 'header {
  stamp {
    sec: -1394051662
    nsec: -929
  }
}
type: BULLET
solver_type: "kn"
min_step_size: -0.9892941095440426
precon_iters: 35294
iters: 7716
sor: 0.9108524923754495
cfm: 0.8677526645181157
erp: 0.4116136895490634
contact_max_correcting_vel: -0.17082858967223502
contact_surface_layer: -0.47406960669690124
gravity {
  header {
    stamp {
      sec: -3539893505
      nsec: 11639
    }
    data {
      value: "ww"
    }
  }
  x: -0.17320128078930597
  y: 0.16597280027998873
  z: 0.22684687818865457
}
enable_physics: true
real_time_factor: 0.4165444695192737
real_time_update_rate: -0.07934968565357936
max_step_size: 0.21204352899446532
magnetic_field {
  header {
    stamp {
      sec: 2831939024
      nsec: -45171
    }
  }
  x: -0.4173798582506203
  y: -0.05119965766083401
  z: -0.486392038817854
}'