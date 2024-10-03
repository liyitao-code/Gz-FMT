gz service --timeout 10000 -s /world/default/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 28
}
plugins {
  name: "gz::sim::systems::Breadcrumbs"
  filename: "gz-sim-breadcrumbs-system"
  innerxml: "<topic>/B2/deploy</topic>\n       \n<disable_physics_time>2.0</disable_physics_time>\n       \n<max_deployments>3</max_deployments>\n        \n<breadcrumb>\n          <sdf version=\"1.6\">\n            <model name=\"B2\">\n              <pose>9770 500242 492404 0.0 500504 506103</pose>\n              <include>\n                <uri>\n                  https://fuel.gazebosim.org/1.0/OpenRobotics/models/X2 Config 1\n                </uri>\n              </include>\n            </model>\n          </sdf>\n        </breadcrumb>"
}
'