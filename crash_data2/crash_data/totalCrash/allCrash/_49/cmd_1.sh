gz service --timeout 10000 -s /world/deformable_sphere/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 10
}
plugins {
  name: "gz::sim::systems::PerformerDetector"
  filename: "gz-sim-performer-detector-system"
  innerxml: "<topic>/performer_detector</topic>\n        \n<geometry>\n          <box>\n            <size>-10 -3006 -5</size>\n          </box>\n        </geometry>"
}
'