gz service --timeout 10000 -s /world/shapes/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 4
}
plugins {
  name: "gz::sim::systems::CommsEndpoint"
  filename: "gz-sim-comms-endpoint-system"
  innerxml: "<address>addr2</address>\n        \n<topic>addr2/rx</topic>\n        \n<broker>\n          <bind_service>/broker/bind</bind_service>\n          <unbind_service>aLhjs5D9xmQSDI</unbind_service>\n        </broker>"
}
'