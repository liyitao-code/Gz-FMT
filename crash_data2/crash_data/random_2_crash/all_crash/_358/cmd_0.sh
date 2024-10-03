gz service --timeout 10000 -s /world/default/entity/system/add --reptype gz.msgs.Boolean --reqtype gz.msgs.EntityPlugin_V --req 'entity {
  id: 4
}
plugins {
  name: "gz::sim::systems::CommsEndpoint"
  filename: "gz-sim-comms-endpoint-system"
  innerxml: "<address>addr2</address>\n        \n<topic>addr2/rx</topic>\n        \n<broker>\n          <bind_service>y3sQkXQZeDW9</bind_service>\n          <unbind_service>/broker/unbind</unbind_service>\n        </broker>"
}
'