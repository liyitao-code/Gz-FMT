gz service --timeout 10000 -s /gazebo/resource_paths/add --reptype gz.msgs.Empty --reqtype gz.msgs.StringMsg_V --req 'header {
  stamp {
    sec: 3733277423
    nsec: 32890
  }
  data {
    key: "ah"
  }
  data {
    key: "a"
    value: "b"
    value: ""
  }
}
data: "tb"'