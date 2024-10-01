#!/usr/bin/env bash
gz sim a.txt -r &
gz service --timeout 10000 -s /model/model/battery/linear_battery/recharge/start --reptype gz.msgs.Empty --reqtype gz.msgs.Boolean --req ''
gz service --timeout 10000 -s /world/world_0/remove --reptype gz.msgs.Boolean --reqtype gz.msgs.Entity --req 'id: 4'
