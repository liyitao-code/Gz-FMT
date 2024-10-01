#!/usr/bin/env bash

for i in {001..999}
do
    python smith.py
    gz sim a.sdf -r -s --headless-rendering 2>&1 | tee exp_one_shot/gz_${i}.log &
    sleep 6
    python servicesmith.py -d exp_one_shot -i $i -m one_shot
    sleep 6
    gz service -s /server_control --reqtype gz.msgs.ServerControl --reptype gz.msgs.Boolean --req 'stop: true' --timeout 5000
    sleep 6
    mv a.sdf exp_one_shot/world_${i}.sdf

done
