#!/usr/bin/env bash
echo $1

cd $1
gz sim a.sdf -r -v 0 -s --headless-rendering 2> gz.replay &
# gz sim a.sdf --seed 1722066148 -r -v 0 -s --headless-rendering 2> gz.replay &
sleep 5
pwd

iteration=$(cat id)
echo $iteration
range=${@:2}
if [[ $range ]]; then
    loop=$(eval echo $range)
else
    loop=$(seq 0 $iteration)
fi

for i in $loop;
do
    if pgrep ruby > /dev/null
    then
        echo $i
        bash cmd_${i}.sh
        gz service --timeout 10000 -s /world/world_0/generate_world_sdf --reptype gz.msgs.StringMsg --reqtype gz.msgs.SdfGeneratorConfig --req '' > world_${i}.sdf
    fi

done

sleep 10
echo before dump
gz service --timeout 10000 -s /world/world_0/generate_world_sdf --reptype gz.msgs.StringMsg --reqtype gz.msgs.SdfGeneratorConfig --req '' > /dev/null
echo after dump
echo after cmd

cd ..
pkill -9 ruby
