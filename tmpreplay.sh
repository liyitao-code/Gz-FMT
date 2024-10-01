
#!/usr/bin/env bash
echo $1

pwd

for i in {0..227};
do
    gz sim /tmp/exp/a.sdf --seed 1719669644 -r -v 0 -s --headless-rendering 2> /tmp/exp/gz_${i}.replay &
    sleep 5
    bash /tmp/exp/${i}_1.sh
    bash /tmp/exp/${i}_2.sh
    sleep 1
    pkill -9 ruby
done

