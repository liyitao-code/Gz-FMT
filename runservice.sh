#!/usr/bin/env bash
# -s --headless-rendering 
./cleanup.sh
gz sim a.sdf -r 2>&1 | tee -a gz.log &
sleep 3
echo abc
find /data/play/robot/workspace/build/ -name '*.gcda'
echo def
python servicesmith.py -d exp -m loop 2>&1 | tee  py.log
# while true; do      
#     if pgrep ruby > /dev/null; then
#         python servicesmith.py
#     else
#         break;
#     fi
# done

