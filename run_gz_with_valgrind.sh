#!/bin/bash

# 设置 Gazebo 环境
source ~/workspace/gz_lastest/install/setup.bash

# 运行带 Valgrind 的 gz sim，但限制运行时间为 30 秒
timeout 5 valgrind --tool=memcheck \
         --leak-check=full \
         --show-leak-kinds=all \
         --track-origins=yes \
         --read-var-info=yes \
         --verbose \
         --log-file=valgrind-gz-sim.log \
         gz sim ~/workspace/rezilla-modelsmith-sim9/test_model/shapes.sdf

# 确保所有相关进程都已终止
pkill -f "gz sim"
pkill -f "valgrind"