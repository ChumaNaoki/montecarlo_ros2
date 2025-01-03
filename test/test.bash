#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
rm -rf build install log
colcon build
source $dir/.bashrc
ls /root/ros2_ws/src/mypkg
timeout 10 ros2 launch mypkg talk_listen.launch.py > /tmp/mypkg.log
cat /tmp/mypkg.log | grep '試行回数:1 円周率: 4.0'
