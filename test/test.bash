#!/bin/bash
# SPDX-FileCopyrightText: 2025 Chuma Naoki
# SPDX-License-Identifier: BSD-3-Clause

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
rm -rf build install log
colcon build

source $dir/.bashrc
timeout 10 ros2 launch montecarlo_ros2 monte_carlo_publisher-result.launch.py > /tmp/mypkg.log
cat /tmp/mypkg.log | grep '試行回数:1 円周率: 0.0' || cat /tmp/mypkg.log | grep '試行回数:1 円周率: 4.0' 
