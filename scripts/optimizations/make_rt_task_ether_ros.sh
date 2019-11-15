#!/bin/bash
ether_ros_pid=$(ps -e | grep "ether_ros" | grep -o -E '[0-9]+' | head -n 1)
sudo echo $ether_ros_pid > /sys/fs/cgroup/cpuset/rt/tasks

