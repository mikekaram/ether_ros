#!/bin/bash
ighm_ros_pid=$(ps -e | grep "ighm_ros" | grep -o -E '[0-9]+' | head -n 1)
sudo echo $ighm_ros_pid > /sys/fs/cgroup/cpuset/rt/tasks

