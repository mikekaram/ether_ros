#!/bin/sh
#enable the creation of cpuset folder
mount -t tmpfs none /sys/fs/cgroup
#create the cpuset folder and mount the cgroup filesystem
mkdir /sys/fs/cgroup/cpuset/
mount -t cgroup -o cpuset none /sys/fs/cgroup/cpuset/
#create the partitions
mkdir /sys/fs/cgroup/cpuset/rt
mkdir /sys/fs/cgroup/cpuset/nrt

# add the general purpose CPUs to the nRT set:
echo 0,1 > /sys/fs/cgroup/cpuset/nrt/cpuset.cpus

# add the real-time CPUs to the RT set:
echo 2,3 > /sys/fs/cgroup/cpuset/rt/cpuset.cpus

# make the CPUs in the RT set exclusive, i.e. do not let tasks in other sets use them:
echo 1 > /sys/fs/cgroup/cpuset/rt/cpuset.cpu_exclusive


## Restart real-time CPUs with CPU hotplug


# if you don't remember your root password, then run sudo passwd root
# su root

# Turn off all CPUs in the real-time partition:
# sudo echo 0 > /sys/devices/system/cpu/cpu3/online
# sudo echo 0 > /sys/devices/system/cpu/cpu2/online

# Then turn them on:
# sudo echo 1 > /sys/devices/system/cpu/cpu3/online
# sudo echo 1 > /sys/devices/system/cpu/cpu2/online


## Not NUMA-enabled


# Associate the nRT set with NUMA node 0:
echo 0 > /sys/fs/cgroup/cpuset/nrt/cpuset.mems

# Associate the RT set with NUMA node 0:
echo 0 > /sys/fs/cgroup/cpuset/rt/cpuset.mems


## Configure load balancing


# Disable load balancing in the root cpuset. This is necessary for settings in the child cpusets to take effect:
echo 0 > /sys/fs/cgroup/cpuset/cpuset.sched_load_balance

# Then disable load balancing in the RT cpuset:
echo 0 > /sys/fs/cgroup/cpuset/rt/cpuset.sched_load_balance

# Finally enable load balancing in the nRT cpuset:
echo 1 > /sys/fs/cgroup/cpuset/nrt/cpuset.sched_load_balance


## Move general purpose tasks to the general GP partition


# For each task in the root cpuset, run the following command, where each pid of task should occur on its own line:
IFS=$'\r\n' GLOBIGNORE='*' command eval  'cpuset_pids=($(cat /sys/fs/cgroup/cpuset/tasks))'
for i in "${cpuset_pids[@]}";
do
echo $i; echo $i > /sys/fs/cgroup/cpuset/nrt/tasks;
done
# echo pid_of_task > /sys/fs/cgroup/cpuset/nrt/tasks

## Move IRQs to the general purpose CPUs

# Some interrupts are not CPU-bound. Unwanted interrupts introduce jitter and can have serious negative impact on real-time performance. They should be handled on the general purpose CPUs whenever possible. The affinity of these interrupts can be controlled using the /proc file system.
# First set the default affinity to CPU0 or CPU1 to make sure that new interrupts won’t be handled by the real-time CPUs. The set {CPU0, CPU1} is represented as a bitmask set to 3, (20 + 21)..
echo 3 > /proc/irq/default_smp_affinity


cd /proc/irq
irq_array=($(ls -d */ | cut -f1 -d'/'))
for i in "${cpuset_pids[@]}";
do
echo $i; echo 3 > /proc/irq/$i/smp_affinity;
done
# Move IRQs to the nRT partition
echo 3 > /proc/irq/<irq>/smp_affinity

# Interrupts that can not be moved will be printed to stderr. When it is known what interrupts can not be moved, consult the hardware and driver documentation to see if this will be an issue. It might be possible to disable the device that causes the interrupt.

# Typical interrupts that should and can be moved are: certain timer interrupts, network related interrupts and serial interface interrupts. If there are any interrupts that are part of the real-time application, they should of course be configured to fire in the real-time partition.


## Execute a task in the real-time partition

# Now it is possible to run a real-time task in the real-time partition:
# echo pid_of_task > /sys/fs/cgroup/cpusets/rt/tasks

# Since we have an RT partition with more then one CPU we might want to choose a specific CPU to run on. Change the task affinity to only include CPU3 in the real-time partition: $ taskset -p 0x8 pid_of_task
