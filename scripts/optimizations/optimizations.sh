#!/bin/bash

## Create CPU isolation with cgroups (Probably already done with isolcpus boot parameter)


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
# Restart is not needed, because the CPUs are isolated from boot.

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

enp5s0_irq_pid=$(ps -e | grep "enp5s0$" | grep -o -E '[0-9]+' | head -n 1)
echo $enp5s0_irq_pid; echo $enp5s0_irq_pid > /sys/fs/cgroup/cpuset/rt/tasks;
enp5s0_r_irq_pid=$(ps -e | grep "enp5s0-r" | grep -o -E '[0-9]+' | head -n 1)
echo $enp5s0_r_irq_pid; echo $enp5s0_r_irq_pid > /sys/fs/cgroup/cpuset/rt/tasks;
enp5s0_t_irq_pid=$(ps -e | grep "enp5s0-t" | grep -o -E '[0-9]+' | head -n 1)
echo $enp5s0_t_irq_pid; echo $enp5s0_t_irq_pid > /sys/fs/cgroup/cpuset/rt/tasks;

## Move IRQs to the general purpose CPUs

# Some interrupts are not CPU-bound. Unwanted interrupts introduce jitter and can have serious negative impact on real-time performance. They should be handled on the general purpose CPUs whenever possible. The affinity of these interrupts can be controlled using the /proc file system.
# First set the default affinity to CPU0 or CPU1 to make sure that new interrupts won’t be handled by the real-time CPUs. The set {CPU0, CPU1} is represented as a bitmask set to 3, (20 + 21)..
echo 3 > /proc/irq/default_smp_affinity


# Move IRQs to the nRT partition
# echo 3 > /proc/irq/<irq>/smp_affinity

# Interrupts that can not be moved will be printed to stderr. When it is known what interrupts can not be moved, consult the hardware and driver documentation to see if this will be an issue. It might be possible to disable the device that causes the interrupt.

# Typical interrupts that should and can be moved are: certain timer interrupts, network related interrupts and serial interface interrupts. If there are any interrupts that are part of the real-time application, they should of course be configured to fire in the real-time partition.

cd /proc/irq
irq_array=($(ls -d */ | cut -f1 -d'/'))
for i in "${irq_array[@]}";
do
echo $i; echo 3 > /proc/irq/$i/smp_affinity;
done
enp5s0_irq_num=$(ps -e | grep "enp5s0$" | grep -o -E '[0-9]+' | head -n 5 | tail -1)
enp5s0_r_irq_num=$(ps -e | grep "enp5s0-r" | grep -o -E '[0-9]+' | head -n 5 | tail -1)
enp5s0_t_irq_num=$(ps -e | grep "enp5s0-t" | grep -o -E '[0-9]+' | head -n 5 | tail -1)
echo 8 > /proc/irq/$enp5s0_irq_num/smp_affinity
echo 8 > /proc/irq/$enp5s0_r_irq_num/smp_affinity
echo 8 > /proc/irq/$enp5s0_t_irq_num/smp_affinity

## Network queues affinity

# Linux can route the packets on different CPUs in an SMP system. Also this handling can create timers on the specific CPUs, an example is the ARP timer management, based on neigh_timer. There are a couple of solutions that can be adopted to minimize the effect of rerouting packets on different CPUs, like migrating all the timers on the non-realtime partition if possible, specifying the affinity of network queues on some architectures.

# If the application needs the packets to be received only in the nRT or RT partition then the affinity should be set as follows:
# echo <NRT cpus mask> > /sys/class/net/<non EtherCAT interface>/queues/<queue>/<x/r>ps_cpus
# echo <RT cpus mask> > /sys/class/net/<EtherCAT interface>/queues/<queue>/<x/r>ps_cpus
echo 8 > /sys/class/net/enp5s0/queues/rx-0/rps_cpus
echo 8 > /sys/class/net/enp5s0/queues/tx-0/xps_cpus
echo 3 > /sys/class/net/enp6s0/queues/rx-0/rps_cpus
echo 3 > /sys/class/net/enp6s0/queues/tx-0/xps_cpus

## Execute a task in the real-time partition

# Now it is possible to run a real-time task in the real-time partition:
# echo pid_of_task > /sys/fs/cgroup/cpusets/rt/tasks

# Since we have an RT partition with more then one CPU we might want to choose a specific CPU to run on. Change the task affinity to only include CPU3 in the real-time partition: $ taskset -p 0x8 pid_of_task. . This is done in the code (ether_ros), so no need to be done externally.

## Time Stamp Counter (tsc - x86 only)

# The time stamp counter is a per-CPU counter for producing time stamps. Since the counters might drift a bit, Linux will periodically check that they are synchronized. But this periodicity means that the tick might appear despite using full dynamic ticks.

# By telling Linux that the counters are reliable, Linux will no longer perform the periodic synchronization. The side effect of this is that the counters may start to drift, something that can be visible in trace logs for example.

# Here is an example of how to use it:

# isolcpus=2,3 nohz_full=2,3 tsc=reliable


## Delay vmstat timer


# It is used for collecting virtual memory statistics.The statistics are updated at an interval specified as seconds in /proc/sys/vm/stat_interval. The amount of jitter can be reduced by writing a large value to this file. However, that will not solve the issue with worst-case latency.

# Example (10000 seconds):
echo 10000 > /proc/sys/vm/stat_interval


# BDI writeback affinity

# It is possible to configure the affinity of the block device writeback flusher threads. Since block I/O can have a serious negative impact on real-time performance, it should be moved to the general purpose partition. Disable NUMA affinity for the writeback threads
echo 0 > /sys/bus/workqueue/devices/writeback/numa

# Set the affinity to only include the general purpose CPUs (CPU0 and CPU1).
echo 3 > /sys/bus/workqueue/devices/writeback/cpumask

## Real-time throttling in partitioned system

# Real-time throttling (RTT) is a kernel feature that limits the amount of CPU time given to Linux tasks with real-time priority. If any process that executes on an isolated CPU runs with real-time priority, the CPU will get interrupts with the interval specified in /proc/sys/kernel/sched_rt_period_us. If the system is configured with CONFIG_NO_HZ_FULL and a real-time process executes on a CONFIG_NO_HZ_FULL CPU, note that real-time throttling will cause the kernel to schedule extra ticks. See Section 2.3, Real-Time Throttling and Section 3.2.4, Optimize Real-Time Throttling for more information.

# Disable real-time throttling by the following command:
echo -1 > /proc/sys/kernel/sched_rt_runtime_us

## Machine check - x86 only

# The x86 architecture has a periodic check for corrected machine check errors (MCE). The periodic machine check requires a timer that causes unwanted jitter. The periodic check can be disabled. Note that this might lead to that silently corrected MCEs goes unlogged. Turn it off on the RT CPUs. For each CPU in the real-time partition, do the following:
echo 0 > /sys/devices/system/machinecheck/machinecheck2/check_interval
echo 0 > /sys/devices/system/machinecheck/machinecheck3/check_interval
# It has been observed that it is enough to disable this for CPU0 only; it will then be disabled on all CPUs.

## Disabling the NMI Watchdog - x86 only

# Disable the debugging feature for catching hardware hangings and cause a kernel panic. On some systems it can generate a lot of interrupts, causing a noticeable increase in power usage:

echo 0 > /proc/sys/kernel/nmi_watchdog


## Increase flush time to disk

# To make write-backs of dirty memory pages occur less often than the default, you can do the following:

echo 1500 > /proc/sys/vm/dirty_writeback_centisecs

## Disable tick maximum deferment

# To have the full tickless configuration, this patch should be included. This allows the tick interval to be maximized by setting sched_tick_max_deferment variable in the /proc filesystem. To disable the maximum deferment, it should be set to -1.

echo -1 > /sys/kernel/debug/sched_tick_max_deferment

## Disable Memory Overcommit

echo 2 > /proc/sys/vm/overcommit_memory

## Change the real-time priority of: EtherCAT IRQs, ksoftirqd thread for CPU3 and maybe Ethercat-IDLE.

enp5s0_pid=$(ps -e | grep "enp5s0$" | grep -o -E '[0-9]+' | head -n 1)
sudo chrt -f -p 82 $enp5s0_pid
enp5s0_r_pid=$(ps -e | grep "enp5s0-r" | grep -o -E '[0-9]+' | head -n 1)
sudo chrt -f -p 82 $enp5s0_r_pid
enp5s0_t_pid=$(ps -e | grep "enp5s0-t" | grep -o -E '[0-9]+' | head -n 1)
sudo chrt -f -p 82 $enp5s0_t_pid
ksoftirqd_pid=$(ps -e | grep "ksoftirqd/3" | grep -o -E '[0-9]+' | head -n 1)
sudo chrt -f -p 60 $ksoftirqd_pid

# Finally kill the irq_balance process of Linux
pkill -9 irqbalance