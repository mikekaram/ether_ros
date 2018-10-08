#!/bin/sh
## tsc boot parameter - x86 only


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
echo 0 >/sys/devices/system/machinecheck/machinecheck2/check_interval
echo 0 >/sys/devices/system/machinecheck/machinecheck3/check_interval
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


## Network queues affinity

# Linux can route the packets on different CPUs in an SMP system. Also this handling can create timers on the specific CPUs, an example is the ARP timer management, based on neigh_timer. There are a couple of solutions that can be adopted to minimize the effect of rerouting packets on different CPUs, like migrating all the timers on the non-realtime partition if possible, specifying the affinity of network queues on some architectures.

# If the application needs the packets to be received only in the nRT partition then the affinity should be set as follows:

# echo <NRT cpus mask> > /sys/class/net/<ethernet interface>/queues/<queue>/<x/r>ps_cpus
echo 8 /sys/class/net/enp5s0/queues/rx-0/rps_cpus
echo 8 /sys/class/net/enp5s0/queues/tx-0/xps_cpus

echo 3 /sys/class/net/enp6s0/queues/rx-0/rps_cpus
echo 3 /sys/class/net/enp6s0/queues/tx-0/xps_cpus

##############################DONT FORGET TO KILL IRQ_BALANCE #######################################
pkill -9 irqbalance
