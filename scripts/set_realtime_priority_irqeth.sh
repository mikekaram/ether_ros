#!/bin/bash
irq_pid=$(ps -e | grep "enp5s0" | grep -o -E '[0-9]+' | head -n 1)
irq_num=$(ps -e | grep "enp5s0" | grep -o -E '[0-9]+' | head -n 5 | tail -1)
sudo chrt -f -p 81 $irq_pid
# sudo chrt -T 5000 -P 50000 -D 50000 -p 0 $irq_pid
echo 2 > /proc/irq/$irq_num/smp_affinity
