#!/bin/bash
irq_pid=$(ps -e | grep "enp0s25" | grep -o -E '[0-9]+' | head -n 1)
sudo chrt -f -p 81 $irq_pid
