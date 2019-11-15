#!/bin/bash
enp5s0_irq_pid=$(ps -e | grep "enp5s0$" | grep -o -E '[0-9]+' | head -n 1);
enp5s0_r_irq_pid=$(ps -e | grep "enp5s0-r" | grep -o -E '[0-9]+' | head -n 1);
enp5s0_t_irq_pid=$(ps -e | grep "enp5s0-t" | grep -o -E '[0-9]+' | head -n 1);
ksoftirqd3_pid=$(ps -e | grep "ksoftirqd/3" | grep -o -E '[0-9]+' | head -n 1);
ether_ros_pid=$(ps -e | grep "ether_ros" | grep -o -E '[0-9]+' | head -n 1);
ether_ros_pids=$(for i in `ps -e -T | grep ether_ros | awk '{print  $2}')`; do echo "-P $i";done)
sudo trace-cmd record -p function_graph -e sched:sched_wakeup -l ecrt_master_send -l kernel_sendmsg -l e1000_xmit_frame -l e1000_intr_msix_rx -l net_rx_action -l napi_gro_receive -l __netif_receive_skb_core -l packet_rcv -l ecrt_master_receive  -l kernel_recvmsg  -P $enp5s0_r_irq_pid -P ether_ros_pids -N 192.168.0.103:12346
