#!/bin/bash
enp5s0_irq_pid=$(ps -e | grep "enp0s25$" | grep -o -E '[0-9]+' | head -n 1);
# enp5s0_r_irq_pid=$(ps -e | grep "enp5s0-r" | grep -o -E '[0-9]+' | head -n 1);
# enp5s0_t_irq_pid=$(ps -e | grep "enp5s0-t" | grep -o -E '[0-9]+' | head -n 1);
etherlab_pid=$(ps -e | grep "EtherCAT-IDLE" | grep -o -E '[0-9]+' | head -n 1);
ksoftirqd3_pid=$(ps -e | grep "ksoftirqd/1" | grep -o -E '[0-9]+' | head -n 1);
ether_ros_pid=$(ps -e | grep "ether_ros" | grep -o -E '[0-9]+' | head -n 1);
sudo trace-cmd record -p function_graph -e sched:sched_switch -e sched:sched_wakeup -l ecrt_master_send -l ec_master_send_datagrams -l ec_device_send -l ec_gen_netdev_start_xmit -l ec_gen_device_start_xmit  -l kernel_sendmsg -l sock_sendmsg -l packet_sendmsg -l dev_queue_xmit -l netdev_pick_tx -l sch_direct_xmit -l dev_hard_start_xmit -l e1000_xmit_frame -l ecrt_master_receive -l ec_device_poll -l ec_gen_poll -l ec_gen_device_poll -l kernel_recvmsg -l sock_recvmsg -l __skb_recv_datagram -l e1000_intr_msi -l __napi_schedule -l __raise_softirq_irqoff -l net_rx_action -l e1000e_poll -l e1000_clean_rx_irq -l napi_gro_receive -l dev_gro_receive -l __netif_receive_skb_core -l sock_queue_rcv_skb -l __sock_queue_rcv_skb -l skb_queue_tail -l schedule -P $enp5s0_irq_pid -P $etherlab_pid -P $ksoftirqd3_pid
