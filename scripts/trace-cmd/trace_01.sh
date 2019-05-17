#!/bin/bash
enp5s0_irq_pid=$(ps -e | grep "enp5s0$" | grep -o -E '[0-9]+' | head -n 1);
enp5s0_r_irq_pid=$(ps -e | grep "enp5s0-r" | grep -o -E '[0-9]+' | head -n 1);
enp5s0_t_irq_pid=$(ps -e | grep "enp5s0-t" | grep -o -E '[0-9]+' | head -n 1);
ksoftirqd3_pid=$(ps -e | grep "ksoftirqd/3" | grep -o -E '[0-9]+' | head -n 1);
ether_ros_pid=$(ps -e | grep "ether_ros" | grep -o -E '[0-9]+' | head -n 1);
sudo trace-cmd record -p function_graph -l ecrt_master_send -l ec_gen_device_start_xmit  -l kernel_sendmsg  -l e1000_xmit_frame -l ecrt_master_receive -l ec_gen_device_poll -l kernel_recvmsg -l __skb_recv_datagram -l e1000_intr_msix_rx -l __napi_schedule -l net_rx_action -l e1000e_poll -l e1000_clean_rx_irq -l napi_gro_receive -l dev_gro_receive -l netif_receive_skb_internal -l __netif_receive_skb_core -l packet_rcv -P $enp5s0_irq_pid -P $ether_ros_pid
