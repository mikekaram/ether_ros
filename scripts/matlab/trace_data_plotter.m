clc
close all;
clear;

filename = '~/catkin_ws/src/ether_ros/experiments/17May2019/output_08.csv';
results_directory='~/catkin_ws/src/ether_ros/experiments/17May2019/';
A = readtable(filename);

%% Network Latency Histogram (t_NET in Zurawski)

time = A.Time;
function_array = A.Function;
duration_array = A.Duration;
condition_array = strcmp(function_array,'e1000_xmit_frame()') | strcmp(function_array,'e1000_intr_msix_rx()');
time = time(condition_array);
function_array = function_array(condition_array);
duration_array = duration_array(condition_array);
l = length(function_array);
for i=2:l
    if(strcmp(function_array(i),function_array(i-1)))
        function_array(i) = {'aa'};
        time(i) = 0;
        duration_array(i) = 0;
    end
end
time = time(time > 0);
duration_array = duration_array(duration_array > 0);
function_array = function_array(~strcmp(function_array,'aa'));

xmit_frame_first=find(strcmp(function_array,'e1000_xmit_frame()'),1);
intr_rx_last=find(strcmp(function_array,'e1000_intr_msix_rx()'),1,'last');
time = time(xmit_frame_first:intr_rx_last);
function_array = function_array(xmit_frame_first:intr_rx_last);
duration_array = duration_array(xmit_frame_first:intr_rx_last);

xmit_frame_indeces = strcmp(function_array,'e1000_xmit_frame()');
time(xmit_frame_indeces) = time(xmit_frame_indeces) + duration_array(xmit_frame_indeces);

j=1;
% netlat_array = zeros(ceil(length(time)/2),1);
for i=1:2:length(time)-1
    netlat_array(j) = time(i+1) - time(i);
    j= j + 1;
end

nbins = 15;

figure()
histogram(netlat_array,nbins)
xlabel('Network Latency (us)')
ylabel('samples')
title('Network Latency (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');

%% Execution Time Plot (t_ALG in Zurawski)

time = A.Time;
duration_array = A.Duration;
function_array = A.Function;
condition_array = strcmp(function_array,'ecrt_master_send()') | strcmp(function_array,'ecrt_master_receive()');
time = time(condition_array);
function_array = function_array(condition_array);
duration_array = duration_array(condition_array);
master_receive_first=find(strcmp(function_array,'ecrt_master_receive()'),1);
master_send_last=find(strcmp(function_array,'ecrt_master_send()'),1,'last');
time = time(master_receive_first:master_send_last);
function_array = function_array(master_receive_first:master_send_last);
duration_array = duration_array(master_receive_first:master_send_last);

receive_indeces = strcmp(function_array,'ecrt_master_receive()');
time(receive_indeces) = time(receive_indeces) + duration_array(receive_indeces);
j=1;
% execution_array = zeros(ceil(length(time)/2),1);
for i=1:2:length(time)-1
    execution_array(j)=time(i+1) - time(i);
    j= j + 1;
end

nbins = 15;

figure()
histogram(execution_array,nbins)
xlabel('Execution Time (us)')
ylabel('samples')
title('Execution Time (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');

%% Period Time Plot (t_CYCLE in Zurawski)

time = A.Time;
duration_array = A.Duration;
function_array = A.Function;
condition_array = strcmp(function_array,'ecrt_master_send()');
time = time(condition_array);

period_array = diff(time);

nbins = 15;

figure()
histogram(period_array,nbins)
xlabel('Period Time (us)')
ylabel('samples')
title('Period Time (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');

%% Idle Time Plot (t_IDLE in Zurawski)

time = A.Time;
duration_array = A.Duration;
function_array = A.Function;
condition_array = strcmp(function_array,'packet_rcv()') | strcmp(function_array,'ecrt_master_receive()');
time = time(condition_array);
function_array = function_array(condition_array);
duration_array = duration_array(condition_array);
packet_rcv_first=find(strcmp(function_array,'packet_rcv()'),1);
master_receive_last=find(strcmp(function_array,'ecrt_master_receive()'),1,'last');
time = time(packet_rcv_first:master_receive_last);
function_array = function_array(packet_rcv_first:master_receive_last);
duration_array = duration_array(packet_rcv_first:master_receive_last);

packet_rcv_indeces = strcmp(function_array,'packet_rcv()');
time(packet_rcv_indeces) = time(packet_rcv_indeces) + duration_array(packet_rcv_indeces);

j=1;
for i=1:2:length(time)-1
    idle_array(j)=time(i+1) - time(i);
    j= j + 1;
end

nbins = 15;

figure()
histogram(idle_array,nbins)
xlabel('Idle Time (us)')
ylabel('samples')
title('Idle Time (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');

%% CPU Time Plot (t_CPU in Zurawski)

time = A.Time;
duration_array = A.Duration;
function_array = A.Function;
condition_array = strcmp(function_array,'packet_rcv()') | strcmp(function_array,'ecrt_master_receive()');
time = time(condition_array);
function_array = function_array(condition_array);
duration_array = duration_array(condition_array);
packet_rcv_first=find(strcmp(function_array,'packet_rcv()'),1);
master_receive_last=find(strcmp(function_array,'ecrt_master_receive()'),1,'last');
time = time(packet_rcv_first:master_receive_last);
function_array = function_array(packet_rcv_first:master_receive_last);
duration_array = duration_array(packet_rcv_first:master_receive_last);

packet_rcv_indeces = strcmp(function_array,'packet_rcv()');
time(packet_rcv_indeces) = time(packet_rcv_indeces) + duration_array(packet_rcv_indeces);

j=1;
for i=1:2:length(time)-1
    idle_array(j)=time(i+1) - time(i);
    j= j + 1;
end

nbins = 15;

figure()
histogram(idle_array,nbins)
xlabel('Idle Time (us)')
ylabel('samples')
title('Idle Time (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');

%% Linux Kernel Receive Latency Plot (part of t_EM in Zurawski) (Interrupt + SoftIRQ Context)

time = A.Time;
duration_array = A.Duration;
function_array = A.Function;
condition_array = strcmp(function_array,'packet_rcv()') | strcmp(function_array,'ecrt_master_receive()');
time = time(condition_array);
function_array = function_array(condition_array);
duration_array = duration_array(condition_array);
packet_rcv_first=find(strcmp(function_array,'packet_rcv()'),1);
master_receive_last=find(strcmp(function_array,'ecrt_master_receive()'),1,'last');
time = time(packet_rcv_first:master_receive_last);
function_array = function_array(packet_rcv_first:master_receive_last);
duration_array = duration_array(packet_rcv_first:master_receive_last);

packet_rcv_indeces = strcmp(function_array,'packet_rcv()');
time(packet_rcv_indeces) = time(packet_rcv_indeces) + duration_array(packet_rcv_indeces);

j=1;
for i=1:2:length(time)-1
    idle_array(j)=time(i+1) - time(i);
    j= j + 1;
end

nbins = 15;

figure()
histogram(idle_array,nbins)
xlabel('Idle Time (us)')
ylabel('samples')
title('Idle Time (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');

%% Linux Kernel Receive Latency Plot (part of t_EM in Zurawski) (Userspace Context)

time = A.Time;
duration_array = A.Duration;
function_array = A.Function;
condition_array = strcmp(function_array,'packet_rcv()') | strcmp(function_array,'ecrt_master_receive()');
time = time(condition_array);
function_array = function_array(condition_array);
duration_array = duration_array(condition_array);
packet_rcv_first=find(strcmp(function_array,'packet_rcv()'),1);
master_receive_last=find(strcmp(function_array,'ecrt_master_receive()'),1,'last');
time = time(packet_rcv_first:master_receive_last);
function_array = function_array(packet_rcv_first:master_receive_last);
duration_array = duration_array(packet_rcv_first:master_receive_last);

packet_rcv_indeces = strcmp(function_array,'packet_rcv()');
time(packet_rcv_indeces) = time(packet_rcv_indeces) + duration_array(packet_rcv_indeces);

j=1;
for i=1:2:length(time)-1
    idle_array(j)=time(i+1) - time(i);
    j= j + 1;
end

nbins = 15;

figure()
histogram(idle_array,nbins)
xlabel('Idle Time (us)')
ylabel('samples')
title('Idle Time (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');

%% Linux Kernel Send Latency Plot (part of t_EM in Zurawski)

time = A.Time;
duration_array = A.Duration;
function_array = A.Function;
condition_array = strcmp(function_array,'packet_rcv()') | strcmp(function_array,'ecrt_master_receive()');
time = time(condition_array);
function_array = function_array(condition_array);
duration_array = duration_array(condition_array);
packet_rcv_first=find(strcmp(function_array,'packet_rcv()'),1);
master_receive_last=find(strcmp(function_array,'ecrt_master_receive()'),1,'last');
time = time(packet_rcv_first:master_receive_last);
function_array = function_array(packet_rcv_first:master_receive_last);
duration_array = duration_array(packet_rcv_first:master_receive_last);

packet_rcv_indeces = strcmp(function_array,'packet_rcv()');
time(packet_rcv_indeces) = time(packet_rcv_indeces) + duration_array(packet_rcv_indeces);

j=1;
for i=1:2:length(time)-1
    idle_array(j)=time(i+1) - time(i);
    j= j + 1;
end

nbins = 15;

figure()
histogram(idle_array,nbins)
xlabel('Idle Time (us)')
ylabel('samples')
title('Idle Time (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');

%% Application Latency Plot (scheduling latency: useful for checking also different schedulers)

time = A.Time;
duration_array = A.Duration;
function_array = A.Function;
condition_array = strcmp(function_array,'packet_rcv()') | strcmp(function_array,'ecrt_master_receive()');
time = time(condition_array);
function_array = function_array(condition_array);
duration_array = duration_array(condition_array);
packet_rcv_first=find(strcmp(function_array,'packet_rcv()'),1);
master_receive_last=find(strcmp(function_array,'ecrt_master_receive()'),1,'last');
time = time(packet_rcv_first:master_receive_last);
function_array = function_array(packet_rcv_first:master_receive_last);
duration_array = duration_array(packet_rcv_first:master_receive_last);

packet_rcv_indeces = strcmp(function_array,'packet_rcv()');
time(packet_rcv_indeces) = time(packet_rcv_indeces) + duration_array(packet_rcv_indeces);

j=1;
for i=1:2:length(time)-1
    idle_array(j)=time(i+1) - time(i);
    j= j + 1;
end

nbins = 15;

figure()
histogram(idle_array,nbins)
xlabel('Idle Time (us)')
ylabel('samples')
title('Idle Time (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');

%% EM Latency Plot (Receive -> Send Time, Includes also the t_ALG or Execution Time)

time = A.Time;
duration_array = A.Duration;
function_array = A.Function;
condition_array = strcmp(function_array,'packet_rcv()') | strcmp(function_array,'ecrt_master_receive()');
time = time(condition_array);
function_array = function_array(condition_array);
duration_array = duration_array(condition_array);
packet_rcv_first=find(strcmp(function_array,'packet_rcv()'),1);
master_receive_last=find(strcmp(function_array,'ecrt_master_receive()'),1,'last');
time = time(packet_rcv_first:master_receive_last);
function_array = function_array(packet_rcv_first:master_receive_last);
duration_array = duration_array(packet_rcv_first:master_receive_last);

packet_rcv_indeces = strcmp(function_array,'packet_rcv()');
time(packet_rcv_indeces) = time(packet_rcv_indeces) + duration_array(packet_rcv_indeces);

j=1;
for i=1:2:length(time)-1
    idle_array(j)=time(i+1) - time(i);
    j= j + 1;
end

nbins = 15;

figure()
histogram(idle_array,nbins)
xlabel('Idle Time (us)')
ylabel('samples')
title('Idle Time (us)')

% filename = sprintf('./figures/xen_hard_soft/period_all_3500_%d_s_hist',duration);
% print(filename,'-djpeg');
