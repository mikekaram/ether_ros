clc
close all;
% clear all

filename = '~/catkin_ws/src/ighm_ros/experiments/20Dec2018/ethercat_data_ground_1.orig-pdo_in_slaves.csv';
A = readtable(filename);
Hip_PWM_Limit = 41.17;
Knee_PWM_Limit = 38.25;

Hip_max_velocity = 75.83*2*pi/60; %rad/s in 60 V
Knee_max_velocity = 55.5*2*pi/60; %rad/s in 60 V

i_knee=(8*26)/(343*48);
i_hip=(12*26)/(637*48);

indeces2names = {'HR','HL','FL','FR'};
A.Time = A.x_pdo_in_slave_0_time;
% A.Properties.VariableNames{'x_pdo_in_slave_0_time'} = 'Time';
for i=1:4
     A.Properties.VariableNames{['x_pdo_in_slave_' num2str(i-1) '_desired_hip_angle']} = [indeces2names{i} '_desired_hip_angle'];
     A.Properties.VariableNames{['x_pdo_in_slave_' num2str(i-1) '_desired_knee_angle']} = [indeces2names{i} '_desired_knee_angle'];
     A.Properties.VariableNames{['x_pdo_in_slave_' num2str(i-1) '_hip_angle']} = [indeces2names{i} '_hip_angle'];
     A.Properties.VariableNames{['x_pdo_in_slave_' num2str(i-1) '_knee_angle']} = [indeces2names{i} '_knee_angle'];
     A.Properties.VariableNames{['x_pdo_in_slave_' num2str(i-1) '_PWM10000_knee']} = [indeces2names{i} '_PWM10000_knee'];
     A.Properties.VariableNames{['x_pdo_in_slave_' num2str(i-1) '_PWM10000_hip']} = [indeces2names{i} '_PWM10000_hip'];
     A.Properties.VariableNames{['x_pdo_in_slave_' num2str(i-1) '_velocity_knee1000']} = [indeces2names{i} '_velocity_knee1000'];
     A.Properties.VariableNames{['x_pdo_in_slave_' num2str(i-1) '_velocity_hip1000']} = [indeces2names{i} '_velocity_hip1000'];
     A.Properties.VariableNames{['x_pdo_in_slave_' num2str(i-1) '_time']} = [indeces2names{i} '_time'];
end
t = A.Time;

HR_hip_angle = -A.HR_hip_angle / 100;
HR_knee_angle = -A.HR_knee_angle / 100;
HR_desired_hip_angle = -A.HR_desired_hip_angle / 100;
HR_desired_knee_angle = -A.HR_desired_knee_angle / 100;
HR_uk_hip = A.HR_PWM10000_hip / 100;
HR_uk_knee = A.HR_PWM10000_knee / 100;
HR_velocity_hip = -A.HR_velocity_hip1000 * i_hip / 1000;
HR_velocity_knee = -A.HR_velocity_knee1000 * i_knee / 1000;
HR_time = A.HR_time;

HL_hip_angle = A.HL_hip_angle / 100;
HL_knee_angle = A.HL_knee_angle / 100;
HL_desired_hip_angle = A.HL_desired_hip_angle / 100;
HL_desired_knee_angle = A.HL_desired_knee_angle / 100;
HL_uk_hip = A.HL_PWM10000_hip / 100;
HL_uk_knee = A.HL_PWM10000_knee / 100;
HL_velocity_hip = A.HL_velocity_hip1000 * i_hip / 1000;
HL_velocity_knee = A.HL_velocity_knee1000 * i_knee / 1000;
HL_time = A.HL_time;

FR_hip_angle = - A.FR_hip_angle / 100;
FR_knee_angle = - A.FR_knee_angle / 100;
FR_desired_hip_angle = - A.FR_desired_hip_angle / 100;
FR_desired_knee_angle = - A.FR_desired_knee_angle / 100;
FR_uk_hip = A.FR_PWM10000_hip / 100;
FR_uk_knee = A.FR_PWM10000_knee / 100;
FR_velocity_hip = - A.FR_velocity_hip1000 * i_hip / 1000;
FR_velocity_knee = - A.FR_velocity_knee1000 * i_knee / 1000;
FR_time = A.FR_time;

FL_hip_angle = A.FL_hip_angle / 100;
FL_knee_angle = A.FL_knee_angle / 100;
FL_desired_hip_angle = A.FL_desired_hip_angle / 100;
FL_desired_knee_angle = A.FL_desired_knee_angle / 100;
FL_uk_hip = A.FL_PWM10000_hip / 100;
FL_uk_knee = A.FL_PWM10000_knee / 100;
FL_velocity_hip = A.FL_velocity_hip1000 * i_hip / 1000;
FL_velocity_knee = A.FL_velocity_knee1000 * i_knee / 1000;
FL_time = A.FL_time;

window=60000:65000;
[HR_x, HR_y]=ForwardKinematics(HR_hip_angle(window),HR_knee_angle(window));
[HL_x, HL_y]=ForwardKinematics(HL_hip_angle(window),HL_knee_angle(window));
[FR_x, FR_y]=ForwardKinematics(FR_hip_angle(window),FR_knee_angle(window));
[FL_x, FL_y]=ForwardKinematics(FL_hip_angle(window),FL_knee_angle(window));
[HR_x_desired, HR_y_desired]=ForwardKinematics(HR_desired_hip_angle(window),HR_desired_knee_angle(window));
[HL_x_desired, HL_y_desired]=ForwardKinematics(HL_desired_hip_angle(window),HL_desired_knee_angle(window));
[FR_x_desired, FR_y_desired]=ForwardKinematics(FR_desired_hip_angle(window),FR_desired_knee_angle(window));
[FL_x_desired, FL_y_desired]=ForwardKinematics(FL_desired_hip_angle(window),FL_desired_knee_angle(window));

%End Effector of Laelaps II Legs
figure
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(2,2,1)
plot(FR_x,FR_y,'k',FR_x_desired,FR_y_desired,'r')
     grid on
     ylabel('+ <-- y axis','fontsize',14)
     xlabel('x axis --> +','fontsize',14)
     title('FR End Effector in Steady State','fontsize',14)
     set(gca,'Ydir','reverse')
subplot(2,2,2)
plot(FL_x,FL_y,'k',FL_x_desired,FL_y_desired,'r')
     grid on
     ylabel('+ <-- y axis','fontsize',14)
     xlabel('x axis --> +','fontsize',14)
     title('FL End Effector in Steady State','fontsize',14)
     set(gca,'Ydir','reverse')
subplot(2,2,3)
plot(HR_x,HR_y,'k',HR_x_desired,HR_y_desired,'r')
     grid on
     ylabel('+ <-- y axis','fontsize',14)
     xlabel('x axis --> +','fontsize',14)
     title('HR End Effector in Steady State','fontsize',14)
     set(gca,'Ydir','reverse')
subplot(2,2,4)
plot(HL_x,HL_y,'k',HL_x_desired,HL_y_desired,'r')
     grid on
     ylabel('+ <-- y axis','fontsize',14)
     xlabel('x axis --> +','fontsize',14)
     title('HL End Effector in Steady State','fontsize',14)
     set(gca,'Ydir','reverse')
tightfig;

%Responce of knee angles
figure
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
plot(t,FR_knee_angle,'k',t,FR_desired_knee_angle,'r')
     grid on
     ylabel('Angle [deg]')
   	 title('Response of FR Knee Angle','fontsize',13)
subplot(4,1,2)
plot(t,FL_knee_angle,'k',t,FL_desired_knee_angle,'r')
     grid on
     ylabel('Angle [deg]')
   	 title('Response of FL Knee Angle','fontsize',13)
subplot(4,1,3)
plot(t,HR_knee_angle,'k',t,HR_desired_knee_angle,'r')
     grid on
     ylabel('Angle [deg]')
   	 title('Response of HR Knee Angle','fontsize',13)
subplot(4,1,4)
plot(t,HL_knee_angle,'k',t,HL_desired_knee_angle,'r')
     grid on
     ylabel('Angle [deg]')
     xlabel('Time [s]')
   	 title('Response of HL Knee Angle','fontsize',13)
tightfig;

%Responce of hip angles
figure
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
plot(t,FR_hip_angle ,'k', t, FR_desired_hip_angle,'r')
     grid on
     ylabel('Angle [deg]')
   	 title('Response of FR Hip Angle','fontsize',13)
subplot(4,1,2)
plot(t,FL_hip_angle,'k', t, FL_desired_hip_angle,'r')
     grid on
     ylabel('Angle [deg]')
   	 title('Response of FL Hip Angle','fontsize',13)
subplot(4,1,3)
plot(t,HR_hip_angle,'k', t, HR_desired_hip_angle,'r')
     grid on
     ylabel('Angle [deg]')
   	 title('Response of HR Hip Angle','fontsize',13)
subplot(4,1,4)
plot(t,HL_hip_angle,'k', t, HL_desired_hip_angle,'r')
     grid on
     ylabel('Angle [deg]')
     xlabel('Time [s]')
   	 title('Response of HL Hip Angle','fontsize',13)
tightfig;

%PWM Commands of Knee motors
figure
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
plot(t,FR_uk_knee,'k','LineWidth',0.1)
hold on
plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
     grid on
     ylabel('PWM Command [%]')
   	 title('PWM Command of FR Knee','fontsize',13)
subplot(4,1,2)
plot(t,FL_uk_knee,'k','LineWidth',0.1)
hold on
plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
     grid on
     ylabel('PWM Command [%]')
   	 title('PWM Command of FL Knee','fontsize',13)
subplot(4,1,3)
plot(t,HR_uk_knee,'k','LineWidth',0.1)
hold on
plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
     grid on
     ylabel('PWM Command [%]')
   	 title('PWM Command of HR Knee','fontsize',13)
subplot(4,1,4)
plot(t,HL_uk_knee,'k','LineWidth',0.1)
hold on
plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
     grid on
     ylabel('PWM Command [%]')
     xlabel('Time [s]')
   	 title('PWM Command of HL Knee','fontsize',13)
tightfig;

%PWM Commands of Hip motors
figure
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
plot(t,FR_uk_hip,'k','LineWidth',0.1)
hold on
plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
     grid on
     ylabel('PWM Command [%]')
   	 title('PWM Command of FR Hip','fontsize',13)
subplot(4,1,2)
plot(t,FL_uk_hip,'k','LineWidth',0.1)
hold on
plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
     grid on
     ylabel('PWM Command [%]')
   	 title('PWM Command of FL Hip','fontsize',13)
subplot(4,1,3)
plot(t,HR_uk_hip,'k','LineWidth',0.1)
hold on
plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
     grid on
     ylabel('PWM Command [%]')
   	 title('PWM Command of HR Hip','fontsize',13)
subplot(4,1,4)
plot(t,HL_uk_hip,'k','LineWidth',0.1)
hold on
plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
     grid on
     ylabel('PWM Command [%]')
     xlabel('Time [s]')
   	 title('PWM Command of HL Hip','fontsize',13)
tightfig;

%Velocity of Knee motors
figure
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
hold on
plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,FR_velocity_knee,'k')
     grid on
     ylabel('Velocity [rad/s]')
   	 title('Response of FR Knee Velocity','fontsize',13)
subplot(4,1,2)
hold on
plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,FL_velocity_knee,'k')
     grid on
     ylabel('Velocity [rad/s]')
   	 title('Response of FL Knee Velocity','fontsize',13)
subplot(4,1,3)
hold on
plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,HR_velocity_knee,'k')
     grid on
     ylabel('Velocity [rad/s]')
   	 title('Response of HR Knee Velocity','fontsize',13)
subplot(4,1,4)
hold on
plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,HL_velocity_knee,'k')
     grid on
     ylabel('Velocity [rad/s]')
     xlabel('Time [s]')
   	 title('Response of HL Knee Velocity','fontsize',13)
tightfig;

%Velocity of Hip motors
figure
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
hold on
plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,FR_velocity_hip,'k')
     grid on
     ylabel('Velocity [rad/s]')
   	 title('Response of FR Hip Velocity','fontsize',13)
subplot(4,1,2)
hold on
plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,FL_velocity_hip,'k')
     grid on
     ylabel('Velocity [rad/s]')
   	 title('Response of FL Hip Velocity','fontsize',13)
subplot(4,1,3)
hold on
plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,HR_velocity_hip,'k')
     grid on
     ylabel('Velocity [rad/s]')
   	 title('Response of HR Hip Velocity','fontsize',13)
subplot(4,1,4)
hold on
plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
plot(t,HL_velocity_hip,'k')
     grid on
     ylabel('Velocity [rad/s]')
     xlabel('Time [s]')
   	 title('Response of HL Hip Velocity','fontsize',13)
tightfig;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Commented section by Stamatis%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Plot Hind Right Leg's Diagrams
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% plot(t,HR_knee_angle_deg,'k',t,HR_Desired_knee_angle,'r')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Angle [deg]')
%      xlabel('Time [s]')
%    	 title('Response of HR Knee Angle','fontsize',13)
% subplot(2,1,2)
% plot(t,HR_hip_angle_deg,'k',t,HR_Desired_hip_angle,'r')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Angle [deg]')
%      xlabel('Time [s]')
%    	 title('Response of HR Hip Angle','fontsize',13)
% tightfig;
%
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% plot(t,HR_uk_knee,'k','LineWidth',0.1)
% hold on
% plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
%      grid on
%      ylabel('PWM Command [%]')
%      xlabel('Time [s]')
%    	 title('PWM Command of HR Knee','fontsize',13)
% subplot(2,1,2)
% plot(t,HR_uk_hip,'k','LineWidth',0.1)
% hold on
% plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
%      grid on
% %     axis([2 7 -50 60])
%      ylabel('PWM Command [%]')
%      xlabel('Time [s]')
%    	 title('PWM command of HR Hip','fontsize',13)
% tightfig;
%
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% hold on
% plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,HR_velocity_knee,'k')
%      grid on
%      ylabel('Velocity [rad/s]')
%      xlabel('Time [s]')
%    	 title('Response of HR Knee Velocity','fontsize',13)
% subplot(2,1,2)
% hold on
% plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,HR_velocity_hip,'k')
%      grid on
%      ylabel('Velocity [rad/s]')
%      xlabel('Time [s]')
%    	 title('Response of HR Hip Velocity','fontsize',13)
% tightfig;
%
% % Plot Hind Left Leg's Diagrams
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% plot(t,HL_knee_angle_deg,'k',t,HL_Desired_knee_angle,'r')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Angle [deg]')
%      xlabel('Time [s]')
%    	 title('Response of HL Knee Angle','fontsize',13)
% subplot(2,1,2)
% plot(t,HL_hip_angle_deg,'k',t,HL_Desired_hip_angle,'r')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Angle [deg]')
%      xlabel('Time [s]')
%    	 title('Response of HL Hip Angle','fontsize',13)
% tightfig;
%
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% plot(t,HL_uk_knee,'k','LineWidth',0.1)
% hold on
% plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
%      grid on
% %     axis([2 7 -50 60])
%      ylabel('PWM Command [%]')
%      xlabel('Time [s]')
%    	 title('PWM Command of HL Knee','fontsize',13)
% subplot(2,1,2)
% plot(t,HL_uk_hip,'k','LineWidth',0.1)
% hold on
% plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
%      grid on
% %     axis([2 7 -50 60])
%      ylabel('PWM Command [%]')
%      xlabel('Time [s]')
%    	 title('PWM command of HL Hip','fontsize',13)
% tightfig;
%
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% hold on
% plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,HL_velocity_knee,'k')
%      grid on
%      ylabel('Velocity [rad/s]')
%      xlabel('Time [s]')
%    	 title('Response of HL Knee Velocity','fontsize',13)
% subplot(2,1,2)
% hold on
% plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,HL_velocity_hip,'k')
%      grid on
%      ylabel('Velocity [rad/s]')
%      xlabel('Time [s]')
%    	 title('Response of HL Hip Velocity','fontsize',13)
% tightfig;
%
% % Plot Fore Right Leg's Diagrams
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% hold all
% plot(t,FR_knee_angle_deg,'k',t,FR_Desired_knee_angle,'r')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Angle [deg]')
%      xlabel('Time [s]')
%    	 title('Response of FR Knee Angle','fontsize',13)
% subplot(2,1,2)
% plot(t,FR_hip_angle_deg,'k',t,FR_Desired_hip_angle,'r')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Angle [deg]')
%      xlabel('Time [s]')
%    	 title('Response of FR Hip Angle','fontsize',13)
% tightfig;
%
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% plot(t,FR_uk_knee,'k','LineWidth',0.1)
% hold on
% plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
%      grid on
% %     axis([2 7 -50 60])
%      ylabel('PWM Command [%]')
%      xlabel('Time [s]')
%    	 title('PWM Command of FR Knee','fontsize',13)
% subplot(2,1,2)
% plot(t,FR_uk_hip,'k','LineWidth',0.1)
% hold on
% plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
%      grid on
% %     axis([2 7 -50 60])
%      ylabel('PWM Command [%]')
%      xlabel('Time [s]')
%    	 title('PWM Command of FR Hip','fontsize',13)
%  tightfig;
%
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% hold on
% plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,FR_velocity_knee,'k')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Velocity [rad/s]')
%      xlabel('Time [s]')
%    	 title('Response of FR Knee Velocity','fontsize',13)
% subplot(2,1,2)
% hold on
% plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,FR_velocity_hip,'k')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Velocity [rad/s]')
%      xlabel('Time [s]')
%    	 title('Response of FR Hip Velocity','fontsize',13)
% tightfig;
%
% % Plot Fore Left Leg's Diagrams
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% hold all
% plot(t,FL_knee_angle_deg,'k',t,FL_Desired_knee_angle,'r')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Angle [deg]')
%      xlabel('Time [s]')
%    	 title('Response of FL Knee Angle','fontsize',13)
% subplot(2,1,2)
% plot(t,FL_hip_angle_deg,'k',t,FL_Desired_hip_angle,'r')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Angle [deg]')
%      xlabel('Time [s]')
%    	 title('Response of FL Hip Angle','fontsize',13)
% tightfig;
%
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% plot(t,FL_uk_knee,'k','LineWidth',0.1)
% hold on
% plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
%      grid on
% %     axis([2 7 -50 60])
%      ylabel('PWM Command [%]')
%      xlabel('Time [s]')
%    	 title('PWM Command of FL Knee','fontsize',13)
% subplot(2,1,2)
% plot(t,FL_uk_hip,'k','LineWidth',0.1)
% hold on
% plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1);
%      grid on
% %     axis([2 7 -50 60])
%      ylabel('PWM Command [%]')
%      xlabel('Time [s]')
%    	 title('PWM Command of FL Hip','fontsize',13)
%  tightfig;
%
% figure
% set(gcf, 'Position', [100 50 900 800],'color','w');
% subplot(2,1,1)
% hold on
% plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,FL_velocity_knee,'k')
%      grid on
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Velocity [rad/s]')
%      xlabel('Time [s]')
%    	 title('Response of FL Knee Velocity','fontsize',13)
% subplot(2,1,2)
% hold on
% plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
% plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1);
%      grid on
% plot(t,FL_velocity_hip,'k')
% %      axis([-0.7 0.7 -0.1 0.7])
%      ylabel('Velocity [rad/s]')
%      xlabel('Time [s]')
%    	 title('Response of FL Hip Velocity','fontsize',13)
% tightfig;
