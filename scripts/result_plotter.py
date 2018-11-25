#!/usr/bin/python
# -*- coding: utf-8 -*
from __future__ import division
import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import csv
import sys
import math
# csv.field_size_limit(sys.maxsize)

# def open_csv(file_path):
#     fields = []
#     rows = []
#     with open(file_path, 'rb') as csvfile:
#         csvreader = csv.reader(
#             csvfile, quoting=csv.QUOTE_NONE, delimiter=',')
#         fields = csvreader.next()
#         fields[0] = fields[0].replace('#','')
#         # print fields
#         rows = [row for row in csvreader]
#     return rows, fields


if __name__=="__main__":
    # rows, fields = open_csv(sys.argv[1])
    df = pd.read_csv(sys.argv[1])
    # print df.iloc[:,0].tolist()
    print df
    Hip_PWM_Limit = 44.0
    Knee_PWM_Limit = 38.25

    Hip_max_velocity = 75.83*2*math.pi/60 #rad/s in 60 V
    Knee_max_velocity = 55.5*2*math.pi/60 #rad/s in 60 V

    i_knee=(8*26)/(343*48)
    i_hip=(12*26)/(637*48)

    # # Initializations
    # t=[]
    # # Hind Right Leg
    # HR_knee_angle_deg=zeros(1,length(A))
    # HR_hip_angle_deg=zeros(1,length(A))
    # HR_velocity_hip=zeros(1,length(A))
    # HR_velocity_knee=zeros(1,length(A))
    # HR_uk_hip=zeros(1,length(A))
    # HR_uk_knee=zeros(1,length(A))
    # HR_Desired_hip_angle=zeros(1,length(A))
    # HR_Desired_knee_angle=zeros(1,length(A))
    # HR_time=zeros(1,length(A))

    # # Hind Left Leg
    # HL_knee_angle_deg=zeros(1,length(A))
    # HL_hip_angle_deg=zeros(1,length(A))
    # HL_velocity_hip=zeros(1,length(A))
    # HL_velocity_knee=zeros(1,length(A))
    # HL_uk_hip=zeros(1,length(A))
    # HL_uk_knee=zeros(1,length(A))
    # HL_Desired_hip_angle=zeros(1,length(A))
    # HL_Desired_knee_angle=zeros(1,length(A))
    # HL_time=zeros(1,length(A))

    # # Fore Right Leg
    # FR_knee_angle_deg=zeros(1,length(A))
    # FR_hip_angle_deg=zeros(1,length(A))
    # FR_velocity_hip=zeros(1,length(A))
    # FR_velocity_knee=zeros(1,length(A))
    # FR_step_command_hip=zeros(1,length(A))
    # FR_step_command_knee=zeros(1,length(A))
    # FR_uk_hip=zeros(1,length(A))
    # FR_uk_knee=zeros(1,length(A))
    # FR_Desired_hip_angle=zeros(1,length(A))
    # FR_Desired_knee_angle=zeros(1,length(A))
    # FR_time=zeros(1,length(A))

    # # Fore Left Leg
    # FL_knee_angle_deg=zeros(1,length(A))
    # FL_hip_angle_deg=zeros(1,length(A))
    # FL_velocity_hip=zeros(1,length(A))
    # FL_velocity_knee=zeros(1,length(A))
    # FL_uk_hip=zeros(1,length(A))
    # FL_uk_knee=zeros(1,length(A))
    # FL_Desired_hip_angle=zeros(1,length(A))
    # FL_Desired_knee_angle=zeros(1,length(A))
    # FL_time=zeros(1,length(A))
    # j=1
    # time_ticks = df['/pdo_in_slave_0/time']
    df.rename(columns={
        '/pdo_in_slave_0/time':'Time',
        '/pdo_in_slave_0/desired_hip_angle':'Desired HR hip angle',
        '/pdo_in_slave_1/desired_hip_angle':'Desired HL hip angle',
        '/pdo_in_slave_2/desired_hip_angle':'Desired FL hip angle',
        '/pdo_in_slave_3/desired_hip_angle':'Desired FR hip angle'
        },inplace=True)
    df['Desired HR hip angle'] = -df['Desired HR hip angle']/100
    df['Desired HL hip angle'] = df['Desired HL hip angle']/100
    df['Desired FL hip angle'] = df['Desired FL hip angle']/100
    df['Desired FR hip angle'] = -df['Desired FR hip angle']/100
    print df
    # exit(0)
    # HR_Desired_hip_angle = -df['/pdo_in_slave_0/desired_hip_angle']/100
    # HR_Desired_hip_angle.rename(columns={'/pdo_in_slave_0/desired_hip_angle':'Desired HR hip angle'},inplace=True)
    # HL_Desired_hip_angle = df['/pdo_in_slave_1/desired_hip_angle']/100
    # HL_Desired_hip_angle.rename(columns={'/pdo_in_slave_1/desired_hip_angle':'Desired HL hip angle'},inplace=True)
    # FL_Desired_hip_angle = df['/pdo_in_slave_2/desired_hip_angle']/100
    # FL_Desired_hip_angle.rename(columns={'/pdo_in_slave_2/desired_hip_angle':'Desired FL hip angle'},inplace=True)
    # FR_Desired_hip_angle = -df['/pdo_in_slave_3/desired_hip_angle']/100
    # FR_Desired_hip_angle.rename(columns={'/pdo_in_slave_3/desired_hip_angle':'Desired FR hip angle'},inplace=True)
    # print(HR_Desired_hip_angle[:10], HL_Desired_hip_angle[:10], FL_Desired_hip_angle[:10], FR_Desired_hip_angle[:10])
    # print time_ticks
    # for row in rows:
    #     t.extend=A(1,i)/1000
    #     HR_hip_angle_deg(i)= -A(2,i)/100
    #     HR_knee_angle_deg(i)= -A(4,i)/100
    #     HR_Desired_hip_angle(i)=-A(6,i)/100
    #     HR_Desired_knee_angle(i)=-A(8,i)/100
    #     HR_uk_hip(i)=A(10,i)/100
    #     HR_uk_knee(i)=A(12,i)/100
    #     HR_velocity_hip(i)=-A(14,i)*i_hip/1000
    #     HR_velocity_knee(i)=-A(16,i)*i_knee/1000
    #     HR_time(i)=A(18,i)/100

    #     HL_hip_angle_deg(i)= A(20,i)/100
    #     HL_knee_angle_deg(i)= A(22,i)/100
    #     HL_Desired_hip_angle(i)=A(24,i)/100
    #     HL_Desired_knee_angle(i)=A(26,i)/100
    #     HL_uk_hip(i)=A(28,i)/100
    #     HL_uk_knee(i)=A(30,i)/100
    #     HL_velocity_hip(i)=A(32,i)*i_hip/1000
    #     HL_velocity_knee(i)=A(34,i)*i_knee/1000
    #     HL_time(i)=A(36,i)/100

    #     FR_hip_angle_deg(i)= -A(38,i)/100
    #     FR_knee_angle_deg(i)= -A(40,i)/100
    #     FR_Desired_hip_angle(i)=-A(42,i)/100
    #     FR_Desired_knee_angle(i)=-A(44,i)/100
    #     FR_uk_hip(i)=A(46,i)/100
    #     FR_uk_knee(i)=A(48,i)/100
    #     FR_velocity_hip(i)=-A(50,i)*i_hip/1000
    #     FR_velocity_knee(i)=-A(52,i)*i_knee/1000
    #     FR_time(i)=A(54,i)/100

    #     FL_hip_angle_deg(i)=A(56,i)/100
    #     FL_knee_angle_deg(i)=A(58,i)/100
    #     FL_Desired_hip_angle(i)=A(60,i)/100
    #     FL_Desired_knee_angle(i)=A(62,i)/100
    #     FL_uk_hip(i)=A(64,i)/100
    #     FL_uk_knee(i)=A(66,i)/100
    #     FL_velocity_hip(i)=A(68,i)*i_hip/1000
    #     FL_velocity_knee(i)=A(70,i)*i_knee/1000
    #     FL_time(i)=A(72,i)/100

    #     # if (i>30000 && i<35000)
    #     [HR_x(j), HR_y(j)]=ForwardKinematics(HR_hip_angle_deg(i),HR_knee_angle_deg(i))
    #     [HR_x_desired(j), HR_y_desired(j)]=ForwardKinematics(HR_Desired_hip_angle(i),HR_Desired_knee_angle(i))
    #     [HL_x(j), HL_y(j)]=ForwardKinematics(HL_hip_angle_deg(i),HL_knee_angle_deg(i))
    #     [HL_x_desired(j), HL_y_desired(j)]=ForwardKinematics(HL_Desired_hip_angle(i),HL_Desired_knee_angle(i))
    #     [FR_x(j), FR_y(j)]=ForwardKinematics(FR_hip_angle_deg(i),FR_knee_angle_deg(i))
    #     [FR_x_desired(j), FR_y_desired(j)]=ForwardKinematics(FR_Desired_hip_angle(i),FR_Desired_knee_angle(i))
    #     [FL_x(j), FL_y(j)]=ForwardKinematics(FL_hip_angle_deg(i),FL_knee_angle_deg(i))
    #     [FL_x_desired(j), FL_y_desired(j)]=ForwardKinematics(FL_Desired_hip_angle(i),FL_Desired_knee_angle(i))
    #     j=j+1
    #     end
    # end
    # Print the desired hip angles

    # Set context to `"paper"`, font_scale=1.5, rc={"font.size":5,"axes.labelsize":5}
    sns.set_context("notebook")
    sns.set_style("whitegrid")
    fig = plt.figure()

    ax1 = fig.add_subplot(4, 1, 1)
    dataframe = df[["Time", "Desired HR hip angle"]][:55000]
    ax1 = sns.lineplot(
        x="Time", y="Desired HR hip angle", palette="muted", data=dataframe, ax=ax1)
    ax1.set_xlabel('')
    ax1.set_ylabel('Angles (deg)')
    ax1.set_title('Response of HR Hip Angle', fontsize=13)
    ax1.set_ylim(0, 40)

    ax2 = fig.add_subplot(4, 1, 2)
    dataframe = df[["Time", "Desired HL hip angle"]][:55000]
    ax2 = sns.lineplot(
        x="Time", y="Desired HL hip angle", palette="muted", data=dataframe, ax=ax2)
    ax2.set_ylabel('Angles (deg)')
    ax2.set_xlabel('')
    ax2.set_title('Response of HL Hip Angle', fontsize=13)
    ax2.set_ylim(0, 40)

    ax3 = fig.add_subplot(4, 1, 3)
    dataframe = df[["Time", "Desired FL hip angle"]][:55000]
    ax3 = sns.lineplot(
        x="Time", y="Desired FL hip angle", palette="muted", data=dataframe, ax=ax3)
    ax3.set_ylabel('Angles (deg)')
    ax3.set_xlabel('')
    ax3.set_title('Response of FL Hip Angle', fontsize=13)
    ax3.set_ylim(0, 40)

    ax4 = fig.add_subplot(4, 1, 4)
    dataframe = df[["Time", "Desired FR hip angle"]][:55000]
    ax4 = sns.lineplot(
        x="Time", y="Desired FR hip angle", palette="muted", data=dataframe, ax=ax4)
    ax4.set_ylabel('Angles (deg)')
    ax4.set_xlabel('')
    ax4.set_title('Response of FR Hip Angle', fontsize=13)
    ax4.set_ylim(0, 40)

    plt.tight_layout()
    plt.xlabel('Time (ticks)')

    dataframe = df[["Time", "Desired HR hip angle",
                    "Desired HL hip angle", "Desired FL hip angle", "Desired FR hip angle"]][:55000]
    data = dataframe.melt('Time', var_name='Desired Hip Angles', value_name='Angles')
    hip_angle_plot = sns.relplot(x="Time", y="Angles", hue='Desired Hip Angles',kind="line", palette = "muted", data=data)
    plt.xticks(rotation=30)
    plt.title('Desired Hip Angles')
    plt.xlabel('Time (ticks)')
    plt.ylabel('Angles (deg)')
    plt.ylim(0,40)

    # plt.tight_layout()
    plt.show()
    # # End Effector of Laelaps II Legs
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,2,1)
    # plot(FR_x,FR_y,'k',FR_x_desired,FR_y_desired,'r')
    #     grid on
    #     ylabel('+ <-- y axis','fontsize',14)
    #     xlabel('x axis --> +','fontsize',14)
    #     title('FR End Effector in Steady State','fontsize',14)
    #     set(gca,'Ydir','reverse')
    # subplot(2,2,2)
    # plot(FL_x,FL_y,'k',FL_x_desired,FL_y_desired,'r')
    #     grid on
    #     ylabel('+ <-- y axis','fontsize',14)
    #     xlabel('x axis --> +','fontsize',14)
    #     title('FL End Effector in Steady State','fontsize',14)
    #     set(gca,'Ydir','reverse')
    # subplot(2,2,3)
    # plot(HR_x,HR_y,'k',HR_x_desired,HR_y_desired,'r')
    #     grid on
    #     ylabel('+ <-- y axis','fontsize',14)
    #     xlabel('x axis --> +','fontsize',14)
    #     title('HR End Effector in Steady State','fontsize',14)
    #     set(gca,'Ydir','reverse')
    # subplot(2,2,4)
    # plot(HL_x,HL_y,'k',HL_x_desired,HL_y_desired,'r')
    #     grid on
    #     ylabel('+ <-- y axis','fontsize',14)
    #     xlabel('x axis --> +','fontsize',14)
    #     title('HL End Effector in Steady State','fontsize',14)
    #     set(gca,'Ydir','reverse')
    # tightfig

    # #Responce of knee angles
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(4,1,1)
    # plot(t,FR_knee_angle_deg,'k',t,FR_Desired_knee_angle,'r')
    #     grid on
    #     ylabel('Angle [deg]')
    #     title('Response of FR Knee Angle','fontsize',13)
    # subplot(4,1,2)
    # plot(t,FL_knee_angle_deg,'k',t,FL_Desired_knee_angle,'r')
    #     grid on
    #     ylabel('Angle [deg]')
    #     title('Response of FL Knee Angle','fontsize',13)
    # subplot(4,1,3)
    # plot(t,HR_knee_angle_deg,'k',t,HR_Desired_knee_angle,'r')
    #     grid on
    #     ylabel('Angle [deg]')
    #     title('Response of HR Knee Angle','fontsize',13)
    # subplot(4,1,4)
    # plot(t,HL_knee_angle_deg,'k',t,HL_Desired_knee_angle,'r')
    #     grid on
    #     ylabel('Angle [deg]')
    #     xlabel('Time [s]')
    #     title('Response of HL Knee Angle','fontsize',13)
    # tightfig

    # #Responce of hip angles
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(4,1,1)
    # plot(t,FR_hip_angle_deg,'k',t,FR_Desired_hip_angle,'r')
    #     grid on
    #     ylabel('Angle [deg]')
    #     title('Response of FR Hip Angle','fontsize',13)
    # subplot(4,1,2)
    # plot(t,FL_hip_angle_deg,'k',t,FL_Desired_hip_angle,'r')
    #     grid on
    #     ylabel('Angle [deg]')
    #     title('Response of FL Hip Angle','fontsize',13)
    # subplot(4,1,3)
    # plot(t,HR_hip_angle_deg,'k',t,HR_Desired_hip_angle,'r')
    #     grid on
    #     ylabel('Angle [deg]')
    #     title('Response of HR Hip Angle','fontsize',13)
    # subplot(4,1,4)
    # plot(t,HL_hip_angle_deg,'k',t,HL_Desired_hip_angle,'r')
    #     grid on
    #     ylabel('Angle [deg]')
    #     xlabel('Time [s]')
    #     title('Response of HL Hip Angle','fontsize',13)
    # tightfig

    # #PWM Commands of Knee motors
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(4,1,1)
    # plot(t,FR_uk_knee,'k','LineWidth',0.1)
    # hold on
    # plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #     grid on
    #     ylabel('PWM Command [#]')
    #     title('PWM Command of FR Knee','fontsize',13)
    # subplot(4,1,2)
    # plot(t,FL_uk_knee,'k','LineWidth',0.1)
    # hold on
    # plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #     grid on
    #     ylabel('PWM Command [#]')
    #     title('PWM Command of FL Knee','fontsize',13)
    # subplot(4,1,3)
    # plot(t,HR_uk_knee,'k','LineWidth',0.1)
    # hold on
    # plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #     grid on
    #     ylabel('PWM Command [#]')
    #     title('PWM Command of HR Knee','fontsize',13)
    # subplot(4,1,4)
    # plot(t,HL_uk_knee,'k','LineWidth',0.1)
    # hold on
    # plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #     grid on
    #     ylabel('PWM Command [#]')
    #     xlabel('Time [s]')
    #     title('PWM Command of HL Knee','fontsize',13)
    # tightfig

    # #PWM Commands of Hip motors
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(4,1,1)
    # plot(t,FR_uk_hip,'k','LineWidth',0.1)
    # hold on
    # plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #     grid on
    #     ylabel('PWM Command [#]')
    #     title('PWM Command of FR Hip','fontsize',13)
    # subplot(4,1,2)
    # plot(t,FL_uk_hip,'k','LineWidth',0.1)
    # hold on
    # plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #     grid on
    #     ylabel('PWM Command [#]')
    #     title('PWM Command of FL Hip','fontsize',13)
    # subplot(4,1,3)
    # plot(t,HR_uk_hip,'k','LineWidth',0.1)
    # hold on
    # plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #     grid on
    #     ylabel('PWM Command [#]')
    #     title('PWM Command of HR Hip','fontsize',13)
    # subplot(4,1,4)
    # plot(t,HL_uk_hip,'k','LineWidth',0.1)
    # hold on
    # plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #     grid on
    #     ylabel('PWM Command [#]')
    #     xlabel('Time [s]')
    #     title('PWM Command of HL Hip','fontsize',13)
    # tightfig

    # #Velocity of Knee motors
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(4,1,1)
    # hold on
    # plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,FR_velocity_knee,'k')
    #     grid on
    #     ylabel('Velocity [rad/s]')
    #     title('Response of FR Knee Velocity','fontsize',13)
    # subplot(4,1,2)
    # hold on
    # plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,FL_velocity_knee,'k')
    #     grid on
    #     ylabel('Velocity [rad/s]')
    #     title('Response of FL Knee Velocity','fontsize',13)
    # subplot(4,1,3)
    # hold on
    # plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,HR_velocity_knee,'k')
    #     grid on
    #     ylabel('Velocity [rad/s]')
    #     title('Response of HR Knee Velocity','fontsize',13)
    # subplot(4,1,4)
    # hold on
    # plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,HL_velocity_knee,'k')
    #     grid on
    #     ylabel('Velocity [rad/s]')
    #     xlabel('Time [s]')
    #     title('Response of HL Knee Velocity','fontsize',13)
    # tightfig

    # #Velocity of Hip motors
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(4,1,1)
    # hold on
    # plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,FR_velocity_hip,'k')
    #     grid on
    #     ylabel('Velocity [rad/s]')
    #     title('Response of FR Hip Velocity','fontsize',13)
    # subplot(4,1,2)
    # hold on
    # plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,FL_velocity_hip,'k')
    #     grid on
    #     ylabel('Velocity [rad/s]')
    #     title('Response of FL Hip Velocity','fontsize',13)
    # subplot(4,1,3)
    # hold on
    # plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,HR_velocity_hip,'k')
    #     grid on
    #     ylabel('Velocity [rad/s]')
    #     title('Response of HR Hip Velocity','fontsize',13)
    # subplot(4,1,4)
    # hold on
    # plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,HL_velocity_hip,'k')
    #     grid on
    #     ylabel('Velocity [rad/s]')
    #     xlabel('Time [s]')
    #     title('Response of HL Hip Velocity','fontsize',13)
    # tightfig


#------------------------------------------------------------------------------#
    # # Plot Hind Right Leg's Diagrams
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # plot(t,HR_knee_angle_deg,'k',t,HR_Desired_knee_angle,'r')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Angle [deg]')
    #      xlabel('Time [s]')
    #    	 title('Response of HR Knee Angle','fontsize',13)
    # subplot(2,1,2)
    # plot(t,HR_hip_angle_deg,'k',t,HR_Desired_hip_angle,'r')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Angle [deg]')
    #      xlabel('Time [s]')
    #    	 title('Response of HR Hip Angle','fontsize',13)
    # tightfig
    #
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # plot(t,HR_uk_knee,'k','LineWidth',0.1)
    # hold on
    # plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #      grid on
    #      ylabel('PWM Command [#]')
    #      xlabel('Time [s]')
    #    	 title('PWM Command of HR Knee','fontsize',13)
    # subplot(2,1,2)
    # plot(t,HR_uk_hip,'k','LineWidth',0.1)
    # hold on
    # plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #      grid on
    # #     axis([2 7 -50 60])
    #      ylabel('PWM Command [#]')
    #      xlabel('Time [s]')
    #    	 title('PWM command of HR Hip','fontsize',13)
    # tightfig
    #
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # hold on
    # plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,HR_velocity_knee,'k')
    #      grid on
    #      ylabel('Velocity [rad/s]')
    #      xlabel('Time [s]')
    #    	 title('Response of HR Knee Velocity','fontsize',13)
    # subplot(2,1,2)
    # hold on
    # plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,HR_velocity_hip,'k')
    #      grid on
    #      ylabel('Velocity [rad/s]')
    #      xlabel('Time [s]')
    #    	 title('Response of HR Hip Velocity','fontsize',13)
    # tightfig
    #
    # # Plot Hind Left Leg's Diagrams
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # plot(t,HL_knee_angle_deg,'k',t,HL_Desired_knee_angle,'r')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Angle [deg]')
    #      xlabel('Time [s]')
    #    	 title('Response of HL Knee Angle','fontsize',13)
    # subplot(2,1,2)
    # plot(t,HL_hip_angle_deg,'k',t,HL_Desired_hip_angle,'r')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Angle [deg]')
    #      xlabel('Time [s]')
    #    	 title('Response of HL Hip Angle','fontsize',13)
    # tightfig
    #
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # plot(t,HL_uk_knee,'k','LineWidth',0.1)
    # hold on
    # plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #      grid on
    # #     axis([2 7 -50 60])
    #      ylabel('PWM Command [#]')
    #      xlabel('Time [s]')
    #    	 title('PWM Command of HL Knee','fontsize',13)
    # subplot(2,1,2)
    # plot(t,HL_uk_hip,'k','LineWidth',0.1)
    # hold on
    # plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #      grid on
    # #     axis([2 7 -50 60])
    #      ylabel('PWM Command [#]')
    #      xlabel('Time [s]')
    #    	 title('PWM command of HL Hip','fontsize',13)
    # tightfig
    #
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # hold on
    # plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,HL_velocity_knee,'k')
    #      grid on
    #      ylabel('Velocity [rad/s]')
    #      xlabel('Time [s]')
    #    	 title('Response of HL Knee Velocity','fontsize',13)
    # subplot(2,1,2)
    # hold on
    # plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,HL_velocity_hip,'k')
    #      grid on
    #      ylabel('Velocity [rad/s]')
    #      xlabel('Time [s]')
    #    	 title('Response of HL Hip Velocity','fontsize',13)
    # tightfig
    #
    # # Plot Fore Right Leg's Diagrams
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # hold all
    # plot(t,FR_knee_angle_deg,'k',t,FR_Desired_knee_angle,'r')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Angle [deg]')
    #      xlabel('Time [s]')
    #    	 title('Response of FR Knee Angle','fontsize',13)
    # subplot(2,1,2)
    # plot(t,FR_hip_angle_deg,'k',t,FR_Desired_hip_angle,'r')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Angle [deg]')
    #      xlabel('Time [s]')
    #    	 title('Response of FR Hip Angle','fontsize',13)
    # tightfig
    #
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # plot(t,FR_uk_knee,'k','LineWidth',0.1)
    # hold on
    # plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #      grid on
    # #     axis([2 7 -50 60])
    #      ylabel('PWM Command [#]')
    #      xlabel('Time [s]')
    #    	 title('PWM Command of FR Knee','fontsize',13)
    # subplot(2,1,2)
    # plot(t,FR_uk_hip,'k','LineWidth',0.1)
    # hold on
    # plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #      grid on
    # #     axis([2 7 -50 60])
    #      ylabel('PWM Command [#]')
    #      xlabel('Time [s]')
    #    	 title('PWM Command of FR Hip','fontsize',13)
    #  tightfig
    #
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # hold on
    # plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,FR_velocity_knee,'k')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Velocity [rad/s]')
    #      xlabel('Time [s]')
    #    	 title('Response of FR Knee Velocity','fontsize',13)
    # subplot(2,1,2)
    # hold on
    # plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,FR_velocity_hip,'k')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Velocity [rad/s]')
    #      xlabel('Time [s]')
    #    	 title('Response of FR Hip Velocity','fontsize',13)
    # tightfig
    #
    # # Plot Fore Left Leg's Diagrams
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # hold all
    # plot(t,FL_knee_angle_deg,'k',t,FL_Desired_knee_angle,'r')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Angle [deg]')
    #      xlabel('Time [s]')
    #    	 title('Response of FL Knee Angle','fontsize',13)
    # subplot(2,1,2)
    # plot(t,FL_hip_angle_deg,'k',t,FL_Desired_hip_angle,'r')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Angle [deg]')
    #      xlabel('Time [s]')
    #    	 title('Response of FL Hip Angle','fontsize',13)
    # tightfig
    #
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # plot(t,FL_uk_knee,'k','LineWidth',0.1)
    # hold on
    # plot(t,Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #      grid on
    # #     axis([2 7 -50 60])
    #      ylabel('PWM Command [#]')
    #      xlabel('Time [s]')
    #    	 title('PWM Command of FL Knee','fontsize',13)
    # subplot(2,1,2)
    # plot(t,FL_uk_hip,'k','LineWidth',0.1)
    # hold on
    # plot(t,Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_PWM_Limit*ones(length(t),1),'r','LineWidth',0.1)
    #      grid on
    # #     axis([2 7 -50 60])
    #      ylabel('PWM Command [#]')
    #      xlabel('Time [s]')
    #    	 title('PWM Command of FL Hip','fontsize',13)
    #  tightfig
    #
    # figure
    # set(gcf, 'Position', [100 50 900 800],'color','w')
    # subplot(2,1,1)
    # hold on
    # plot(t,Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Knee_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,FL_velocity_knee,'k')
    #      grid on
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Velocity [rad/s]')
    #      xlabel('Time [s]')
    #    	 title('Response of FL Knee Velocity','fontsize',13)
    # subplot(2,1,2)
    # hold on
    # plot(t,Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    # plot(t,-Hip_max_velocity*ones(length(t),1),'r','LineWidth',0.1)
    #      grid on
    # plot(t,FL_velocity_hip,'k')
    # #      axis([-0.7 0.7 -0.1 0.7])
    #      ylabel('Velocity [rad/s]')
    #      xlabel('Time [s]')
    #    	 title('Response of FL Hip Velocity','fontsize',13)
    # tightfig
