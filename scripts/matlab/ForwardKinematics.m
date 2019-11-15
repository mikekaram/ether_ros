function [ x,y ] = ForwardKinematics( hip_angle_deg,knee_angle_deg )
%UNTITLED Forward Kinematics of Laelaps II
%   x (+) right, y (+) down
% Forward Kinematics
% Leg Parameters
l1 = 0.25;
l2 = 0.35;
hip_angle_rad = hip_angle_deg * pi/180;
knee_angle_rad = knee_angle_deg * pi/180;
x = l1 * sin(hip_angle_rad) + l2 * sin(knee_angle_rad);
y = l1 * cos(hip_angle_rad) + l2 * cos(knee_angle_rad);
end

