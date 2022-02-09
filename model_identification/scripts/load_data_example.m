%% Clear everything
clear; close all; clc;

bag_file = 'data/sim_data_2021-12-23-16-08-29.bag';
flight_data = rosbag(bag_file);

% Odometry
bag_select = select(flight_data, 'Topic', '/mavros/local_position/odom');
bag_struct = readMessages(bag_select, 'DataFormat', 'struct');

odom_data = table();
odom_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, bag_struct);

odom_data.x = cellfun(@(m) double(m.Pose.Pose.Position.X), bag_struct);
odom_data.y = cellfun(@(m) double(m.Pose.Pose.Position.Y), bag_struct);
odom_data.z = cellfun(@(m) double(m.Pose.Pose.Position.Z), bag_struct);

odom_data.qw = cellfun(@(m) double(m.Pose.Pose.Orientation.W), bag_struct);
odom_data.qx = cellfun(@(m) double(m.Pose.Pose.Orientation.X), bag_struct);
odom_data.qy = cellfun(@(m) double(m.Pose.Pose.Orientation.Y), bag_struct);
odom_data.qz = cellfun(@(m) double(m.Pose.Pose.Orientation.Z), bag_struct);

odom_data.xdot = cellfun(@(m) double(m.Twist.Twist.Linear.X), bag_struct);
odom_data.ydot = cellfun(@(m) double(m.Twist.Twist.Linear.Y), bag_struct);
odom_data.zdot = cellfun(@(m) double(m.Twist.Twist.Linear.Z), bag_struct);
