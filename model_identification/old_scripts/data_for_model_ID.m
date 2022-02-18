%% Clear everything
clear; close all; clc;

%% Load flight data
bag_file = '../data/f550.bag';
flight_data = rosbag(bag_file);
 
% cmd_data.thrust = rc_rev(3) * (cellfun(@(m) double(m.Channels(3)), bag_struct) - rc_min(3)) / (2 * rc_half_range(3));
% cmd_data.roll = rc_rev(1) * rp_max * ((cellfun(@(m) double(m.Channels(1)), bag_struct) - rc_mean(1)) / rc_half_range(1));
% cmd_data.pitch = rc_rev(2) * rp_max * ((cellfun(@(m) double(m.Channels(2)), bag_struct) - rc_mean(2)) / rc_half_range(2));
% cmd_data.yaw_rate = rc_rev(4) * yr_max * ((cellfun(@(m) double(m.Channels(4)), bag_struct) - rc_mean(4)) / rc_half_range(4));

% RC boundary values
rc_max = 2006;
rc_min = 982;
rc_mean = 0.5 * (rc_max + rc_min);
rc_half_range = 0.5 * (rc_max - rc_min);

rp_max = deg2rad(35);
yr_max = deg2rad(45);

% Attitude/Thrust commands
bag_select = select(flight_data, 'Topic', '/mavros/rc/in');
bag_struct = readMessages(bag_select, 'DataFormat', 'struct');

cmd_data = table();
cmd_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, bag_struct);

cmd_data.thrust = (cellfun(@(m) double(m.Channels(1)), bag_struct) - rc_min) / (2 * rc_half_range);
cmd_data.roll = rp_max * ((cellfun(@(m) double(m.Channels(2)), bag_struct) - rc_mean) / rc_half_range);
cmd_data.pitch = rp_max * ((cellfun(@(m) double(m.Channels(3)), bag_struct) - rc_mean) / rc_half_range);
cmd_data.yaw_rate = yr_max * ((cellfun(@(m) double(m.Channels(4)), bag_struct) - rc_mean) / rc_half_range);

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

%% Process Data

% Get Euler Angles from Quaternion
% Get velocities on world frame
for i = 1 : length(odom_data.time)
    pdot_B = [0, odom_data.xdot(i), odom_data.ydot(i), odom_data.zdot(i)];
    q = [odom_data.qw(i), odom_data.qx(i), odom_data.qy(i), odom_data.qz(i)];
    
    pdot_W = quatmultiply(q, quatmultiply(pdot_B, quatinv(q)));
    
    odom_data.xdot_W(i) = pdot_W(2);
    odom_data.ydot_W(i) = pdot_W(3);
    odom_data.zdot_W(i) = pdot_W(4);

    ypr = quat2eul(q);
    odom_data.yaw(i) = ypr(1);
    odom_data.pitch(i) = ypr(2);
    odom_data.roll(i) = ypr(3);
end

% Get everything on CMD time
modelID_data = table();
modelID_data.time = cmd_data.time;

modelID_data.x = interp1(odom_data.time, odom_data.x, modelID_data.time, 'spline');
modelID_data.y = interp1(odom_data.time, odom_data.y, modelID_data.time, 'spline');
modelID_data.z = interp1(odom_data.time, odom_data.z, modelID_data.time, 'spline');

modelID_data.xdot = interp1(odom_data.time, odom_data.xdot_W, modelID_data.time, 'spline');
modelID_data.ydot = interp1(odom_data.time, odom_data.ydot_W, modelID_data.time, 'spline');
modelID_data.zdot = interp1(odom_data.time, odom_data.zdot_W, modelID_data.time, 'spline');

modelID_data.qw = interp1(odom_data.time, odom_data.qw, modelID_data.time, 'spline');
modelID_data.qx = interp1(odom_data.time, odom_data.qx, modelID_data.time, 'spline');
modelID_data.qy = interp1(odom_data.time, odom_data.qy, modelID_data.time, 'spline');
modelID_data.qz = interp1(odom_data.time, odom_data.qz, modelID_data.time, 'spline');

modelID_data.yaw = interp1(odom_data.time, odom_data.yaw, modelID_data.time, 'spline');
modelID_data.pitch = interp1(odom_data.time, odom_data.pitch, modelID_data.time, 'spline');
modelID_data.roll = interp1(odom_data.time, odom_data.roll, modelID_data.time, 'spline');

modelID_data.Tcmd = cmd_data.thrust;
modelID_data.Ycmd = cmd_data.yaw_rate;
modelID_data.Pcmd = cmd_data.pitch;
modelID_data.Rcmd = cmd_data.roll;

%% Save data
data_filename = [bag_file(1:end-3), 'csv'];

writetable(modelID_data, data_filename);