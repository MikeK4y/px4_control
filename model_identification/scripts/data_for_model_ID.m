%% Clear everything
clear; close all; clc;

%% Load flight data
% bag_file = 'data/sim_data_2021-12-22-12-02-28.bag';
% bag_file = 'data/sim_data_2021-12-23-15-09-10.bag';
% bag_file = 'data/sim_data_2021-12-23-15-30-48.bag';
bag_file = 'data/sim_data_2021-12-23-16-08-29.bag';
flight_data = rosbag(bag_file);

% Attitude/Thrust commands
bag_select = select(flight_data, 'Topic', '/mavros/setpoint_raw/attitude');
bag_struct = readMessages(bag_select, 'DataFormat', 'struct');

cmd_data = table();
cmd_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, bag_struct);

cmd_data.qw = cellfun(@(m) double(m.Orientation.W), bag_struct);
cmd_data.qx = cellfun(@(m) double(m.Orientation.X), bag_struct);
cmd_data.qy = cellfun(@(m) double(m.Orientation.Y), bag_struct);
cmd_data.qz = cellfun(@(m) double(m.Orientation.Z), bag_struct);

cmd_data.thrust = cellfun(@(m) double(m.Thrust), bag_struct);

cmd_data.yaw_rate = cellfun(@(m) double(m.BodyRate.Z), bag_struct);

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

% IMU
bag_select = select(flight_data, 'Topic', '/mavros/imu/data');
bag_struct = readMessages(bag_select, 'DataFormat', 'struct');

imu_data = table();
imu_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, bag_struct);

imu_data.xddot = cellfun(@(m) double(m.LinearAcceleration.X), bag_struct);
imu_data.yddot = cellfun(@(m) double(m.LinearAcceleration.Y), bag_struct);
imu_data.zddot = cellfun(@(m) double(m.LinearAcceleration.Z), bag_struct);

imu_data.qw = cellfun(@(m) double(m.Orientation.W), bag_struct);
imu_data.qx = cellfun(@(m) double(m.Orientation.X), bag_struct);
imu_data.qy = cellfun(@(m) double(m.Orientation.Y), bag_struct);
imu_data.qz = cellfun(@(m) double(m.Orientation.Z), bag_struct);

imu_data.qxdot = cellfun(@(m) double(m.AngularVelocity.X), bag_struct);
imu_data.qydot = cellfun(@(m) double(m.AngularVelocity.Y), bag_struct);
imu_data.qzdot = cellfun(@(m) double(m.AngularVelocity.Z), bag_struct);

%% Process Data

% Convert CMDs from Quaternion to RPY
for i = 1 : length(cmd_data.time)
    q = [cmd_data.qw(i), cmd_data.qx(i), cmd_data.qy(i), cmd_data.qz(i)];
    
    ypr = quat2eul(q);
    
    cmd_data.r(i) = ypr(3);
    cmd_data.p(i) = ypr(2);
end

% Remove gravity from IMU data and get Acceleration on World frame
% Also get Euler Angles from Quaternion
gv = [0, 0, 0, -9.8066];

for i = 1 : length(imu_data.time)
    pddot = [0, imu_data.xddot(i), imu_data.yddot(i), imu_data.zddot(i)];
    q = [imu_data.qw(i), imu_data.qx(i), imu_data.qy(i), imu_data.qz(i)];
    
    gv_B = quatmultiply(quatinv(q), quatmultiply(gv, q));
    
    pddot_B = pddot + gv_B;
    
    pddot_W = quatmultiply(q, quatmultiply(pddot_B, quatinv(q)));
    
    imu_data.xddot(i) = pddot_W(2);
    imu_data.yddot(i) = pddot_W(3);
    imu_data.zddot(i) = pddot_W(4);
    
    ypr = quat2eul(q);
    imu_data.yaw(i) = ypr(1);
    imu_data.pitch(i) = ypr(2);
    imu_data.roll(i) = ypr(3);
end

% Get velocities on world frame
for i = 1 : length(odom_data.time)
    pdot_B = [0, odom_data.xdot(i), odom_data.ydot(i), odom_data.zdot(i)];
    q = [odom_data.qw(i), odom_data.qx(i), odom_data.qy(i), odom_data.qz(i)];
    
    pdot_W = quatmultiply(q, quatmultiply(pdot_B, quatinv(q)));
    
    odom_data.xdot_W(i) = pdot_W(2);
    odom_data.ydot_W(i) = pdot_W(3);
    odom_data.zdot_W(i) = pdot_W(4);
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

modelID_data.xddot = interp1(imu_data.time, imu_data.xddot, modelID_data.time, 'spline');
modelID_data.yddot = interp1(imu_data.time, imu_data.yddot, modelID_data.time, 'spline');
modelID_data.zddot = interp1(imu_data.time, imu_data.zddot, modelID_data.time, 'spline');

modelID_data.qw = interp1(imu_data.time, imu_data.qw, modelID_data.time, 'spline');
modelID_data.qx = interp1(imu_data.time, imu_data.qx, modelID_data.time, 'spline');
modelID_data.qy = interp1(imu_data.time, imu_data.qy, modelID_data.time, 'spline');
modelID_data.qz = interp1(imu_data.time, imu_data.qz, modelID_data.time, 'spline');

modelID_data.qxdot = interp1(imu_data.time, imu_data.qxdot, modelID_data.time, 'spline');
modelID_data.qydot = interp1(imu_data.time, imu_data.qydot, modelID_data.time, 'spline');
modelID_data.qzdot = interp1(imu_data.time, imu_data.qzdot, modelID_data.time, 'spline');

modelID_data.yaw = interp1(imu_data.time, imu_data.yaw, modelID_data.time, 'spline');
modelID_data.pitch = interp1(imu_data.time, imu_data.pitch, modelID_data.time, 'spline');
modelID_data.roll = interp1(imu_data.time, imu_data.roll, modelID_data.time, 'spline');

modelID_data.Tcmd = cmd_data.thrust;
modelID_data.Ycmd = cmd_data.yaw_rate;
modelID_data.Pcmd = cmd_data.p;
modelID_data.Rcmd = cmd_data.r;

% Measurements
measurement_data = table();
measurement_data.time = cmd_data.time;

measurement_data.x = interp1(odom_data.time, odom_data.x, measurement_data.time, 'spline');
measurement_data.y = interp1(odom_data.time, odom_data.y, measurement_data.time, 'spline');
measurement_data.z = interp1(odom_data.time, odom_data.z, measurement_data.time, 'spline');

measurement_data.xdot = interp1(odom_data.time, odom_data.xdot, measurement_data.time, 'spline');
measurement_data.ydot = interp1(odom_data.time, odom_data.ydot, measurement_data.time, 'spline');
measurement_data.zdot = interp1(odom_data.time, odom_data.zdot, measurement_data.time, 'spline');

measurement_data.yaw = interp1(imu_data.time, imu_data.yaw, measurement_data.time, 'spline');
measurement_data.pitch = interp1(imu_data.time, imu_data.pitch, measurement_data.time, 'spline');
measurement_data.roll = interp1(imu_data.time, imu_data.roll, measurement_data.time, 'spline');

%% Save data
data_filename = [bag_file(1:end-3), 'csv'];
measurement_filename = [bag_file(1:end-4), '_measurements.csv'];

writetable(modelID_data, data_filename);
writetable(measurement_data, measurement_filename);