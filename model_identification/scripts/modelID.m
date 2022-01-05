%% Clear everything
clear; close all; clc;

%% Load modelID data
data_file = 'sim_data_2021-12-23-16-08-29.csv';
modelID_data = readtable(data_file);

%% Estimate Attitude dynamics
dt = mean(diff(modelID_data.time));

roll_data = iddata(modelID_data.roll, modelID_data.Rcmd, dt);
pitch_data = iddata(modelID_data.pitch, modelID_data.Pcmd, dt);

roll_tf = tfest(roll_data, 1, 0);
pitch_tf = tfest(pitch_data, 1, 0);

tr = 1 / roll_tf.Denominator(2);
kr = roll_tf.Numerator / roll_tf.Denominator(2);

tp = 1 / pitch_tf.Denominator(2);
kp = pitch_tf.Numerator / pitch_tf.Denominator(2);

%% Estimate damping and thrust coefficient 
% Define Cost function
cost_fun = @(params)costFunction(modelID_data, params);

% Set initial value and bounds
% params = [kT, dx, dy, dz]
params_0 = [10, -0.5, -0.5, -0.5];
lb = [0, -100, -100, -100];
ub = [1000, 0, 0, 0];

% Setup and run Least Squares
opt_options = optimoptions('lsqnonlin','Display','iter', 'MaxIterations', 100);
params_opt = lsqnonlin(cost_fun, params_0, lb, ub, opt_options);

%% Save model parameters
model_params = [params_opt(2:4), params_opt(1), tp, tr, kp, kr];
save('model_params.mat', 'model_params');

%% Check Results
% xddot_est = zeros(length(modelID_data.time), 1);
% yddot_est = zeros(length(modelID_data.time), 1);
% zddot_est = zeros(length(modelID_data.time), 1);
% 
% for i = 2 : length(modelID_data.time)
%     [xddot_est(i), yddot_est(i), zddot_est(i)] = droneModel(...
%         modelID_data.xdot(i - 1), modelID_data.ydot(i - 1), modelID_data.zdot(i - 1), ...
%         [modelID_data.qw(i - 1), modelID_data.qx(i - 1), modelID_data.qy(i - 1), modelID_data.qz(i - 1)], ...
%         modelID_data.Tcmd(i - 1), ...
%         params_opt(1), params_opt(2), params_opt(3), params_opt(4));
% end
% 
% %% Plot results
% 
% figure
% subplot(4, 1, 1)
% hold on
% plot(modelID_data.time, modelID_data.xddot, 'b')
% plot(modelID_data.time, xddot_est, 'r')
% hold off
% subplot(4, 1, 2)
% hold on
% plot(modelID_data.time, modelID_data.yddot, 'b')
% plot(modelID_data.time, yddot_est, 'r')
% hold off
% subplot(4, 1, 3)
% hold on
% plot(modelID_data.time, modelID_data.zddot, 'b')
% plot(modelID_data.time, zddot_est, 'r')
% hold off
% subplot(4, 1, 4)
% plot(modelID_data.time, modelID_data.Tcmd)