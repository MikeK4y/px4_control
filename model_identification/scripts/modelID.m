%% Clear everything
clear; close all; clc;

%% Load modelID data
data_file = '../data/f550.csv';
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
% params = [kT, dxy, dz]
params_0 = [10, -0.5, -0.5];
lb = [0, -100, -100];
ub = [1000, 0, 0];

% Setup and run Least Squares
opt_options = optimoptions('lsqnonlin','Display','iter', 'MaxIterations', 100);
params_opt = lsqnonlin(cost_fun, params_0, lb, ub, opt_options);

%% Save model parameters
model_params = [params_opt(2), params_opt(2), params_opt(3), params_opt(1), tp, tr, kp, kr];
save('model_params.mat', 'model_params');

%% Check Results
xdot_est = zeros(length(modelID_data.time), 1);
ydot_est = zeros(length(modelID_data.time), 1);
zdot_est = zeros(length(modelID_data.time), 1);

for i = 1 : (length(modelID_data.time) - 1)
    [xddot_est, yddot_est, zddot_est] = droneModel(...
        modelID_data.xdot(i), modelID_data.ydot(i), modelID_data.zdot(i), ...
        [modelID_data.qw(i), modelID_data.qx(i), modelID_data.qy(i), modelID_data.qz(i)], ...
        modelID_data.Tcmd(i), ...
        params_opt(1), params_opt(2), params_opt(2), params_opt(3));

    dt = modelID_data.time(i + 1) - modelID_data.time(i);
    xdot_est(i + 1) = modelID_data.xdot(i) + dt * xddot_est;
    ydot_est(i + 1) = modelID_data.ydot(i) + dt * yddot_est;
    zdot_est(i + 1) = modelID_data.zdot(i) + dt * zddot_est;
end

%% Plot results
figure
subplot(2,1,1)
compare(roll_data, roll_tf)
subplot(2,1,2)
compare(pitch_data, pitch_tf)

figure
subplot(3, 1, 1)
hold on
plot(modelID_data.time, modelID_data.xdot, 'b')
plot(modelID_data.time, xdot_est, 'r')
hold off
subplot(3, 1, 2)
hold on
plot(modelID_data.time, modelID_data.ydot, 'b')
plot(modelID_data.time, ydot_est, 'r')
hold off
subplot(3, 1, 3)
hold on
plot(modelID_data.time, modelID_data.zdot, 'b')
plot(modelID_data.time, zdot_est, 'r')
hold off