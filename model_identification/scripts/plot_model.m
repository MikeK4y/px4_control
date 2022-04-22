function plot_model(modelID_data, model)
% state = [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz]'
% input = [y_cmd, p_cmd, r_cmd, T_cmd]'
% model_params = [dxy, dz, kT, tp, tr, kp, kr]'

%% Generate model data
% x_est = zeros(length(modelID_data.time), 1);
% y_est = zeros(length(modelID_data.time), 1);
% z_est = zeros(length(modelID_data.time), 1);

xdot_est = zeros(length(modelID_data.time), 1);
ydot_est = zeros(length(modelID_data.time), 1);
zdot_est = zeros(length(modelID_data.time), 1);

params = [...
    model.damping_coeff(2); model.damping_coeff(3);...
    model.thrust_coeff;...
    model.pitch_time_c; model.roll_time_c; model.pitch_gain; model.roll_gain];

for i = 1 : (length(modelID_data.time) - 1)
    dt = modelID_data.time(i + 1) - modelID_data.time(i);
    
    X = [...
        modelID_data.x(i); modelID_data.y(i); modelID_data.z(i);...
        modelID_data.xdot(i); modelID_data.ydot(i); modelID_data.zdot(i);...
        modelID_data.qw(i); modelID_data.qx(i); modelID_data.qy(i); modelID_data.qz(i);];
    
    u = [modelID_data.Ycmd(i); modelID_data.Pcmd(i); modelID_data.Rcmd(i); modelID_data.Tcmd(i);];
    
    state_k = droneModel(X, u, params, dt);
    
%     x_est(i + 1) = state_k(1);
%     y_est(i + 1) = state_k(2);
%     z_est(i + 1) = state_k(3);
    
    xdot_est(i + 1) = state_k(4);
    ydot_est(i + 1) = state_k(5);
    zdot_est(i + 1) = state_k(6);
end

dt = mean(diff(modelID_data.time));
roll_data = iddata(modelID_data.roll, modelID_data.Rcmd, dt);
pitch_data = iddata(modelID_data.pitch, modelID_data.Pcmd, dt);

roll_tf = idtf(model.roll_gain / model.roll_time_c, [1, 1/model.roll_time_c]);
pitch_tf = idtf(model.pitch_gain / model.pitch_time_c, [1, 1/model.pitch_time_c]);

%% Figures
% Attitude
figure
subplot(2,1,1)
hold on
compare(roll_data, roll_tf)
title('Roll data and model')
hold off
subplot(2,1,2)
hold on
compare(pitch_data, pitch_tf)
title('Pitch data and model')
hold off

% % Position
% figure
% subplot(3, 1, 1)
% hold on
% plot(modelID_data.time(2:end), modelID_data.x(2:end), 'b')
% plot(modelID_data.time(2:end), x_est(2:end), 'r.')
% title('Position x-axis')
% hold off
% subplot(3, 1, 2)
% hold on
% plot(modelID_data.time(2:end), modelID_data.y(2:end), 'b')
% plot(modelID_data.time(2:end), y_est(2:end), 'r.')
% title('Position y-axis')
% hold off
% subplot(3, 1, 3)
% hold on
% plot(modelID_data.time(2:end), modelID_data.z(2:end), 'b')
% plot(modelID_data.time(2:end), z_est(2:end), 'r.')
% title('Position z-axis')
% hold off

% Velocity
figure
subplot(3, 1, 1)
hold on
plot(modelID_data.time(2:end), modelID_data.xdot(2:end), 'b')
plot(modelID_data.time(2:end), xdot_est(2:end), 'r.')
title('Velocity x-axis')
hold off
subplot(3, 1, 2)
hold on
plot(modelID_data.time(2:end), modelID_data.ydot(2:end), 'b')
plot(modelID_data.time(2:end), ydot_est(2:end), 'r.')
title('Velocity y-axis')
hold off
subplot(3, 1, 3)
hold on
plot(modelID_data.time(2:end), modelID_data.zdot(2:end), 'b')
plot(modelID_data.time(2:end), zdot_est(2:end), 'r.')
title('Velocity z-axis')
hold off
end