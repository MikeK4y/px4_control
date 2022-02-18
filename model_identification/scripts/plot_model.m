function plot_model(modelID_data, model)
%% Generate model data
xdot_est = zeros(length(modelID_data.time), 1);
ydot_est = zeros(length(modelID_data.time), 1);
zdot_est = zeros(length(modelID_data.time), 1);

for i = 1 : (length(modelID_data.time) - 1)
    [xddot_est, yddot_est, zddot_est] = droneModel(...
        modelID_data.xdot(i), modelID_data.ydot(i), modelID_data.zdot(i), ...
        [modelID_data.qw(i), modelID_data.qx(i), modelID_data.qy(i), modelID_data.qz(i)], ...
        modelID_data.Tcmd(i), ...
        model.thrust_coeff, model.damping_coeff(1), model.damping_coeff(2), model.damping_coeff(3));

    dt = modelID_data.time(i + 1) - modelID_data.time(i);
    xdot_est(i + 1) = modelID_data.xdot(i) + dt * xddot_est;
    ydot_est(i + 1) = modelID_data.ydot(i) + dt * yddot_est;
    zdot_est(i + 1) = modelID_data.zdot(i) + dt * zddot_est;
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