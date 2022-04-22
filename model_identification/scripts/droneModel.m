function [state_k] = droneModel(state, input, model_params, dt)
% state = [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz]'
% input = [y_cmd, p_cmd, r_cmd, T_cmd]'
% model_params = [dxy, dz, kT, tp, tr, kp, kr]'

% Propagate state
k1 = droneModelRK(state, input, model_params);
k2 = droneModelRK(addUpdate(state, 0.5 * dt * k1), input, model_params);
k3 = droneModelRK(addUpdate(state, 0.5 * dt * k2), input, model_params);
k4 = droneModelRK(addUpdate(state, dt * k3), input, model_params);

state_k = addUpdate(state, dt * (k1 + 2 * k2 + 2 * k3 + k4)/6);

end

function [X_dot] = droneModelRK(X, input, model_params)
% Gravity
G = [0; 0; -9.8066];

% Thrust
T = [0; 0; model_params(3)*input(4)];
R = quat2rotm(X(7:10)');
T_W = R * T;

% Damping
D = diag([model_params(1), model_params(1), model_params(2)]) * X(4:6);

v_k = (D + T_W + G);

%% Position update
p_k = X(4:6);

%% Attitude update
kr = model_params(7);
kp = model_params(6);
tr = model_params(5);
tp = model_params(4);

qw = X(7);
qx = X(8);
qy = X(9);
qz = X(10);

ypr = quat2eul(X(7:10)');

omega_x = (kr * input(3) - ypr(3)) / tr;
omega_y = (kp * input(2) - ypr(2)) / tp;
omega_z = input(1);

q_w_dot = 0.5 * (-omega_x*qx - omega_y*qy - omega_z*qz);
q_x_dot = 0.5 * ( omega_x*qw + omega_y*qz - omega_z*qy);
q_y_dot = 0.5 * (-omega_x*qz + omega_y*qw + omega_z*qx);
q_z_dot = 0.5 * ( omega_x*qy - omega_y*qw + omega_z*qw);

%% Update State
X_dot = [p_k; v_k; q_w_dot; q_x_dot; q_y_dot; q_z_dot];

end

function x_k = addUpdate(x, dx)
% Position
x_k(1:3, 1) = x(1:3) + dx(1:3);
% Velocity
x_k(4:6, 1) = x(4:6) + dx(4:6);
% Attitude
x_k(7:10, 1) = quatnormalize((x(7:10) + dx(7:10))')';
end