function [xddot, yddot, zddot] = droneModel(xdot, ydot, zdot, qW_B, Tcmd, kT, dx, dy, dz)

% Gravity
G = [0; 0; -9.8066];

% Thrust / m
T = [0; 0; kT*Tcmd];
T_W = quat2rotm(quatnormalize(qW_B)) * T;

% Damping
D = diag([dx, dy, dz]) * [xdot; ydot; zdot];

pddot = D + T_W + G;

xddot = pddot(1);
yddot = pddot(2);
zddot = pddot(3);

end