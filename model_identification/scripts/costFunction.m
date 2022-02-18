function J = costFunction(state, params)

J = 0;

for i = 1 : (length(state.time) - 1)
    [xddot_est, yddot_est, zddot_est] = droneModel(...
        state.xdot(i), state.ydot(i), state.zdot(i), ...
        [state.qw(i), state.qx(i), state.qy(i), state.qz(i)], ...
        state.Tcmd(i), ...
        params(1), params(2), params(2), params(3));

    %     dxddot = state.xddot(i) - xddot_est;
    %     dyddot = state.yddot(i) - yddot_est;
    %     dzddot = state.zddot(i) - zddot_est;

    dt = state.time(i + 1) - state.time(i);
    dxddot = state.xdot(i + 1) - (state.xdot(i) + xddot_est * dt);
    dyddot = state.ydot(i + 1) - (state.ydot(i) + yddot_est * dt);
    dzddot = state.zdot(i + 1) - (state.zdot(i) + zddot_est * dt);

    J = J + dxddot*dxddot + dyddot*dyddot + dzddot*dzddot;

end

end