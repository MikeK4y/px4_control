function J = costFunction(state, params)

J = 0;

for i = 1 : (length(state.time) - 1)
    [xddot_est, yddot_est, zddot_est] = droneModel(...
        state.xdot(i), state.ydot(i), state.zdot(i), ...
        [state.qw(i), state.qx(i), state.qy(i), state.qz(i)], ...
        state.Tcmd(i), ...
        params(1), params(2), params(2), params(3));
    
    dxddot = state.xddot(i + 1) - xddot_est;
    dyddot = state.yddot(i + 1) - yddot_est;
    dzddot = state.zddot(i + 1) - zddot_est;
    
    J = J + dxddot*dxddot + dyddot*dyddot + dzddot*dzddot;
    
end

end