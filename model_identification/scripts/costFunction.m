function J = costFunction(state, params)
% state = [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz]'
% input = [y_cmd, p_cmd, r_cmd, T_cmd]'
% model_params = [dx, dy, dz, kT, tp, tr, kp, kr]'

J = 0;

for i = 1 : (length(state.time) - 1)
    dt = state.time(i + 1) - state.time(i);
    
    X = [...
        state.x(i); state.y(i); state.z(i);...
        state.xdot(i); state.ydot(i); state.zdot(i);...
        state.qw(i); state.qx(i); state.qy(i); state.qz(i);];
    
    u = [state.Ycmd(i); state.Pcmd(i); state.Rcmd(i); state.Tcmd(i);];
    
    state_k = droneModel(X, u, params, dt);
    
    Xk = [...
%         state.x(i + 1); state.y(i + 1); state.z(i + 1);...
        state.xdot(i + 1); state.ydot(i + 1); state.zdot(i + 1)];
    
    dX = Xk - state_k(4:6);
    
    J = J + dX' * dX;
    
end

end