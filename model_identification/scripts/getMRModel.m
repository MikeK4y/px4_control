function mr_model = getMRModel(modelID_data)
%% Estimate Attitude dynamics
dt = mean(diff(modelID_data.time));

roll_data = iddata(modelID_data.roll, modelID_data.Rcmd, dt);
pitch_data = iddata(modelID_data.pitch, modelID_data.Pcmd, dt);

roll_tf = tfest(roll_data, 1, 0);
pitch_tf = tfest(pitch_data, 1, 0);

mr_model.roll_time_c = 1 / roll_tf.Denominator(2);
mr_model.roll_gain = roll_tf.Numerator / roll_tf.Denominator(2);
if mr_model.roll_gain > 1.0
    mr_model.roll_gain = 1.0;
end

mr_model.pitch_time_c = 1 / pitch_tf.Denominator(2);
mr_model.pitch_gain = pitch_tf.Numerator / pitch_tf.Denominator(2);
if mr_model.pitch_gain > 1.0
    mr_model.pitch_gain = 1.0;
end
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

mr_model.thrust_coeff = params_opt(1);
mr_model.damping_coeff = [params_opt(2), params_opt(2), params_opt(3)];
end