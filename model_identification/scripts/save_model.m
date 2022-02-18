function save_model(model, model_filename)
fileID = fopen(model_filename, 'w');

fprintf(fileID, '# Attitude Dynamics\n');
fprintf(fileID, 't_roll: %.5f\n', model.roll_time_c);
fprintf(fileID, 'k_roll: %.5f\n', model.roll_gain);
fprintf(fileID, 't_pitch: %.5f\n', model.pitch_time_c);
fprintf(fileID, 'k_pitch: %.5f\n', model.pitch_gain);
fprintf(fileID, '\n');

fprintf(fileID, '# Damping coefficients\n');
fprintf(fileID, 'damping_coef: [%.5f, %.5f, %.5f]\n', model.damping_coeff(1), model.damping_coeff(2), model.damping_coeff(3));
fprintf(fileID, '\n');

fprintf(fileID, '# Thrust coefficient\n');
fprintf(fileID, 'k_thrust: %.5f\n', model.thrust_coeff);
fprintf(fileID, '\n');

fprintf(fileID, '# Gravity on world frame\n');
fprintf(fileID, 'gravity: -9.8066\n');
fprintf(fileID, '\n');

fprintf(fileID, '# Cost function weights\n');
fprintf(fileID, 'pos_w: [20, 20, 20]\n');
fprintf(fileID, 'vel_w: [5, 5, 5]\n');
fprintf(fileID, 'att_w: [1, 1, 1]\n');
fprintf(fileID, '\n');

fprintf(fileID, 'yaw_rate_cmd_w: 10\n');
fprintf(fileID, 'pitch_cmd_w: 150\n');
fprintf(fileID, 'roll_cmd_w: 150\n');
fprintf(fileID, 'thrust_cmd_w: 150\n');
fprintf(fileID, '\n');

fclose(fileID);
end