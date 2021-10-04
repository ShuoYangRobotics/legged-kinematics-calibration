function r = ekf_leg_residual(state, meas, param)
% for feature residual, 
% state contains the pose of the robot 
% meas contains the projected feature locations of the robot
body_p = state(1:3);
body_q = quaternion(state(4:7)');
body_v = state(8:10);

joint_angle_list = meas(7:18);
joint_av_list = meas(19:30);
omega = meas(4:6);

r = zeros(3*param.num_leg,1);
for i = 1:param.num_leg
    angle = joint_angle_list((i-1)*3+1:(i-1)*3+3);
    av = joint_av_list((i-1)*3+1:(i-1)*3+3);
    % get opt rho TODO: check dimension here
    rho_opt = state(16+i,1);
    p_rf = autoFunc_fk_pf_pos(angle,[rho_opt],[param.ox(i);param.oy(i);param.d(i);param.lt]);
    J_rf = autoFunc_d_fk_dt(angle,[rho_opt],[param.ox(i);param.oy(i);param.d(i);param.lt]);
    % it seems velocity on y direction cannot be very correctly infered 
    leg_v = (-J_rf*av-skew(omega)*p_rf);
    r((i-1)*3+1:(i-1)*3+3) = body_v - leg_v;
end


end