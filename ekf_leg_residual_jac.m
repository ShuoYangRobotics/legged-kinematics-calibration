function jac = ekf_leg_residual_jac(state, meas, param)


jac = zeros(3*param.num_leg, param.state_size-1);

body_p = state(1:3);
body_q = quaternion(state(4:7)');
body_v = state(8:10);
R_er = quat2rotm(body_q);

joint_angle_list = meas(7:18);
joint_av_list = meas(19:30);
omega = meas(4:6);
for i = 1:param.num_leg
    angle = joint_angle_list((i-1)*3+1:(i-1)*3+3);
    av = joint_av_list((i-1)*3+1:(i-1)*3+3);
    % get opt rho TODO: check dimension here
    rho_opt = state(16+i,1);
    p_rf = autoFunc_fk_pf_pos(angle,[rho_opt],[param.ox(i);param.oy(i);param.d(i);param.lt]);
    J_rf = autoFunc_d_fk_dt(angle,[rho_opt],[param.ox(i);param.oy(i);param.d(i);param.lt]);
    leg_v = (-J_rf*av-skew(omega)*p_rf);
    
    % position 0
    jac((i-1)*3+1:(i-1)*3+3,1:3) = zeros(3,3);
    % orientation 
    jac((i-1)*3+1:(i-1)*3+3,4:6) = R_er*skew(leg_v);
    % velocity
    jac((i-1)*3+1:(i-1)*3+3,7:9) = eye(3);
    % rho_opt
    dJdrho = autoFunc_dJ_drho(angle,[rho_opt],[param.ox(i);param.oy(i);param.d(i);param.lt]);
    dfdrho = autoFunc_d_fk_drho(angle,[rho_opt],[param.ox(i);param.oy(i);param.d(i);param.lt]);
    g = -R_er*(kron(av',eye(3))*dJdrho+skew(omega)*dfdrho);
    jac((i-1)*3+1:(i-1)*3+3,15+i) = -g;
    
end


end