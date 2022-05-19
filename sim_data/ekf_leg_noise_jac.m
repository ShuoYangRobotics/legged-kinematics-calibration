function jac = ekf_leg_noise_jac(state, meas, dt, param)

% jac is 12x12, which is the same as leg_r

V = zeros(6,6);
V(1:3,1:3) = 0.0000001;  % joint angle noise
V(4:6,4:6) = 0.0000001;  % joint angular velocity noise

J = zeros(12, 6);

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
    rho_opt = state(10+(i-1)*param.rho_opt_size+1:10+i*param.rho_opt_size,1);
    p_rf = autoFunc_fk_pf_pos(angle,rho_opt,param.rho_fix(:,i));
    J_rf = autoFunc_d_fk_dt(angle,rho_opt,param.rho_fix(:,i));
    
    dJdt = autoFunc_dJ_dt(angle,rho_opt,param.rho_fix(:,i));
    h = R_er*(kron(av',eye(3))*dJdt+skew(omega)*J_rf);
    
    J((i-1)*3+1:(i-1)*3+3,1:3) = h;
    J((i-1)*3+1:(i-1)*3+3,4:6) = R_er*J_rf;
end


jac = J*V*J';

end