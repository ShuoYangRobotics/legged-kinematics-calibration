function z = simple_ekf_measurement(state, R_er, ...
        omega, joint_angle_list, joint_av_list, param)
    
z = zeros(param.num_leg,1);
for i = 1:param.num_leg 
    angle = [joint_angle_list((i-1)*3+1);joint_angle_list((i-1)*3+2);joint_angle_list((i-1)*3+3)];
    av = [joint_av_list((i-1)*3+1);joint_av_list((i-1)*3+2);joint_av_list((i-1)*3+3)];
    
    rho = state((i-1)*param.rho_opt_size+1:(i-1)*param.rho_opt_size+param.rho_opt_size);
    drho = state((i-1)*param.rho_opt_size+1+param.rho_opt_size*param.num_leg:(i-1)*param.rho_opt_size+param.rho_opt_size+param.rho_opt_size*param.num_leg);
    p_rf = autoFunc_fk_pf_pos(angle,rho,param.rho_fix(:,i));
    J_rf = autoFunc_d_fk_dt(angle,rho,param.rho_fix(:,i));
    df_drho = autoFunc_d_fk_drho(angle,rho,param.rho_fix(:,i));
    
%     leg_v = -R_er*(J_rf*av+df_drho*drho+skew(omega)*p_rf);
    leg_v = -R_er*(J_rf*av+skew(omega)*p_rf);
    % only the x velocity
    z(i) = leg_v(1);    
end
    
end