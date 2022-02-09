function jac = simple_ekf_measurement_jac(state, R_er, ...
        omega, joint_angle_list, joint_av_list, param)

jac = zeros(3*param.num_leg, param.rho_opt_size*param.num_leg*2);    
    
for i = 1:param.num_leg
    angle = [joint_angle_list((i-1)*3+1);joint_angle_list((i-1)*3+2);joint_angle_list((i-1)*3+3)];
    av = [joint_av_list((i-1)*3+1);joint_av_list((i-1)*3+2);joint_av_list((i-1)*3+3)];
   
    rho = state((i-1)*param.rho_opt_size+1:(i-1)*param.rho_opt_size+param.rho_opt_size);
    drho = state((i-1)*param.rho_opt_size+1+param.rho_opt_size*param.num_leg:(i-1)*param.rho_opt_size+param.rho_opt_size+param.rho_opt_size*param.num_leg);
    J_rf = autoFunc_d_fk_dt(angle,rho,param.rho_fix(:,i));
    dJdrho = autoFunc_dJ_drho(angle,rho,param.rho_fix(:,i));
    dfdrho = autoFunc_d_fk_drho(angle,rho,param.rho_fix(:,i));
    dfddrho = autoFunc_d_fk_ddrho(angle,rho,param.rho_fix(:,i));
    dz_drho = -R_er*(kron(av',eye(3))*dJdrho+skew(omega)*dfdrho);

    dz_ddrho = -R_er*dfdrho;
%     jac(i,(i-1)*param.rho_opt_size+1:i*param.rho_opt_size) = dz_drho(1,:);
    jac((i-1)*3+1:(i-1)*3+3,(i-1)*param.rho_opt_size+1:i*param.rho_opt_size) = dz_drho;
%     jac(i,(i-1)*param.rho_opt_size+1+param.rho_opt_size*param.num_leg:i*param.rho_opt_size+param.rho_opt_size*param.num_leg) = dz_ddrho(1,:);
end

end