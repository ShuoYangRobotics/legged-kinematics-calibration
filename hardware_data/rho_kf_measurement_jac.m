function jac = rho_kf_measurement_jac(state, R_er, ...
        omega, joint_angle_list, joint_av_list, contact_weights, param)

jac = zeros(3, param.rho_opt_size*param.num_leg);    
total_weight = sum(contact_weights);
    
if (total_weight > 0)
    for i = 1:param.num_leg
        angle = [joint_angle_list((i-1)*3+1);joint_angle_list((i-1)*3+2);joint_angle_list((i-1)*3+3)];
        av = [joint_av_list((i-1)*3+1);joint_av_list((i-1)*3+2);joint_av_list((i-1)*3+3)];
       
        rho = state((i-1)*param.rho_opt_size+1:(i-1)*param.rho_opt_size+param.rho_opt_size);
        dJdrho = autoFunc_dJ_drho(angle,rho,param.rho_fix(:,i));
        dfdrho = autoFunc_d_fk_drho(angle,rho,param.rho_fix(:,i));
        dz_drho = -R_er*(kron(av',eye(3))*dJdrho+skew(omega)*dfdrho);
    
        jac(1:3,(i-1)*param.rho_opt_size+1:i*param.rho_opt_size) = contact_weights(i)/total_weight*dz_drho;
      end
end

end