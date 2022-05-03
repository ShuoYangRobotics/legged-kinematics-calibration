function z = rho_kf_measurement(state, R_er, ...
        omega, joint_angle_list, joint_av_list, contact_weights, param)
% contact_weights should be 0000 to 1111    

z = zeros(3,1);
total_weight = sum(contact_weights);
if (total_weight > 0)
    for i = 1:param.num_leg 
        angle = [joint_angle_list((i-1)*3+1);joint_angle_list((i-1)*3+2);joint_angle_list((i-1)*3+3)];
        av = [joint_av_list((i-1)*3+1);joint_av_list((i-1)*3+2);joint_av_list((i-1)*3+3)];
        
        rho = state((i-1)*param.rho_opt_size+1:(i-1)*param.rho_opt_size+param.rho_opt_size);
        p_rf = autoFunc_fk_pf_pos(angle,rho,param.rho_fix(:,i));
        J_rf = autoFunc_d_fk_dt(angle,rho,param.rho_fix(:,i));
        
        leg_v = -R_er*(J_rf*av+skew(omega)*p_rf);
    
        z = z + contact_weights(i)*leg_v;    
    end
    z = z / total_weight;
end
    
end