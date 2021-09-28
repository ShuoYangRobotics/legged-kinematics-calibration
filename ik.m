function out_theta = ik(tgt_p,init_theta,rho_opt, rho_fix)

error = 999;
theta = init_theta;
itr = 0;
while (norm(error) > 1e-5) && itr < 500
    itr = itr + 1;
    p_rf = autoFunc_fk_pf_pos(theta,rho_opt, rho_fix);
    error = tgt_p - p_rf;
    J = autoFunc_d_fk_dt(theta,rho_opt, rho_fix);
    theta = theta + 0.1*inv(J'*J)*J'*error;
end
out_theta = theta;
end