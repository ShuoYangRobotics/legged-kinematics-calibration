function [est_state_list] = ekf_estimation(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param)
% most important function!

% gt_state_list just for debug and compare 

traj_len = max(size(traj_t));
dt = traj_t(3)-traj_t(2);
est_state_list = zeros(param.state_size, traj_len);
    
    
est_state_list(:,1) = state_init;
% for j=1:param.num_leg
%   est_state_list(10+j,1) = est_state_list(10+j,1) + 0.05*randn;
% %   est_state_list(16+j,1) = est_state_list(16+j,1);
% end
% random initial rho_opt
est_state_list(10+1:10+param.rho_opt_size*4,1) = param.rho_opt_init(:);
num_visual_features = max(size(visible_feature_ids));

% estimation covariance
P = 0.1*eye(param.state_size-1);
% P(7:9,7:9) = 0.0001*eye(3);
P(9+1:9+param.rho_opt_size*4,9+1:9+param.rho_opt_size*4) = 0.15*eye(param.rho_opt_size*4);

% constant parameters 
% process noise  angle, velocity, rho
Q = diag([0.000001*ones(3,1);0.000001*ones(3,1);0.000001*ones(param.rho_opt_size*4,1)]);

% measurement noise
R = 0.0000001*eye(3*max(size(fix_foot_id_list))+2*num_visual_features);
R(2*num_visual_features+1:end,2*num_visual_features+1:end) = 0.0000001*eye(3*max(size(fix_foot_id_list)));

W = zeros(param.state_size-1, param.state_size-1);
for i=2:traj_len
    % integrate state
    est_state_list(:,i) = ekf_process(est_state_list(:,i-1), meas_list(:,i-1), dt, param);
    F = ekf_process_jac(est_state_list(:,i-1), meas_list(:,i), dt, param);
    
    % map process noise to dx
    F_w = zeros(param.state_size-1, 6+param.rho_opt_size*4);
    F_w(4:6,1:3) = eye(3);
    F_w(7:9,4:6) = eye(3);
    F_w(9+(1-1)*param.rho_opt_size+1:9+1*param.rho_opt_size,6+(1-1)*param.rho_opt_size+1:6+1*param.rho_opt_size) = eye(param.rho_opt_size);
    F_w(9+(2-1)*param.rho_opt_size+1:9+2*param.rho_opt_size,6+(2-1)*param.rho_opt_size+1:6+2*param.rho_opt_size) = eye(param.rho_opt_size);
    F_w(9+(3-1)*param.rho_opt_size+1:9+3*param.rho_opt_size,6+(3-1)*param.rho_opt_size+1:6+3*param.rho_opt_size) = eye(param.rho_opt_size);
    F_w(9+(4-1)*param.rho_opt_size+1:9+4*param.rho_opt_size,6+(4-1)*param.rho_opt_size+1:6+4*param.rho_opt_size) = eye(param.rho_opt_size);
    
    P = F*P*F' + F_w*Q*F_w';  % TODO: add a noise term?
    
    cam_r = ekf_feature_residual(est_state_list(:,i), meas_list(:,i), visible_feature_ids, param);
    leg_r = ekf_leg_residual(est_state_list(:,i), meas_list(:,i), param);

    cam_r_jac = ekf_feature_residual_jac(est_state_list(:,i), meas_list(:,i), visible_feature_ids, param);
    leg_r_jac = ekf_leg_residual_jac(est_state_list(:,i), meas_list(:,i), param);

    r = [cam_r;leg_r];
    H = [cam_r_jac; leg_r_jac];
    
    % joint angle noise
    leg_r_noise = ekf_leg_noise_jac(est_state_list(:,i), meas_list(:,i), dt, param);
    R(2*num_visual_features+1:end,2*num_visual_features+1:end) = R(2*num_visual_features+1:end,2*num_visual_features+1:end) +  leg_r_noise;
    
    K = P*H'/(H*P*H'+R);

    delta_x = K*(0-r);
    P = (eye(param.state_size-1) - K*H)*P;
    est_state_list(:,i) = ekf_state_update(est_state_list(:,i), delta_x);
    % for linear invariant system this is correct
%     O=[H;H*F;H*F^2;H*F^3;H*F^4];
%     rank(O)
    % for linear variant system we need observablity gramian
    W = W + (F')^(i-2)*H'*H*(F)^(i-2);
    size(W)
    rank(W)
    
end
est_state_list(:,traj_len) = est_state_list(:,traj_len-1);

end