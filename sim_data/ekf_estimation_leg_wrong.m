function [est_state_list] = ekf_estimation_leg_wrong(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param)
traj_len = max(size(traj_t));
dt = traj_t(3)-traj_t(2);
est_state_list = zeros(param.state_size, traj_len);
    
    
est_state_list(:,1) = state_init;
est_state_list(10+1:10+param.rho_opt_size*4,1) = param.rho_opt_true(:)+0.01;

num_visual_features = max(size(visible_feature_ids));

% estimation covariance
P = 0.05*eye(param.state_size-1);
% P(7:9,7:9) = 0.0001*eye(3);
P(10:13,10:13) = 0.00*eye(4);

% constant parameters 
% process noise  angle, velocity, rho
Q = diag([0.001*ones(3,1);0.001*ones(3,1);0.00*ones(4,1)]);

% measurement noise
R = 0.001*eye(3*max(size(fix_foot_id_list)));
R(1:end,1:end) = 0.0001*eye(3*max(size(fix_foot_id_list)));

W = zeros(param.state_size-1, param.state_size-1);

for i=2:traj_len-1
    % integrate state
    est_state_list(:,i) = ekf_process(est_state_list(:,i-1), meas_list(:,i), dt, param);
    F = ekf_process_jac(est_state_list(:,i-1), meas_list(:,i), dt, param);
    
    est_state_list(10+1:10+param.rho_opt_size*4,i) = param.rho_opt_true(:)+0.01;
    % map process noise to dx
    F_w = zeros(param.state_size-1, 10);
    F_w(4:6,1:3) = eye(3);
    F_w(7:9,4:6) = eye(3);
    F_w(10,7) = 1;
    F_w(11,8) = 1;
    F_w(12,9) = 1;
    F_w(13,10) = 1;
    
    P = F*P*F' + F_w*Q*F_w';  % TODO: add a noise term?
    
    cam_r = ekf_feature_residual(est_state_list(:,i), meas_list(:,i), visible_feature_ids, param);
    leg_r = ekf_leg_residual(est_state_list(:,i), meas_list(:,i), param);

    cam_r_jac = ekf_feature_residual_jac(est_state_list(:,i), meas_list(:,i), visible_feature_ids, param);
    leg_r_jac = ekf_leg_residual_jac(est_state_list(:,i), meas_list(:,i), param);

    r = [leg_r];
    H = [leg_r_jac];
    
    % joint angle noise
    leg_r_noise = ekf_leg_noise_jac(est_state_list(:,i), meas_list(:,i), dt, param);
    R = R +  leg_r_noise;
    
    K = P*H'*inv(H*P*H'+R);

    delta_x = K*(0-r);
    P = (eye(param.state_size-1) - K*H)*P;
    est_state_list(:,i) = ekf_state_update(est_state_list(:,i), delta_x);
    est_state_list(10+1:10+param.rho_opt_size*4,i) = param.rho_opt_true(:)+0.01;

    % for linear invariant system this is correct
%     O=[H;H*F;H*F^2;H*F^3;H*F^4];
%     rank(O)
    % for linear variant system we need observablity gramian
    W = W + (F')^(i-2)*H'*H*(F)^(i-2);
    rank(W)
end
est_state_list(:,traj_len) = est_state_list(:,traj_len-1);

end