function [est_state_list] = ekf_estimation(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param)
% most important function!

% gt_state_list just for debug and compare 

traj_len = max(size(traj_t));
dt = traj_t(3)-traj_t(2);
est_state_list = zeros(param.state_size, traj_len);
    
    
est_state_list(:,1) = state_init;
for j=1:param.num_leg
%   est_state_list(16+j,1) = est_state_list(16+j,1) + 0.05*randn;
  est_state_list(16+j,1) = est_state_list(16+j,1) + 0.05*randn;
end
num_visual_features = max(size(visible_feature_ids));

% estimation covariance
P = 0.00001*eye(param.state_size-1);
% P(7:9,7:9) = 0.0001*eye(3);
P(16:19,16:19) = 0.005*eye(4);

% constant parameters 
% process noise  angle, velocity, rho
Q = diag([0.001*ones(3,1);0.001*ones(3,1);0.5*ones(1,1)]);

% measurement noise
R = 0.001*eye(3*max(size(fix_foot_id_list))+2*num_visual_features);
R(2*num_visual_features+1:end,2*num_visual_features+1:end) = 0.00001*eye(3*max(size(fix_foot_id_list)));


for i=2:traj_len-1
    % integrate state
    est_state_list(:,i) = ekf_process(est_state_list(:,i-1), meas_list(:,i), dt, param);
    F = ekf_process_jac(est_state_list(:,i-1), meas_list(:,i), dt, param);
    
    % map process noise to dx
    F_w = zeros(param.state_size-1, 7);
    F_w(4:6,1:3) = eye(3);
    F_w(7:9,4:6) = eye(3);
    F_w(16,7) = 1;
    F_w(17,7) = 1;
    F_w(18,7) = 1;
    F_w(19,7) = 1;
    
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
    
    K = P*H'*pinv(H*P*H'+R);

    delta_x = K*(0-r);
    P = (eye(param.state_size-1) - K*H)*P;
    est_state_list(:,i) = ekf_state_update(est_state_list(:,i), delta_x);
end
est_state_list(:,traj_len) = est_state_list(:,traj_len-1);

end