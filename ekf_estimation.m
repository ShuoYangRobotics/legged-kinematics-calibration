function [est_state_list] = ekf_estimation(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param)
% most important function!

% gt_state_list just for debug and compare 

traj_len = max(size(traj_t));
dt = traj_t(3)-traj_t(2);
est_state_list = zeros(param.state_size, traj_len);
    
    
est_state_list(:,1) = state_init;
for j=1:param.num_leg
  est_state_list(16+j,1) = est_state_list(16+j,1) + 0.05*randn;
end
num_visual_features = max(size(visible_feature_ids));

% estimation covariance
P = 0.00001*eye(param.state_size-1);
% P(7:9,7:9) = 0.0001*eye(3);
P(16:19,16:19) = 0.05*eye(4);

% constant parameters 
V = 0.001*eye(3*max(size(fix_foot_id_list))+2*num_visual_features);
V(2*num_visual_features+1:end,2*num_visual_features+1:end) = 0.00001*eye(3*max(size(fix_foot_id_list)));


for i=2:traj_len-1
    % integrate state
    est_state_list(:,i) = ekf_process(est_state_list(:,i-1), meas_list(:,i), dt, param);
    F = ekf_process_jac(est_state_list(:,i-1), meas_list(:,i), dt, param);
    
    P = F*P*F';
    
    cam_r = ekf_feature_residual(est_state_list(:,i), meas_list(:,i), visible_feature_ids, param);
    leg_r = ekf_leg_residual(est_state_list(:,i), meas_list(:,i), param);

    cam_r_jac = ekf_feature_residual_jac(est_state_list(:,i), meas_list(:,i), visible_feature_ids, param);
    leg_r_jac = ekf_leg_residual_jac(est_state_list(:,i), meas_list(:,i), param);

    r = [cam_r;leg_r];
    H = [cam_r_jac; leg_r_jac];


    K = P*H'*pinv(H*P*H'+V);

    delta_x = K*(0-r);
    P = (eye(param.state_size-1) - K*H)*P;
    est_state_list(:,i) = ekf_state_update(est_state_list(:,i), delta_x);
end
est_state_list(:,traj_len) = est_state_list(:,traj_len-1);

end