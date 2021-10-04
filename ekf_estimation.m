function [est_state_list] = ekf_estimation(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, param)
% most important function!


traj_len = max(size(traj_t));
est_state_list = zeros(param.state_size, traj_len);

est_state_list(:,1) = state_init;

% estimation covariance
P = zeros(param.state_size, param.state_size);



end