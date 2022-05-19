function r = ekf_feature_residual(state, meas, visible_feature_ids, param)
% for feature residual, 
% state contains the pose of the robot 
% meas contains the projected feature locations of the robot
body_p = state(1:3);
body_q = quaternion(state(4:7)');
proj_feature_pt = project_features(body_p, body_q, visible_feature_ids, param);

r = meas(31:end)-proj_feature_pt;


end