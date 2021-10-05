function jac = ekf_feature_residual_jac(state, meas, visible_feature_ids, param)
% the jacobian of ekf_feature_residual
% for feature residual, 
% state contains the pose of the robot 
% meas contains the projected feature locations of the robot
p_er = state(1:3);
q_er = quaternion(state(4:7)');
R_er = quat2rotm(q_er);
num_visual_features = max(size(visible_feature_ids));

jac = zeros(2*num_visual_features, param.state_size-1);

for i = 1:num_visual_features
    id = visible_feature_ids(i);
    features_w = [param.feature_x(id);param.feature_y(id);param.feature_z(id)];
    feature_r = R_er'*(features_w - p_er);
    feature_c = param.R_rc'*(feature_r - param.p_rc);   
    % 2x3 camera projection jacobian 
    Jac_cam_proj = 1/feature_c(3)*[1 0 -feature_c(1)/feature_c(3);
                                   0 1 -feature_c(2)/feature_c(3)];
    % first follows
    % https://intra.ece.ucr.edu/~mourikis/papers/Li2013IJRR.pdf eqn 41 42 not working very well
    % then derive it again following https://arxiv.org/pdf/1711.02508.pdf eqn 188
    jac((i-1)*2+1:(i-1)*2+2,:) =  -Jac_cam_proj*param.R_rc'*[-R_er' skew(R_er'*(features_w - p_er)) zeros(3, 3+param.rho_opt_size*param.num_leg)];                  
end


end