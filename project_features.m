function [feature_px_pos] = project_features(body_p, body_q, visible_feature_ids, param)
% project features(visible_feature_ids) to current camera frame
% we do not care whether features are in the FOV or not
% body_q must be quaternion
% convert feature to camera frame 
num_visual_features = max(size(visible_feature_ids));
features_w = [param.feature_x(visible_feature_ids)';param.feature_y(visible_feature_ids)';param.feature_z(visible_feature_ids)'];

p_er        = body_p(1:3);
q_er        = body_q;
R_er = quat2rotm(q_er);

feature_r = R_er'*(features_w - repmat(p_er,1,num_visual_features));

feature_c = param.R_rc'*(feature_r - repmat(param.p_rc,1,num_visual_features));

% project 
feature_px_pos = [feature_c(1,:)./feature_c(3,:);feature_c(2,:)./feature_c(3,:)];
feature_px_pos = feature_px_pos(:);
end