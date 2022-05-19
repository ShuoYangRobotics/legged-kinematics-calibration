function [num_visible_features, visible_feature_ids, feature_px_pos] = project_visible_features(state, param)
% project features in param to camera frame according to current pose in
% state

% convert feature to camera frame 
features_w = [param.feature_x';param.feature_y';param.feature_z'];

p_er        = state(1:3);
q_er        = quaternion(state(4:7)');
R_er = quat2rotm(q_er);

feature_r = R_er'*(features_w - repmat(p_er,1,param.num_features));

feature_c = param.R_rc'*(feature_r - repmat(param.p_rc,1,param.num_features));

% project 
feature_pxs = [feature_c(1,:)./feature_c(3,:);feature_c(2,:)./feature_c(3,:)];

feature_px_x_inview = feature_pxs(1,:)>-param.horiz_film/2 & feature_pxs(1,:)<param.horiz_film/2;
feature_px_y_inview = feature_pxs(2,:)>-param.verti_film/2 & feature_pxs(2,:)<param.verti_film/2;
feature_px_inview = feature_px_x_inview & feature_px_y_inview;

num_visible_features = sum(feature_px_inview);
id_list = 1:param.num_features;
visible_feature_ids = id_list(feature_px_inview);

feature_px_pos = feature_pxs(:,feature_px_inview);

feature_px_pos = feature_px_pos(:);

end