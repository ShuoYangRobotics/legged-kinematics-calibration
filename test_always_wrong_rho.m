%% init necessary parameters
kinematics_init_lc;
param_init;

%% generate one random stance
tgt_x = 1.8;
tgt_y = 0;
tgt_z = 0;
tgt_yaw = 0*randn;
tgt_pitch = -5;
tgt_roll = 0*randn;

q = quaternion([tgt_yaw tgt_pitch tgt_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(q);
pose_init = [tgt_x;tgt_y;tgt_z;w;x;y;z];
angle_init = repmat([0;1.2;-1.8],sum(param.active_leg),1);

%% generate features
features_init;
[num_visible_features, visible_feature_ids, feature_px_pos] = project_visible_features(pose_init, param);

%% generate another stance
% % generate new stance location 
next_x = tgt_x+ 0.10;
next_y = tgt_y + 0.10;
next_z = tgt_z+ 0.05;
next_yaw = tgt_yaw+ 5;
next_pitch = tgt_pitch+ 5;
next_roll = tgt_roll+ 5;

next_q = quaternion([next_yaw next_pitch next_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(next_q);
pose_next = [next_x;next_y;next_z;w;x;y;z];

%% generate trajectory between the two stance state, especially the measurements

fix_foot_id_list = [1;2;3;4];
disp(['fix foot '  num2str(fix_foot_id_list')])

dt = 1/400;
traj_steps = 40;
T = dt*(traj_steps-1);
traj_t = 0:dt:T;

param.state_size = 10 + param.rho_opt_size*param.num_leg;
param.meas_size = 6 + 6*param.num_leg;
param.with_noise =0;
% tested 
[gt_state_list, meas_list] = get_traj(traj_t, pose_init, pose_next, angle_init, fix_foot_id_list, visible_feature_ids, feature_px_pos, param);


%% start to use EKF to estimate state 
state_init = gt_state_list(:,1);
[est_state_list1] = ekf_estimation_leg_true(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param);

[est_state_list2] = ekf_estimation_leg_wrong(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param);

compare_ekf_true_and_wrong(est_state_list1, est_state_list2, gt_state_list, traj_t, 'compare', param)

% %% save calf length estimation result 
% figure('Name','sim_calibrate',...
%     'Units','inches',...
% 'Position',[0 0 3.5 2],...
% 'PaperPositionMode','auto')
% clf;
% line_width = 3;
% axis_font_size = 10
% title_font_size = 6
% lgd_font_size = 10
% ylim_low = 0.19;
% ylim_high = 0.22;
% tiledlayout(2,2, 'Padding', 'none', 'TileSpacing', 'compact');
% nexttile;
% plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx),'k--','LineWidth',line_width); hold on;
% % draw 3sigma variance 1 
% plot(traj_t(plot_start:plot_end_idx), ...
%         est_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx) + ...
%         3*est_variance_list(9+(1-1)*param.rho_opt_size+1:9+1*param.rho_opt_size,plot_start:plot_end_idx),'r-.',...
%     traj_t(plot_start:plot_end_idx), ...
%         est_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx) - ...
%             3*est_variance_list(9+(1-1)*param.rho_opt_size+1:9+1*param.rho_opt_size,plot_start:plot_end_idx),'r-.','LineWidth',line_width/1.5)
% 
% ylim([ylim_low ylim_high])
% set(gca,'FontSize',axis_font_size,'YTick',0.15:0.01:0.25,'XTick',0:0.05:0.1)
% title(strcat('Leg 1 (Front-left)'), 'FontName','Times','interpreter', 'latex', 'FontSize',title_font_size);
% % legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% % lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
% %              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% % lgd.FontSize = lgd_font_size;
% 
% nexttile;
% plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx),'k--','LineWidth',line_width); hold on;
% % draw 3sigma variance 2 
% plot(traj_t(plot_start:plot_end_idx), ...
%         est_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx) + ...
%         3*est_variance_list(9+(2-1)*param.rho_opt_size+1:9+2*param.rho_opt_size,plot_start:plot_end_idx),'r-.',...
%     traj_t(plot_start:plot_end_idx), ...
%         est_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx) - ...
%             3*est_variance_list(9+(2-1)*param.rho_opt_size+1:9+2*param.rho_opt_size,plot_start:plot_end_idx),'r-.','LineWidth',line_width/1.5)
% 
% ylim([ylim_low ylim_high])
% set(gca,'FontSize',axis_font_size,'YTick',0.15:0.01:0.25,'XTick',0:0.05:0.1)
% title(strcat('Leg 2 (Front-right)'), 'FontName','Times','interpreter', 'latex', 'FontSize',title_font_size);
% % legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% % lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
% %              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% % lgd.FontSize = lgd_font_size;
% 
% nexttile;
% plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx),'k--','LineWidth',line_width); hold on;
% % draw 3sigma variance 3 
% plot(traj_t(plot_start:plot_end_idx), ...
%         est_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx) + ...
%         3*est_variance_list(9+(3-1)*param.rho_opt_size+1:9+3*param.rho_opt_size,plot_start:plot_end_idx),'r-.',...
%     traj_t(plot_start:plot_end_idx), ...
%         est_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx) - ...
%             3*est_variance_list(9+(3-1)*param.rho_opt_size+1:9+3*param.rho_opt_size,plot_start:plot_end_idx),'r-.','LineWidth',line_width/1.5)
% 
% ylim([ylim_low ylim_high])
% set(gca,'FontSize',axis_font_size,'YTick',0.15:0.01:0.25,'XTick',0:0.05:0.1)
% title(strcat('Leg 3 (Rear-left)'), 'FontName','Times','interpreter', 'latex', 'FontSize',title_font_size);
% % legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% % lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
% %              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% % lgd.FontSize = lgd_font_size;
% xlabel({'Time (s)'},...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'interpreter','latex',...
% 'FontSize',axis_font_size,...
% 'FontName','Times')
% 
% nexttile;
% plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx),'k--','LineWidth',line_width); hold on;
% % draw 3sigma variance 4 
% plot(traj_t(plot_start:plot_end_idx), ...
%         est_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx) + ...
%         3*est_variance_list(9+(4-1)*param.rho_opt_size+1:9+4*param.rho_opt_size,plot_start:plot_end_idx),'r-.',...
%     traj_t(plot_start:plot_end_idx), ...
%         est_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx) - ...
%             3*est_variance_list(9+(4-1)*param.rho_opt_size+1:9+4*param.rho_opt_size,plot_start:plot_end_idx),'r-.','LineWidth',line_width/1.5)
% 
% ylim([ylim_low ylim_high])
% set(gca,'FontSize',axis_font_size,'YTick',0.15:0.01:0.25,'XTick',0:0.05:0.1)
% title(strcat('Leg 4 (Rear-right)'),'FontName','Times', 'interpreter', 'latex', 'FontSize',title_font_size);
% xlabel({'Time (s)'},...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'interpreter','latex',...
% 'FontSize',axis_font_size,...
% 'FontName','Times')
% % legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% % lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
% %              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% % lgd.FontSize = lgd_font_size;
% 
% matlab2tikz('sim_ekf_param_output.tex',...
%             'extraAxisOptions',{'ylabel near ticks','xlabel near ticks',...
%             'scaled ticks=false',...
%             'stack plots=false' })