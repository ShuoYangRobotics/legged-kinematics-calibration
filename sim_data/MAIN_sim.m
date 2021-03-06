%% init necessary parameters
% init kinematics parameter
% kinematics_init_ltlc;
run ../kinematics_init_lc;
run ../param_init;
% % assume all legs are active 
% param.active_leg = [1,1,1,1];

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
angle_init = 0.05*randn(3*sum(param.active_leg),1) + repmat([0;1.2;-1.8],sum(param.active_leg),1);

%% generate features
features_init;
[num_visible_features, visible_feature_ids, feature_px_pos] = project_visible_features(pose_init, param);

%% generate another stance
% % generate new stance location 
next_x = tgt_x+ 0.10 + 0.01*randn;
next_y = tgt_y+ 0.10 + 0.01*randn;
next_z = tgt_z+ 0.05;
next_yaw = tgt_yaw+ 5;
next_pitch = tgt_pitch+ 5 + 0.01*randn;
next_roll = tgt_roll+ 5+ 0.01*randn;

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

% assume all features available at state_init are also avaiable on the
% entire trajectory so we do not need to care about feature association

% state: 
%      1 2 3     4 5 6 7       8 9 10      11 12 13     14 15 16      n_rho   n_rho     n_rho     n_rho
%    position  quaternion     velocity     acc bias    gyro bias   rho_opt1 rho_opt2 rho_opt3   rho_opt4
% measurement
%    1 2 3    4 5 6    7 8 9   10 11 12   13 14 15   16 17 18    19 20 21   22 23 24   25 26 27   28 29 30
%     acc     omega    angle1   angle2     angle3    angle4         av1        av2        av3        av4
%  fix_leg_ids can be viewed as measurement too

param.state_size = 10 + param.rho_opt_size*param.num_leg;
param.meas_size = 6 + 6*param.num_leg;
param.with_noise = 1;
% tested 
[gt_state_list, meas_list] = get_traj(traj_t, pose_init, pose_next, angle_init, fix_foot_id_list, visible_feature_ids, feature_px_pos, param);

%% draw everything
fig_id = 1;
fig = figure(fig_id);
% set(gcf, 'units','normalized','outerposition',[0 0.2 0.3 0.7]);

clf; hold on;
draw_robot(fig, gt_state_list(1:7,1), meas_list(7:18,1), param, 1);
draw_robot(fig, gt_state_list(1:7,end), meas_list(7:18,end), param, 0.8);
% draw feature
plot3(param.feature_x, param.feature_y, param.feature_z, 'g*');
plot3(param.feature_x(visible_feature_ids), param.feature_y(visible_feature_ids), param.feature_z(visible_feature_ids), 'r*');

% adjust view point of the figure 
com_pos = pose_init(1:3);
set(gca,'CameraPosition',[com_pos(1)+5 com_pos(2)+5 com_pos(3)+2]);
set(gca,'CameraTarget',[com_pos(1) com_pos(2) com_pos(3)]);
set(gca,'CameraUpVector',[0 0 1]);
set(gca,'CameraViewAngle',8.6765);
set(gca, 'XLim', [com_pos(1)-3.2 com_pos(1)+3.6])
set(gca, 'YLim', [com_pos(2)-3.2 com_pos(2)+3.2])
set(gca, 'ZLim', [com_pos(3)-2.1 com_pos(3)+2.1])
drawnow;

% most important part!
%% test measurement models
% test measurement models
test_id = 10;
% camera jacobian
cam_r = ekf_feature_residual(gt_state_list(:,test_id), meas_list(:,test_id), visible_feature_ids, param);
cam_r_jac = ekf_feature_residual_jac(gt_state_list(:,test_id), meas_list(:,test_id), visible_feature_ids, param);

% leg jacobian
leg_r = ekf_leg_residual(gt_state_list(:,test_id), meas_list(:,test_id), param);
leg_r_jac = ekf_leg_residual_jac(gt_state_list(:,test_id), meas_list(:,test_id), param);
leg_r

delta_x = zeros(param.state_size-1,1);
delta_x(1:6) = 0.01*(2*randn(6,1)-1);
new_state=ekf_state_update(gt_state_list(:,test_id), delta_x);
new_r = ekf_feature_residual(new_state, meas_list(:,test_id), visible_feature_ids, param);
new_diff_r = cam_r + cam_r_jac*delta_x;
[gt_state_list(:,test_id) new_state]
[new_r new_diff_r]
sum(new_r-new_diff_r)/max(size(new_diff_r))  % still large, may need to further check. Or maybe the camera projection is too nonlinear by nature

delta_x = 0.01*(2*randn(param.state_size-1,1)-1);
new_state=ekf_state_update(gt_state_list(:,test_id), delta_x);
new_leg_r = ekf_leg_residual(new_state, meas_list(:,test_id), param);
new_diff_leg_r = leg_r + leg_r_jac*delta_x;
[new_leg_r new_diff_leg_r]
sum(new_leg_r-new_diff_leg_r)/max(size(new_diff_leg_r))  % pass

next_state = ekf_process(gt_state_list(:,test_id), meas_list(:,test_id), dt, param);
next_state_jac = ekf_process_jac(gt_state_list(:,test_id), meas_list(:,test_id), dt, param);
% [gt_state_list(:,test_id) gt_state_list(:,test_id+1) next_state]
new_next_state = ekf_process(new_state, meas_list(:,test_id), dt, param);
new_diff_next_state = ekf_state_update(next_state,next_state_jac*delta_x);
sum(new_next_state-new_diff_next_state)/max(size(new_diff_next_state))                     % pass

%% start to use EKF to estimate state 
state_init = gt_state_list(:,1);
[est_state_list, est_variance_list] = ekf_estimation(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param);
plot_start = 1
plot_end_idx = size(traj_t,2)-1
%
% compare est_state_list with gt_state_list;
[est_state_list(:,1) est_state_list(:,end) gt_state_list(:,1) gt_state_list(:,end) ]
% figure(3)
% line_width = 4;
% axis_font_size = 19
% title_font_size = 19
% lgd_font_size = 10
% tiledlayout(7,2, 'Padding', 'none', 'TileSpacing', 'compact'); 
% nexttile
% plot(traj_t(plot_start:plot_end_idx), est_state_list(1,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(1,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% set(gca,'FontSize',axis_font_size)
% title('Position X', 'interpreter', 'latex', 'FontSize',title_font_size)
% % lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% % lgd.FontSize = lgd_font_size;
% nexttile
% plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list(8,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(8,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% set(gca,'FontSize',axis_font_size)
% title('Velocity X', 'interpreter', 'latex', 'FontSize',title_font_size)
% % lgd = legend('Estimation', 'Ground Truth', 'interpreter', 'latex');
% % lgd.FontSize = lgd_font_size;
% nexttile
% plot(traj_t(plot_start:plot_end_idx), est_state_list(2,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(2,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% set(gca,'FontSize',axis_font_size)
% title('Position Y', 'interpreter', 'latex', 'FontSize',title_font_size)
% % lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% % lgd.FontSize = lgd_font_size;
% nexttile
% plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list(9,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(9,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% set(gca,'FontSize',axis_font_size)
% title('Velocity Y', 'interpreter', 'latex', 'FontSize',title_font_size)
% % lgd = legend('Estimation', 'Ground Truth', 'interpreter', 'latex');
% % lgd.FontSize = lgd_font_size;
% nexttile
% plot(traj_t(plot_start:plot_end_idx), est_state_list(3,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(3,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% set(gca,'FontSize',axis_font_size)
% title('Position Z', 'interpreter', 'latex', 'FontSize',title_font_size)
% % lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% % lgd.FontSize = lgd_font_size;
% nexttile
% plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list(10,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(10,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% set(gca,'FontSize',axis_font_size)
% title('Velocity Z', 'interpreter', 'latex', 'FontSize',title_font_size)
% % lgd = legend('Estimation', 'Ground Truth', 'interpreter', 'latex');
% % lgd.FontSize = lgd_font_size;
% nexttile([2 1]);
% plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% ylim([0.1 0.3])
% set(gca,'FontSize',axis_font_size)
% title(strcat('Front-left Leg', {' '}, {'$\rho$ = ['}, param.rho_opt_str, {'] '}), 'interpreter', 'latex', 'FontSize',title_font_size);
% % legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% % lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
% %              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% % lgd.FontSize = lgd_font_size;
% 
% nexttile([2 1]);
% plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% ylim([0.1 0.3])
% set(gca,'FontSize',axis_font_size)
% title(strcat('Front-right Leg', {' '}, {'$\rho$ = ['}, param.rho_opt_str, {'] '}), 'interpreter', 'latex', 'FontSize',title_font_size);
% % legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% % lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
% %              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% % lgd.FontSize = lgd_font_size;
% 
% nexttile([2 1]);
% plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% ylim([0.1 0.3])
% set(gca,'FontSize',axis_font_size)
% title(strcat('Rear-left Leg', {' '}, {'$\rho$ = ['}, param.rho_opt_str, {'] '}), 'interpreter', 'latex', 'FontSize',title_font_size);
% % legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% % lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
% %              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% % lgd.FontSize = lgd_font_size;
% 
% nexttile([2 1]);
% plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx),...
%     traj_t(plot_start:plot_end_idx), gt_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx),'-.','LineWidth',line_width)
% ylim([0.1 0.3])
% set(gca,'FontSize',axis_font_size)
% title(strcat('Rear-right Leg', {' '}, {'$\rho$ = ['}, param.rho_opt_str, {'] '}), 'interpreter', 'latex', 'FontSize',title_font_size);
% % legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% % lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
% %              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% % lgd.FontSize = lgd_font_size;

%% save calf length estimation result 
figure('Name','sim_calibrate',...
    'Units','inches',...
'Position',[0 0 3.5 2],...
'PaperPositionMode','auto')
clf;
line_width = 3;
axis_font_size = 10
title_font_size = 6
lgd_font_size = 10
ylim_low = 0.19;
ylim_high = 0.22;
tiledlayout(2,2, 'Padding', 'none', 'TileSpacing', 'compact');
nexttile;
plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx),'k--','LineWidth',line_width); hold on;
% draw 3sigma variance 1 
plot(traj_t(plot_start:plot_end_idx), ...
        est_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx) + ...
        3*est_variance_list(9+(1-1)*param.rho_opt_size+1:9+1*param.rho_opt_size,plot_start:plot_end_idx),'r-.',...
    traj_t(plot_start:plot_end_idx), ...
        est_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx) - ...
            3*est_variance_list(9+(1-1)*param.rho_opt_size+1:9+1*param.rho_opt_size,plot_start:plot_end_idx),'r-.','LineWidth',line_width/1.5)

ylim([ylim_low ylim_high])
set(gca,'FontSize',axis_font_size,'YTick',0.15:0.01:0.25,'XTick',0:0.05:0.1)
title(strcat('Leg 1 (Front-left)'), 'FontName','Times','interpreter', 'latex', 'FontSize',title_font_size);
% legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
%              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% lgd.FontSize = lgd_font_size;

nexttile;
plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx),'k--','LineWidth',line_width); hold on;
% draw 3sigma variance 2 
plot(traj_t(plot_start:plot_end_idx), ...
        est_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx) + ...
        3*est_variance_list(9+(2-1)*param.rho_opt_size+1:9+2*param.rho_opt_size,plot_start:plot_end_idx),'r-.',...
    traj_t(plot_start:plot_end_idx), ...
        est_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx) - ...
            3*est_variance_list(9+(2-1)*param.rho_opt_size+1:9+2*param.rho_opt_size,plot_start:plot_end_idx),'r-.','LineWidth',line_width/1.5)

ylim([ylim_low ylim_high])
set(gca,'FontSize',axis_font_size,'YTick',0.15:0.01:0.25,'XTick',0:0.05:0.1)
title(strcat('Leg 2 (Front-right)'), 'FontName','Times','interpreter', 'latex', 'FontSize',title_font_size);
% legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
%              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% lgd.FontSize = lgd_font_size;

nexttile;
plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx),'k--','LineWidth',line_width); hold on;
% draw 3sigma variance 3 
plot(traj_t(plot_start:plot_end_idx), ...
        est_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx) + ...
        3*est_variance_list(9+(3-1)*param.rho_opt_size+1:9+3*param.rho_opt_size,plot_start:plot_end_idx),'r-.',...
    traj_t(plot_start:plot_end_idx), ...
        est_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx) - ...
            3*est_variance_list(9+(3-1)*param.rho_opt_size+1:9+3*param.rho_opt_size,plot_start:plot_end_idx),'r-.','LineWidth',line_width/1.5)

ylim([ylim_low ylim_high])
set(gca,'FontSize',axis_font_size,'YTick',0.15:0.01:0.25,'XTick',0:0.05:0.1)
title(strcat('Leg 3 (Rear-left)'), 'FontName','Times','interpreter', 'latex', 'FontSize',title_font_size);
% legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
%              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% lgd.FontSize = lgd_font_size;
xlabel({'Time (s)'},...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',axis_font_size,...
'FontName','Times')

nexttile;
plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx),'k--','LineWidth',line_width); hold on;
% draw 3sigma variance 4 
plot(traj_t(plot_start:plot_end_idx), ...
        est_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx) + ...
        3*est_variance_list(9+(4-1)*param.rho_opt_size+1:9+4*param.rho_opt_size,plot_start:plot_end_idx),'r-.',...
    traj_t(plot_start:plot_end_idx), ...
        est_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx) - ...
            3*est_variance_list(9+(4-1)*param.rho_opt_size+1:9+4*param.rho_opt_size,plot_start:plot_end_idx),'r-.','LineWidth',line_width/1.5)

ylim([ylim_low ylim_high])
set(gca,'FontSize',axis_font_size,'YTick',0.15:0.01:0.25,'XTick',0:0.05:0.1)
title(strcat('Leg 4 (Rear-right)'),'FontName','Times', 'interpreter', 'latex', 'FontSize',title_font_size);
xlabel({'Time (s)'},...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',axis_font_size,...
'FontName','Times')
% legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
%              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% lgd.FontSize = lgd_font_size;

% matlab2tikz('sim_ekf_param_output.tex',...
%             'extraAxisOptions',{'ylabel near ticks','xlabel near ticks',...
%             'scaled ticks=false',...
%             'stack plots=false' })