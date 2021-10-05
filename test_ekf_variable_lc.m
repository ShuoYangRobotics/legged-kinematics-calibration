%% init necessary parameters
% init kinematics parameter
% kinematics_init;
% init parameter
% param_init;

% assume all legs are active 
param.active_leg = [1,1,1,1];

%% generate one random stance
tgt_x = 1.8;
tgt_y = 0;
tgt_z = 0;
tgt_yaw = 10*randn;
tgt_pitch = 5*randn;
tgt_roll = 5*randn;

q = quaternion([tgt_yaw tgt_pitch tgt_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(q);
pose_init = [tgt_x;tgt_y;tgt_z;w;x;y;z];
angle_init = 0.2*randn(3*sum(param.active_leg),1) + repmat([0;1.2;-1.8],sum(param.active_leg),1);

%% generate features
features_init;
[num_visible_features, visible_feature_ids, feature_px_pos] = project_visible_features(pose_init, param);

%% generate another stance
% % generate new stance location 
next_x = tgt_x + 0.07+0.03*(2*randn-1);
next_y = tgt_y + 0.02+0.01*(2*randn-1);
next_z = tgt_z + 0.01+0.02*(2*randn-1);
next_yaw = tgt_yaw+3;
next_pitch = tgt_pitch;
next_roll = tgt_roll;

next_q = quaternion([next_yaw next_pitch next_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(next_q);
pose_next = [next_x;next_y;next_z;w;x;y;z];

%% generate trajectory between the two stance state, especially the measurements

fix_foot_id_list = [1;2;3;4];
disp(['fix foot '  num2str(fix_foot_id_list')])

dt = 1/200;
traj_steps = 66;
T = dt*(traj_steps-1);
traj_t = 0:dt:T;

% assume all features available at state_init are also avaiable on the
% entire trajectory so we do not need to care about feature association

% state: 
%      1 2 3     4 5 6 7       8 9 10       n_rho   n_rho     n_rho     n_rho
%    position  quaternion     velocity    rho_opt1 rho_opt2 rho_opt3   rho_opt4
% measurement
%    1 2 3    4 5 6    7 8 9   10 11 12   13 14 15   16 17 18    19 20 21   22 23 24   25 26 27   28 29 30
%     acc     omega    angle1   angle2     angle3    angle4         av1        av2        av3        av4
%  fix_leg_ids can be viewed as measurement too

param.state_size = 10 + param.rho_opt_size*param.num_leg;
param.meas_size = 6 + 6*param.num_leg;

% not fully tested 
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
[est_state_list] = ekf_estimation(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param);

% compare est_state_list with gt_state_list;
[est_state_list(:,1) est_state_list(:,end) gt_state_list(:,end) gt_state_list(:,1)]
figure(3)
subplot(5,2,1)
plot(traj_t, est_state_list(1,:),traj_t, gt_state_list(1,:))
title('Vision + Leg - Position X')
legend('Estimation', 'Ground Truth');
subplot(5,2,2)
plot(traj_t, est_state_list(2,:),traj_t, gt_state_list(2,:))
title('Vision + Leg - Position Y')
legend('Estimation', 'Ground Truth');
subplot(5,2,3)
plot(traj_t, est_state_list(3,:),traj_t, gt_state_list(3,:))
title('Vision + Leg - Position Z')
legend('Estimation', 'Ground Truth');
subplot(5,2,4)
plot(traj_t, est_state_list(8,:),traj_t, gt_state_list(8,:))
title('Vision + Leg - Velocity X')
legend('Estimation', 'Ground Truth');
subplot(5,2,5)
plot(traj_t, est_state_list(9,:),traj_t, gt_state_list(9,:))
title('Vision + Leg - Velocity Y')
legend('Estimation', 'Ground Truth');
subplot(5,2,6)
plot(traj_t, est_state_list(10,:),traj_t, gt_state_list(10,:))
title('Vision + Leg - Velocity Z')
legend('Estimation', 'Ground Truth');
subplot(5,2,7)
plot(traj_t, est_state_list(11,:),traj_t, gt_state_list(11,:))
title('Vision + Leg - FL lc')
legend('Estimation', 'Ground Truth');
subplot(5,2,8)
plot(traj_t, est_state_list(12,:),traj_t, gt_state_list(12,:))
title('Vision + Leg - FR lc')
legend('Estimation', 'Ground Truth');
subplot(5,2,9)
plot(traj_t, est_state_list(13,:),traj_t, gt_state_list(13,:))
title('Vision + Leg - RL lc')
legend('Estimation', 'Ground Truth');
subplot(5,2,10)
plot(traj_t, est_state_list(14,:),traj_t, gt_state_list(14,:))
title('Vision + Leg - RR lc')
legend('Estimation', 'Ground Truth');

%% EKF vision only 
state_init = gt_state_list(:,1);
[est_state_list] = ekf_estimation_vision_only(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param);

% compare est_state_list with gt_state_list;
[est_state_list(:,1) est_state_list(:,end) gt_state_list(:,end) gt_state_list(:,1)]
figure(4)
subplot(5,2,1)
plot(traj_t, est_state_list(1,:),traj_t, gt_state_list(1,:))
title('Vison Only - Position X')
legend('Estimation', 'Ground Truth');
subplot(5,2,2)
plot(traj_t, est_state_list(2,:),traj_t, gt_state_list(2,:))
title('Vison Only - Position Y')
legend('Estimation', 'Ground Truth');
subplot(5,2,3)
plot(traj_t, est_state_list(3,:),traj_t, gt_state_list(3,:))
title('Vison Only - Position Z')
legend('Estimation', 'Ground Truth');
subplot(5,2,4)
plot(traj_t, est_state_list(8,:),traj_t, gt_state_list(8,:))
title('Vison Only - Velocity X')
legend('Estimation', 'Ground Truth');
subplot(5,2,5)
plot(traj_t, est_state_list(9,:),traj_t, gt_state_list(9,:))
title('Vison Only - Velocity Y')
legend('Estimation', 'Ground Truth');
subplot(5,2,6)
plot(traj_t, est_state_list(10,:),traj_t, gt_state_list(10,:))
title('Vison Only - Velocity Z')
legend('Estimation', 'Ground Truth');
subplot(5,2,7)
plot(traj_t, est_state_list(11,:),traj_t, gt_state_list(11,:))
title('Vison Only - FL lc')
legend('Estimation', 'Ground Truth');
subplot(5,2,8)
plot(traj_t, est_state_list(12,:),traj_t, gt_state_list(12,:))
title('Vison Only - FR lc')
legend('Estimation', 'Ground Truth');
subplot(5,2,9)
plot(traj_t, est_state_list(13,:),traj_t, gt_state_list(13,:))
title('Vison Only - RL lc')
legend('Estimation', 'Ground Truth');
subplot(5,2,10)
plot(traj_t, est_state_list(14,:),traj_t, gt_state_list(14,:))
title('Vison Only - RR lc')
legend('Estimation', 'Ground Truth');

%% EKF leg only 
state_init = gt_state_list(:,1);
[est_state_list] = ekf_estimation_leg_only(traj_t, state_init, meas_list, fix_foot_id_list, visible_feature_ids, gt_state_list, param);

% compare est_state_list with gt_state_list;
[est_state_list(:,1) est_state_list(:,end) gt_state_list(:,end) gt_state_list(:,1)]
figure(5)
subplot(5,2,1)
plot(traj_t, est_state_list(1,:),traj_t, gt_state_list(1,:))
title('Leg Only - Position X')
legend('Estimation', 'Ground Truth');
subplot(5,2,2)
plot(traj_t, est_state_list(2,:),traj_t, gt_state_list(2,:))
title('Leg Only - Position Y')
legend('Estimation', 'Ground Truth');
subplot(5,2,3)
plot(traj_t, est_state_list(3,:),traj_t, gt_state_list(3,:))
title('Leg Only - Position Z')
legend('Estimation', 'Ground Truth');
subplot(5,2,4)
plot(traj_t, est_state_list(8,:),traj_t, gt_state_list(8,:))
title('Leg Only - Velocity X')
legend('Estimation', 'Ground Truth');
subplot(5,2,5)
plot(traj_t, est_state_list(9,:),traj_t, gt_state_list(9,:))
title('Leg Only - Velocity Y')
legend('Estimation', 'Ground Truth');
subplot(5,2,6)
plot(traj_t, est_state_list(10,:),traj_t, gt_state_list(10,:))
title('Leg Only - Velocity Z')
legend('Estimation', 'Ground Truth');
subplot(5,2,7)
plot(traj_t, est_state_list(11,:),traj_t, gt_state_list(11,:))
title('Leg Only - FL lc')
legend('Estimation', 'Ground Truth');
subplot(5,2,8)
plot(traj_t, est_state_list(12,:),traj_t, gt_state_list(12,:))
title('Leg Only - FR lc')
legend('Estimation', 'Ground Truth');
subplot(5,2,9)
plot(traj_t, est_state_list(13,:),traj_t, gt_state_list(13,:))
title('Leg Only - RL lc')
legend('Estimation', 'Ground Truth');
subplot(5,2,10)
plot(traj_t, est_state_list(14,:),traj_t, gt_state_list(14,:))
title('Leg Only - RR lc')
legend('Estimation', 'Ground Truth');
