% generate a random robot stance
% randomly choose one leg, fix its location, then
% move the robot to a slightly different body location, keep foot location
% the same 
% need to calculate ik a bit

%% init necessary parameters
% init kinematics parameter
% kinematics_init_lc;
% init parameter
% param_init;

% assume all legs are active 
param.active_leg = [1,1,1,1];

%% generate one random stance
tgt_x = 1.8;
tgt_y = 0;
tgt_z = 0;
tgt_yaw = 0;
tgt_pitch = 0;
tgt_roll = 0;

q = quaternion([tgt_yaw tgt_pitch tgt_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(q);
pose_init = [tgt_x;tgt_y;tgt_z;w;x;y;z];
angle_measure = 0.2*randn(3*sum(param.active_leg),1) + repmat([0;1.2;-1.8],sum(param.active_leg),1);
measurement = [zeros(3,1);zeros(3,1);angle_measure;zeros(3,1);zeros(3,1)];

%% generate another stance
% % generate new stance location 
next_x = 1.9;
next_y = 0;
next_z = 0;
next_yaw = tgt_yaw+1e-5;
next_pitch = tgt_pitch+1e-3;
next_roll = tgt_roll+1e-3;

next_q = quaternion([next_yaw next_pitch next_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(next_q);
pose_next = [next_x;next_y;next_z;w;x;y;z;zeros(3,1);zeros(3,1);zeros(3,1)];
next_p_er        = pose_next(1:3);
next_q_er        = quaternion(pose_next(4:7)');

% calculate leg angles at new stance
param.num_fix_foot = 2;
fix_foot_id_list = [1;2;3;4];
disp(['fix foot '  num2str(fix_foot_id_list')])
% calculate fix foot_position
p_er        = pose_init(1:3);
q_er        = quaternion(pose_init(4:7)');
R_er = quat2rotm(q_er);
% joint angle, automatically construct theta_list according to active
% legs
theta_list = zeros(3*param.num_leg,1);
idx = 1;
for i=1:param.num_leg 
    if param.active_leg(i) == 1
        theta_list((i-1)*3+1:(i-1)*3+3) = measurement(6+(idx-1)*3+1:6+(idx-1)*3+3);
        idx = idx + 1;
    end
end
next_angles = 0.2*randn(3,param.num_leg) + repmat([0;1.2;-1.8],1,param.num_leg);

for i=1:size(fix_foot_id_list,1)
    fix_foot_id = fix_foot_id_list(i);
    theta = theta_list((fix_foot_id-1)*3+1:(fix_foot_id-1)*3+3);
    p_rf = autoFunc_fk_pf_pos(theta,[param.lc],[param.ox(fix_foot_id);param.oy(fix_foot_id);param.d(fix_foot_id);param.lt]);
    p_wf = R_er*p_rf + p_er;  % the fix foot position in world frame


    angle = get_joint_angle_from_foot_pos_and_body_pose(next_p_er, next_q_er, p_wf, fix_foot_id, theta, param);
    next_angles(:,fix_foot_id) = angle;
end

% modify the next_angle_measure term corresponding to fix_foot_id
next_angle_measure = next_angles(:,logical(param.active_leg));
next_angle_measure = next_angle_measure(:);

next_measurement = [zeros(3,1);zeros(3,1);next_angle_measure;zeros(3,1);zeros(3,1)];
%% generate features
features_init;

[num_visible_features, visible_feature_ids, feature_px_pos] = project_visible_features(pose_init, param);



%% generate trajectory between the two stance state, especially the measurements

dt = 1/200;
traj_steps = 15;   % 0.075s
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

param.state_size = 16 + param.rho_opt_size*param.num_leg;
param.meas_size = 6 + 6*param.num_leg;
% measurement also includes features 

% not fully tested 
[gt_state_list, meas_list] = get_traj(traj_t, pose_init, pose_next, angle_measure, fix_foot_id_list, visible_feature_ids, feature_px_pos, param);

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
% set(gca, 'XLim', [com_pos(1)-3.2 com_pos(1)+3.6])
% set(gca, 'YLim', [com_pos(2)-3.2 com_pos(2)+3.2])
% set(gca, 'ZLim', [com_pos(3)-2.1 com_pos(3)+2.1])
drawnow;
axis equal


fig_id = 2;
fig2 = figure(fig_id);
subplot(3,1,1)
plot(traj_t, gt_state_list(1,:));hold on;
plot(traj_t, gt_state_list(2,:));hold on;
plot(traj_t, gt_state_list(3,:));hold on;
subplot(3,1,2)
plot(traj_t, gt_state_list(8,:));hold on;
plot(traj_t, gt_state_list(9,:));hold on;
plot(traj_t, gt_state_list(10,:));hold on;
subplot(3,1,3)
plot(traj_t, meas_list(1,:));hold on;
plot(traj_t, meas_list(2,:));hold on;
plot(traj_t, meas_list(3,:));hold on;

%% calculate velocity at mid step
state = gt_state_list(:,floor(traj_steps/2));
meas =  meas_list(:,floor(traj_steps/2));
body_p = state(1:3);
body_q = quaternion(state(4:7)');
R_er = quat2rotm(body_q);
body_v = state(8:10);

joint_angle_list = meas(7:18);
joint_av_list = meas(19:30);
omega = meas(4:6);

% true calf length
r = zeros(3*param.num_leg,1);
for i = 1:param.num_leg
    angle = joint_angle_list((i-1)*3+1:(i-1)*3+3);
    av = joint_av_list((i-1)*3+1:(i-1)*3+3);
    % get opt rho TODO: check dimension here
    rho_opt = state(10+(i-1)*param.rho_opt_size+1:10+i*param.rho_opt_size,1);
    p_rf = autoFunc_fk_pf_pos(angle,rho_opt,param.rho_fix(:,i));
    J_rf = autoFunc_d_fk_dt(angle,rho_opt,param.rho_fix(:,i));
    % it seems velocity on y direction cannot be very correctly infered 
    leg_v = (-J_rf*av-skew(omega)*p_rf)
    r((i-1)*3+1:(i-1)*3+3) = body_v - R_er*leg_v
end
% wrong calf length
r = zeros(3*param.num_leg,1);
for i = 1:param.num_leg
    angle = joint_angle_list((i-1)*3+1:(i-1)*3+3);
    av = joint_av_list((i-1)*3+1:(i-1)*3+3);
    % get opt rho TODO: check dimension here
    rho_opt = state(10+(i-1)*param.rho_opt_size+1:10+i*param.rho_opt_size,1) - 0.01;
    p_rf = autoFunc_fk_pf_pos(angle,rho_opt,param.rho_fix(:,i));
    J_rf = autoFunc_d_fk_dt(angle,rho_opt,param.rho_fix(:,i));
    % it seems velocity on y direction cannot be very correctly infered 
    leg_v = (-J_rf*av-skew(omega)*p_rf)
    r((i-1)*3+1:(i-1)*3+3) = body_v - R_er*leg_v
end

