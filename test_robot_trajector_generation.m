% generate a random robot stance
% randomly choose one leg, fix its location, then
% move the robot to a slightly different body location, keep foot location
% the same 
% need to calculate ik a bit

%% init necessary parameters
% init kinematics parameter
% kinematics_init;
% init parameter
% param_init;

% adjust number of active legs 
param.active_leg = [0,1,0,0];

%% generate one random stance
tgt_x = 1.8;
tgt_y = 0;
tgt_z = 0;
tgt_yaw = 10*randn;
tgt_pitch = 5*randn;
tgt_roll = 5*randn;

q = quaternion([tgt_yaw tgt_pitch tgt_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(q);
state_init = [tgt_x;tgt_y;tgt_z;w;x;y;z;zeros(3,1);zeros(3,1);zeros(3,1)];
angle_measure = 0.2*randn(3*sum(param.active_leg),1) + repmat([0;1.1;-0.7],sum(param.active_leg),1);

measurement = [zeros(3,1);zeros(3,1);angle_measure;zeros(3,1);zeros(3,1)];

%% generate another stance
% fix one random foot
fix_foot_id = randi(param.num_leg);
while param.active_leg(fix_foot_id) ~= 1
    fix_foot_id = randi(param.num_leg);
end
disp(['fix foot '  num2str(fix_foot_id)])
% calculate fix foot_position
p_er        = state_init(1:3);
q_er        = quaternion(state_init(4:7)');
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
theta = theta_list((fix_foot_id-1)*3+1:(fix_foot_id-1)*3+3);
p_rf = autoFunc_fk_pf_pos(theta,[param.lc],[param.ox(fix_foot_id);param.oy(fix_foot_id);param.d(fix_foot_id);param.lt]);
p_wf = R_er*p_rf + p_er;  % the fix foot position in world frame
% % generate new stance location only has x direction displacement
next_x = tgt_x + 0.05*(2*randn-1);
next_y = tgt_y;
next_z = tgt_z;
next_yaw = tgt_yaw;
next_pitch = tgt_pitch;
next_roll = tgt_roll;

next_q = quaternion([next_yaw next_pitch next_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(next_q);
state_next = [next_x;next_y;next_z;w;x;y;z;zeros(3,1);zeros(3,1);zeros(3,1)];
next_p_er        = state_next(1:3);
next_q_er        = quaternion(state_next(4:7)');
next_R_er = quat2rotm(next_q_er);
next_angles = 0.2*randn(3,param.num_leg) + repmat([0;-0.7;-1.8],1,param.num_leg);
next_prf = next_R_er'*(p_wf - next_p_er);
angle = ik(next_prf, theta, [param.lc],[param.ox(fix_foot_id);param.oy(fix_foot_id);param.d(fix_foot_id);param.lt]);
[theta angle]
% modify the next_angle_measure term corresponding to fix_foot_id
next_angles(:,fix_foot_id) = angle;
next_angle_measure = next_angles(:,logical(param.active_leg));
next_angle_measure = next_angle_measure(:);

next_measurement = [zeros(3,1);zeros(3,1);next_angle_measure;zeros(3,1);zeros(3,1)];

%% generate trajectory between the two stance state, especially the measurements

%% draw everything
fig_id = 1;
fig = figure(fig_id);
set(gcf, 'units','normalized','outerposition',[0 0.2 0.3 0.7]);

clf; hold on;
draw_robot(fig, state_init, measurement, param, 1);
draw_robot(fig, state_next, next_measurement, param, 0.8);
% adjust view point of the figure 
com_pos = state_init(1:3);
set(gca,'CameraPosition',[com_pos(1)+5 com_pos(2)+5 com_pos(3)+2]);
set(gca,'CameraTarget',[com_pos(1) com_pos(2) com_pos(3)]);
set(gca,'CameraUpVector',[0 0 1]);
set(gca,'CameraViewAngle',8.6765);
set(gca, 'XLim', [com_pos(1)-1.2 com_pos(1)+1.6])
set(gca, 'YLim', [com_pos(2)-1.2 com_pos(2)+1.2])
set(gca, 'ZLim', [com_pos(3)-1.1 com_pos(3)+0.6])
drawnow;