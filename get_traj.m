function [state_list, meas_list] = get_traj(traj_t, pose_init, pose_end, angle_init, fix_foot_id_list, visible_feature_ids, feature_px_pos_init, param)
% input
%  - traj_t: a list of trajectory time
%  - pose_init:  pose at t=0
%  - pose_end:  pose at t=traj_t(end)
%  - angle_measure_init:  joint angles initially
%  - angle_measure_end :  joint angles at the end
%  - visible_feature_ids: The ids of features
%  - feature_px_pos_init: The feature positions on image at t=0, it is a
%  mx1 vector
%  - param:               contains a lot of things especially features

% output 
%  - state_list has size param.state_size x size(traj_t)
%  - meas_list has size (param.meas_size+feature_meas_size) x size(traj_t)

% setup output matrix
num_visible_features = max(size(visible_feature_ids));
feature_meas_size = 2*num_visible_features;

traj_len = max(size(traj_t));
state_list = zeros(param.state_size, traj_len);
meas_list = zeros(param.meas_size+feature_meas_size, traj_len);

traj_dt = traj_t - [0 traj_t(1:end-1)];

% position trajectory
pos_init = pose_init(1:3);
pos_end = pose_end(1:3);

p_list = zeros(3, traj_len); p_list(:,1) = pos_init; p_list(:,traj_len) = pos_end;
dp_list = zeros(3, traj_len);
ddp_list = zeros(3, traj_len);
for i=2:traj_len
    [p,dp,ddp] = hermite_cubic_knot(traj_t(i), traj_t(1), pos_init, zeros(3,1), traj_t(end), pos_end, (pos_end-pos_init)/traj_t(end));
    p_list(:,i) = p;
    dp_list(:,i) = dp;
    ddp_list(:,i) = ddp;
    
end

% orientation trajectory
q_init = quaternion(pose_init(4:7)');
q_end = quaternion(pose_end(4:7)');

q_list = zeros(4, traj_len);
w_list = zeros(3, traj_len);
interpolatedQuaternions = slerp(q_init,q_end,traj_t/traj_t(end));

for i=1:traj_len
    [q_list(1,i),q_list(2,i),q_list(3,i),q_list(4,i)] = parts(interpolatedQuaternions(i));
end

for i=2:traj_len
    dq = interpolatedQuaternions(i-1)'*interpolatedQuaternions(i);
    [w,x,y,z] = parts(dq);
    nw = acos(w)/traj_dt(i)*2;
    omega = [x;y;z]*nw/sin(nw*traj_dt(i)/2);
    w_list(:,i-1) = omega;
end

% test intergration
% qx = q_init;
% 
% for i=1:traj_len-1
%     w =  w_list(:,i);
%     nw = norm(w);
%     vw = [0;0;0];
%     if nw ~= 0
%         vw = w/nw*sin(nw*traj_dt(i+1)/2);
%     else
%         vw = [0;0;0];
%     end
% 
% %     qx =qx*quaternion(cos(nw*dt/2),vw(1),vw(2),vw(3))
%     qx =qx+qx*quaternion(0,0.5*w(1)*traj_dt(i+1),0.5*w(2)*traj_dt(i+1),0.5*w(3)*traj_dt(i+1))
% end
%q_end should equals to the final qx
% qx
% q_end

% start to calculate leg angles
% first
% calculate fix foot_position
p_er        = pose_init(1:3);
q_er        = quaternion(pose_init(4:7)');
R_er = quat2rotm(q_er);
fix_foot_pos_list = zeros(3,size(fix_foot_id_list,1));
for i=1:size(fix_foot_id_list,1)
    fix_foot_id = fix_foot_id_list(i);
    theta = angle_init((fix_foot_id-1)*3+1:(fix_foot_id-1)*3+3);
    p_rf = autoFunc_fk_pf_pos(theta,param.rho_opt_true(:,fix_foot_id),param.rho_fix(:,fix_foot_id));
    fix_foot_pos_list(:,i) = R_er*p_rf + p_er;  % the fix foot position in world frame

end

% leg order 1FL 2FR 3RL 4RR
joint_angle_list = zeros(3*param.num_leg, traj_len);
joint_av_list = zeros(3*param.num_leg, traj_len);
joint_angle_list(:,1) = angle_init;

% generate camera feature measurements
feature_pt_pos_list = zeros(feature_meas_size, traj_len);
feature_pt_pos_list(:,1) = feature_px_pos_init;
    
for i=2:traj_len
    next_p_er = p_list(:,i);
    next_q_er = quaternion(q_list(:,i)');
    
    for j = 1:param.num_leg
        mask = fix_foot_id_list(:) == j;
        if any(mask) 
            fix_pos = fix_foot_pos_list(:,mask);
            fix_foot_id = fix_foot_id_list(mask);
            init_angle = joint_angle_list((j-1)*3+1:(j-1)*3+3,i-1);
            angle = get_joint_angle_from_foot_pos_and_body_pose(next_p_er, next_q_er, fix_pos, fix_foot_id, init_angle, param);
            joint_angle_list((j-1)*3+1:(j-1)*3+3,i) = angle;
        else
            joint_angle_list((j-1)*3+1:(j-1)*3+3,i) = joint_angle_list((j-1)*3+1:(j-1)*3+3,i-1);
        end
    end
    joint_av_list(:,i) = (joint_angle_list(:,i) - joint_angle_list(:,i-1))/traj_dt(i);
    
    feature_pt_pos_list(:,i) = project_features(next_p_er, next_q_er, visible_feature_ids, param);
end


% assemble p_list dp_list ddp_list q_list w_list joint_angle_list joint_av_list
% into state_list, meas_list
for i=1:traj_len
    state_list(1:3,i) = p_list(:,i);
    state_list(4:7,i) = q_list(:,i);
    state_list(8:10,i) = dp_list(:,i);  
    
    % rho depends on the definition in kinematics init 
    for j=1:param.num_leg
        state_list(10+(j-1)*param.rho_opt_size+1:10+j*param.rho_opt_size,i) = param.rho_opt_true(:,j);
    end
    
    % todo: inject some noise here?
    meas_list(1:3,i) = R_er'*ddp_list(:,i);  % notice this does not have gravity
    meas_list(4:6,i) = w_list(:,i);
    meas_list(7:18,i) = joint_angle_list(:,i);
    meas_list(19:30,i) = joint_av_list(:,i);
    meas_list(31:end,i) = feature_pt_pos_list(:,i);
end

end





























