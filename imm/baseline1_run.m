% assume accel_IMU, gyro_IMU, joint_ang, joint_vel, foot_force, param are available

% use baseline to determine a single velocity, use contact flag to combine leg velocities

% the threshold to convert foot force to binary contact 
foot_forces_thres = [90,90,20,90];
% determing foot_forces_thres dynamically
foot_force_extrema = zeros(param.num_leg,2);


% save output velocity timeseries 
baseline_v = zeros(length(joint_vel.Data),3);

% save estimated contact 
contact_estimation = zeros(length(joint_vel.Data),4);


% for each time step 
for t_idx = 2:size(joint_vel.Time,1)
    t = joint_vel.Time(t_idx);
    dt = joint_vel.Time(t_idx) - joint_vel.Time(t_idx-1);
    
    % interpolate to get state(mocap pos and orientation)
    tmp_idxs = find(pos_mocap.Time-t>=0);
    mocap_t1 = orient_mocap.Time(tmp_idxs(1)-1);
    mocap_t2 = orient_mocap.Time(tmp_idxs(1));
    alpha = (t-mocap_t1)/(mocap_t2-mocap_t1);
    qi = quatinterp(orient_mocap.Data(tmp_idxs(1)-1,:),orient_mocap.Data(tmp_idxs(1),:), alpha,'slerp');
    R_er = quat2rotm(qi);
    
    % interpolate to get mocap vel 
    tmp_idxs = find(vel_mocap.Time-t>=0);
    mocap_t1 = vel_mocap.Time(tmp_idxs(1)-1);
    mocap_t2 = vel_mocap.Time(tmp_idxs(1));
    alpha = (t-mocap_t1)/(mocap_t2-mocap_t1);
    vi = (1-alpha)*vel_mocap.Data(tmp_idxs(1)-1,:) + alpha*vel_mocap.Data(tmp_idxs(1),:);

    % interpolate to get IMU w and a
    tmp_idxs = find(gyro_IMU.Time-t>=0);
    imu_t1 = gyro_IMU.Time(tmp_idxs(1)-1);
    imu_t2 = gyro_IMU.Time(tmp_idxs(1));
    alpha = (t-imu_t1)/(imu_t2-imu_t1);
    wi = (1-alpha)*gyro_IMU.Data(tmp_idxs(1)-1,:) + alpha*gyro_IMU.Data(tmp_idxs(1),:);
    ai = (1-alpha)*accel_IMU.Data(tmp_idxs(1)-1,:) + alpha*accel_IMU.Data(tmp_idxs(1),:);
    
    % four leg velocities
    v = zeros(3,param.num_leg);
    for i = 1:param.num_leg
        angle = joint_ang.Data(t_idx,(i-1)*3+1:(i-1)*3+3)';
        av = joint_vel.Data(t_idx,(i-1)*3+1:(i-1)*3+3)';
        p_rf = autoFunc_fk_pf_pos(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        J_rf = autoFunc_d_fk_dt(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        df_drho = autoFunc_d_fk_drho(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        % it seems velocity on y direction cannot be very correctly infered 
        leg_v = (-J_rf*av-skew(wi)*p_rf);
%         leg_v = (-(J_rf*av+df_drho*drho)-skew(wi)*p_rf);
        v(:,i) = R_er*leg_v;
    end

    % from foot force, generate transition probablitity
    foot_forces = foot_force.Data(t_idx,:);
    contact_flags = zeros(param.num_leg,1);
    for j=1:param.num_leg
        if (foot_forces(j) < foot_force_extrema(j,1))
            foot_force_extrema(j,1) = foot_forces(j);
        end
        if (foot_forces(j) > foot_force_extrema(j,2))
            foot_force_extrema(j,2) = foot_forces(j);
        end
        threadhold = 0.5*(foot_force_extrema(j,2)-foot_force_extrema(j,1))+foot_force_extrema(j,1);
        if (foot_forces(j) > threadhold)
            contact_flags(j) = 1;
        end
    end
    
    
    num_contacts = sum(contact_flags)+0.0001;
    baseline_v(t_idx,:) = v * contact_flags / num_contacts;
    
end

vel_baseline1 = timeseries(baseline_v,joint_vel.Time,'Name',"Baseline velocity");   

%% compare with vel_mocap
figure(1); clf
plot(vel_imm.Time, vel_imm.Data(:,1)) ; hold on;
plot(vel_baseline1.Time, vel_baseline1.Data(:,1)) ; hold on;
% plot(vel_imm.Time, movmean(vel_imm.Data(:,1),30,1)) ; hold on;
plot(vel_mocap.Time, vel_mocap.Data(:,1)) ;
legend('vel imm X', 'vel baseline X',  'vel mocap X');