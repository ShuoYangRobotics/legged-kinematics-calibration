function lo_vel_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel, rho_bias, param)
% the 

lo_v_data = zeros(size(joint_vel.Time,1),12);
for idx = 2:size(joint_vel.Time,1)
    t = joint_vel.Time(idx);
    tmp_idxs = find(pos_mocap.Time-t>=0);
    % interpolate to get state(mocap pos and orientation)
    mocap_t1 = orient_mocap.Time(tmp_idxs(1)-1);
    mocap_t2 = orient_mocap.Time(tmp_idxs(1));
    alpha = (t-mocap_t1)/(mocap_t2-mocap_t1);
    qi = quatinterp(orient_mocap.Data(tmp_idxs(1)-1,:),orient_mocap.Data(tmp_idxs(1),:), alpha,'slerp');
    R_er = quat2rotm(qi);
%     pi = (1-alpha)*pos_mocap.Data(tmp_idxs(1)-1,:) + alpha*pos_mocap.Data(tmp_idxs(1),:);
%     vi = (1-alpha)*vel_mocap.Data(tmp_idxs(1)-1,:) + alpha*vel_mocap.Data(tmp_idxs(1),:);
    % interpolate to get meas(joint data, IMU gyro)
    tmp_idxs = find(gyro_IMU.Time-t>=0);
    % interpolate to get state(mocap pos and orientation)
    imu_t1 = gyro_IMU.Time(tmp_idxs(1)-1);
    imu_t2 = gyro_IMU.Time(tmp_idxs(1));
    alpha = (t-imu_t1)/(imu_t2-imu_t1);
    wi = (1-alpha)*gyro_IMU.Data(tmp_idxs(1)-1,:) + alpha*gyro_IMU.Data(tmp_idxs(1),:);
    ai = (1-alpha)*accel_IMU.Data(tmp_idxs(1)-1,:) + alpha*accel_IMU.Data(tmp_idxs(1),:);
    
    % get rho state
    rho = zeros(2*param.rho_opt_size*param.num_leg,1)';
    tmp_idxs = find(rho_bias.Time-t>=0);
    rho_bias_t1 = rho_bias.Time(tmp_idxs(1)-1);
    rho_bias_t2 = rho_bias.Time(tmp_idxs(1));
    alpha = (t-rho_bias_t1)/(rho_bias_t2-rho_bias_t1);
    rho = (1-alpha)*rho_bias.Data(tmp_idxs(1)-1,:) + alpha*rho_bias.Data(tmp_idxs(1),:);
    
    v = zeros(3*param.num_leg,1);
    for i = 1:param.num_leg
        angle = joint_ang.Data(idx,(i-1)*3+1:(i-1)*3+3)';
        av = joint_vel.Data(idx,(i-1)*3+1:(i-1)*3+3)';
        % get opt rho TODO: check dimension here
        rho_opt = rho((i-1)*param.rho_opt_size+1:i*param.rho_opt_size)';
        drho = rho((i-1)*param.rho_opt_size+1+param.rho_opt_size*param.num_leg:(i-1)*param.rho_opt_size+param.rho_opt_size+param.rho_opt_size*param.num_leg)';
        p_rf = autoFunc_fk_pf_pos(angle,rho_opt,param.rho_fix(:,i));
        J_rf = autoFunc_d_fk_dt(angle,rho_opt,param.rho_fix(:,i));
        df_drho = autoFunc_d_fk_drho(angle,rho_opt,param.rho_fix(:,i));
        % it seems velocity on y direction cannot be very correctly infered 
        leg_v = (-J_rf*av-skew(wi)*p_rf);
%         leg_v = (-(J_rf*av+df_drho*drho)-skew(wi)*p_rf);
        v((i-1)*3+1:(i-1)*3+3) = R_er*leg_v;
    end

    
    lo_v_data(idx,:) = v;
end

lo_vel_ts = timeseries(lo_v_data,joint_vel.Time,'Name',"lo velocity");
end