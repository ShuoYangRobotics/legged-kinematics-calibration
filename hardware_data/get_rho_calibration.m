function [rho_param,db_ts] = get_rho_calibration(sensor_data, contact_flags_ts, param)
% now we run kinmetic calibration

% run ekf to estimate fk parameters
traj_len = length(sensor_data.joint_vel.Data);
% state_init = [param.lc_init + 0.1*randn(param.rho_opt_size*param.num_leg,1)];
state_init = [param.lc_init*ones(param.rho_opt_size*param.num_leg,1)];

est_state_size = param.rho_opt_size*param.num_leg;
est_state_list = zeros(est_state_size, traj_len);
est_variance_list = zeros(est_state_size, traj_len);
est_state_time = zeros(1,traj_len);
% estimation covariance
rho_P = 0.5*eye(est_state_size);
% process noise covariance
rho_Q = diag([0.0001*ones(param.rho_opt_size*param.num_leg,1)]);
% measurement noise covariance
rho_R3 = diag(100*ones(3,1));
% save initial state
est_state_list(:,1) = state_init;
% save variance
est_variance_list(:,1) = 0.01*zeros(est_state_size,1);

db = zeros(1,traj_len);
% get contact flag
contact_estimation = resample(contact_flags_ts,sensor_data.joint_vel.Time);

% for each time step 
for t_idx = 2:size(sensor_data.joint_vel.Time,1)
    t = sensor_data.joint_vel.Time(t_idx);
    est_state_time(:,t_idx) = t;
    if mod(t_idx,4950) == 0
        display(t_idx);
    end

    est_state_time(:,t_idx) = t;
    % process update    
    est_state_list(:,t_idx) = rho_kf_process(est_state_list(:,t_idx-1));
    F = rho_kf_process_jac(param);

    rho_P = F*rho_P*F' + rho_Q;

    % interpolate to get state(mocap pos and orientation)
    tmp_idxs = find(sensor_data.pos_mocap.Time-t>=0);
    mocap_t1 = sensor_data.orient_mocap.Time(tmp_idxs(1)-1);
    mocap_t2 = sensor_data.orient_mocap.Time(tmp_idxs(1));
    alpha = (t-mocap_t1)/(mocap_t2-mocap_t1);
    qi = quatinterp(sensor_data.orient_mocap.Data(tmp_idxs(1)-1,:),sensor_data.orient_mocap.Data(tmp_idxs(1),:), alpha,'slerp');
    R_er = quat2rotm(qi);
    % interpolate to get mocap vel as sensor measurement
    tmp_idxs = find(sensor_data.vel_mocap.Time-t>=0);
    mocap_t1 = sensor_data.vel_mocap.Time(tmp_idxs(1)-1);
    mocap_t2 = sensor_data.vel_mocap.Time(tmp_idxs(1));
    alpha = (t-mocap_t1)/(mocap_t2-mocap_t1);
    vi = (1-alpha)*sensor_data.vel_mocap.Data(tmp_idxs(1)-1,:) + alpha*sensor_data.vel_mocap.Data(tmp_idxs(1),:);
    % interpolate to get IMU w and a
    tmp_idxs = find(sensor_data.gyro_IMU.Time-t>=0);
    imu_t1 = sensor_data.gyro_IMU.Time(tmp_idxs(1)-1);
    imu_t2 = sensor_data.gyro_IMU.Time(tmp_idxs(1));
    alpha = (t-imu_t1)/(imu_t2-imu_t1);
    wi = (1-alpha)*sensor_data.gyro_IMU.Data(tmp_idxs(1)-1,:) + alpha*sensor_data.gyro_IMU.Data(tmp_idxs(1),:);
    ai = (1-alpha)*sensor_data.accel_IMU.Data(tmp_idxs(1)-1,:) + alpha*sensor_data.accel_IMU.Data(tmp_idxs(1),:);
    
    % the predict output
    z = [vi'];
    contacts = zeros(param.num_leg,1);
    for j = 1:param.num_leg
        contacts(j) = contact_estimation.Data(t_idx,j) > 0.99;
    end
    if (sum(contacts) > 0)
        zhat = rho_kf_measurement(est_state_list(:,t_idx), R_er, ...
            wi, sensor_data.joint_ang.Data(t_idx,:), sensor_data.joint_vel.Data(t_idx,:),contacts, param);
    
        H = rho_kf_measurement_jac(est_state_list(:,t_idx), R_er, ...
            wi, sensor_data.joint_ang.Data(t_idx,:), sensor_data.joint_vel.Data(t_idx,:),contacts, param);
        
        % test H
    %     drho = 0.01*rand(4,1);
    %     zhat2 = rho_kf_measurement(est_state_list(:,t_idx)+drho, R_er, ...
    %         wi, joint_ang.Data(t_idx,:), joint_vel.Data(t_idx,:),contacts, param);
    %     zhat2 - (zhat+H*drho)
    
        innov = z-zhat;

        db(t_idx) = mean(svd(H));
    
        weight = 0;
        weight = 300./(1+exp(10*(mean(svd(H))-0.3)));
        rho_R3_mod = rho_R3 + weight*eye(3);
    
        rho_K = rho_P*H'/(H*rho_P*H'+rho_R3_mod);
        delta_x = rho_K*(innov);
        rho_P = (eye(est_state_size) - rho_K*H)*rho_P;
        rho_P = (rho_P+rho_P')/2;
        est_state_list(:,t_idx) = est_state_list(:,t_idx) + delta_x;
    else
        est_state_list(:,t_idx) = est_state_list(:,t_idx);
    end
    
    est_variance_list(:,t_idx) = diag(rho_P(1:est_state_size,1:est_state_size));
end
%%
rho_param_data = est_state_list';
% rho_param_data = movmean(rho_param_data,55,1);
rho_param = timeseries(rho_param_data,est_state_time,'Name',"rho_param");

db_ts = timeseries(db,est_state_time,'Name',"db");
end