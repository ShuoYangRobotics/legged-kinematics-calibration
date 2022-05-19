%% init configuration
run ../kinematics_init_lc;  % add libraries generate kinematics as matlab functions
run ../param_init;          % get param as global variable
warning('off')
% read rosbag, modify the path for different dataset
disp('read rosbag');
file_path = '/home/shuoyang/rosbag/2022-01-10-14-42-35.bag';  %1
% file_path = '/home/shuoyang/rosbag/2022-01-10-14-29-58.bag';  %2
% file_path = '/home/shuoyang/rosbag/2022-01-10-14-44-11.bag';    %3
[sensor_data, param] = get_sensor_data_from_dataset(file_path, param);
contact_flags = get_contact_flags_from_dataset(file_path);
% 
% default kinematics parameter
rho_param_data = param.lc_init*ones(size(sensor_data.joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_param = timeseries(rho_param_data,sensor_data.joint_ang.Time,'Name',"zero_rho_param");
% basic imu lo fusion 
lo_v_ts = get_lo_velocity_ts(sensor_data.accel_IMU, sensor_data.gyro_IMU, ...
    sensor_data.pos_mocap, sensor_data.orient_mocap,...
    sensor_data.vel_mocap, sensor_data.joint_ang, sensor_data.joint_vel,...
    rho_param, param);
[pos_est_ts1, vel_est_ts1] = imu_lo_fusion(sensor_data.accel_IMU, lo_v_ts, contact_flags);

%% doing kinematics calibration
[cali_rho_param,db_ts] = get_rho_calibration(sensor_data, contact_flags, param);

% imu lo fusion again using calibrated leg length
lo_v_ts_cali = get_lo_velocity_ts(sensor_data.accel_IMU, sensor_data.gyro_IMU, ...
    sensor_data.pos_mocap, sensor_data.orient_mocap,...
    sensor_data.vel_mocap, sensor_data.joint_ang, sensor_data.joint_vel,...
    cali_rho_param, param);
[pos_est_ts2, vel_est_ts2] = imu_lo_fusion(sensor_data.accel_IMU, lo_v_ts_cali, contact_flags);

% plot result
visualize_result;