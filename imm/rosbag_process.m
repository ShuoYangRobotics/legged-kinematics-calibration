% read data bags, generate lists of measurements
% run ../kinematics_init_lc
% run ../param_init

% bagselect = rosbag('/home/shuoy/rosbag/0110_aaron_lab/2022-01-10-14-29-58.bag');
bagselect = rosbag('/home/shuoy/rosbag/0110_aaron_lab/2022-01-10-14-33-56.bag');

% bagselect = rosbag('/home/shuoy/rosbag/home_test/2022-01-05-12-08-57.bag');
% bagselect = rosbag('/home/shuoy/rosbag/1217_aaron_lab/1217_circle.bag');
start_time =bagselect.StartTime;
duration = 25;

%% select IMU data 
bSel2 = select(bagselect,"Time",[start_time start_time + duration],'Topic','/hardware_a1/imu');
accel_IMU = timeseries(bSel2,"LinearAcceleration.X","LinearAcceleration.Y","LinearAcceleration.Z");
accel_IMU.Time = accel_IMU.Time-accel_IMU.Time(1);
accel_IMU.Data = movmean(accel_IMU.Data,40,1);

gyro_IMU = timeseries(bSel2,"AngularVelocity.X","AngularVelocity.Y","AngularVelocity.Z");
gyro_IMU.Time = gyro_IMU.Time-gyro_IMU.Time(1);

gyro_IMU.Data = movmean(gyro_IMU.Data,40,1);

%% select mocap data, process it to get velocity
bSel = select(bagselect,"Time",[start_time start_time + duration],'Topic','/mocap_node/mocap/pose');
orient_mocap = timeseries(bSel,"Pose.Orientation.W", "Pose.Orientation.X","Pose.Orientation.Y","Pose.Orientation.Z");
orient_mocap.Time = orient_mocap.Time-orient_mocap.Time(1);

pos_mocap = timeseries(bSel,"Pose.Position.X","Pose.Position.Y","Pose.Position.Z");
pos_mocap.Time = pos_mocap.Time-pos_mocap.Time(1);
dt_list = [0.001;pos_mocap.Time(2:end)-pos_mocap.Time(1:end-1)];

[b,g] = sgolay(5,25);
dt = 0.0028;
dx = zeros(length(pos_mocap.Data),3);
for p = 1:3
  dx(:,p) = conv(pos_mocap.Data(:,p), factorial(1)/(-dt)^1 * g(:,2), 'same');
end
dx(1:100,:) = 0;
dx(end-100:end,:) = 0;

% this velocity uses the IMU marker as root so
% p_b = p_r + R_er*p_br
% v_b = v_r + R_er*w*p_br
p_rb = [0.2293;0;0.095];
for idx = 2:size(pos_mocap.Time,1)
    t = pos_mocap.Time(idx);    
    R_er = quat2rotm(orient_mocap.Data(idx,:));
    tmp_idxs = find(gyro_IMU.Time-t>=0);
    % interpolate to get state(mocap pos and orientation)
    imu_t1 = gyro_IMU.Time(tmp_idxs(1)-1);
    imu_t2 = gyro_IMU.Time(tmp_idxs(1));
    alpha = (t-imu_t1)/(imu_t2-imu_t1);
    wi = (1-alpha)*gyro_IMU.Data(tmp_idxs(1)-1,:) + alpha*gyro_IMU.Data(tmp_idxs(1),:);

    dx(idx,:) = dx(idx,:) - (R_er*cross(wi,p_rb)')';
end

vel_mocap = timeseries(dx,pos_mocap.Time,'Name',"mocap velocity");


%% select leg data, must use struct data 
bSel3 = select(bagselect,"Time",[start_time start_time + duration],'Topic','/hardware_a1/joint_foot');
msgStructs = readMessages(bSel3,'DataFormat','struct');
num_data = size(msgStructs,1);
foot_force_data = zeros(num_data,param.num_leg);
joint_ang_data = zeros(num_data,param.num_dof);
joint_vel_data = zeros(num_data,param.num_dof);
time = zeros(num_data,1);
dt = zeros(num_data,1);
start_time = double(msgStructs{1}.Header.Stamp.Sec) + double(msgStructs{1}.Header.Stamp.Nsec)*10^-9;

for i=1:num_data
    time(i) = double(msgStructs{i}.Header.Stamp.Sec) + double(msgStructs{i}.Header.Stamp.Nsec)*10^-9;
    time(i) = time(i) - start_time;
    for j=1:param.num_leg
        foot_force_data(i,j) = msgStructs{i}.Effort(12+j);
    end
    if i >=2 
        dt(i) = time(i) - time(i-1);
    end
    for j=1:param.num_dof
        joint_ang_data(i,j) = msgStructs{i}.Position(j);
%         joint_vel_data(i,j) = msgStructs{i}.Velocity(j);
%         if i >=2 
%             joint_vel_data(i,j) = (joint_ang_data(i,j)-joint_ang_data(i-1,j))/dt(i);
%         end
    end
end
%%
joint_ang_data = movmean(joint_ang_data,15,1);
foot_force = timeseries(foot_force_data,time,'Name',"foot force");
joint_ang = timeseries(joint_ang_data,time,'Name',"joint angle");


[b,g] = sgolay(5,11);
dt = 0.002;   %  HARDWARE_FEEDBACK_FREQUENCY in A1_Ctrl
joint_vel_smooth_data = zeros(length(joint_ang.Data),param.num_dof);
for p = 1:param.num_dof
  joint_vel_smooth_data(:,p) = conv(joint_ang_data(:,p), factorial(1)/(-dt)^1 * g(:,2), 'same');
end
joint_vel_smooth_data(1:100,:) = 0;
joint_vel_smooth_data(end-100:end,:) = 0;
joint_vel = timeseries(joint_vel_smooth_data,time,'Name',"joint velocity");



%% plot 
figure(10)
subplot(5,1,1);
plot(pos_mocap)
subplot(5,1,2);
plot(vel_mocap)
title('mocap velocity')
xlim([10 15])
subplot(5,1,3);
plot(joint_ang)
title('Joint Angle')
xlim([10 15])
subplot(5,1,4);
plot(joint_vel)
title('Joint Angle Velocity')
xlim([10 15])
subplot(5,1,5);
plot(gyro_IMU)
title('IMU gyroscope')
xlim([10 15])

