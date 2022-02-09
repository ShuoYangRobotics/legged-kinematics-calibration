%

% read data bags, generate lists of measurements
run ../param_init

rosbags_path = '/home/shuoy/rosbag/isaac_a1/';

% bagselect = rosbag(strcat(rosbags_path, '2022-01-03-16-56-20.bag'));
bagselect = rosbag(strcat(rosbags_path, '2022-01-04-13-09-07.bag'));
%2022-01-03-22-46-20.bag
start_time =bagselect.StartTime;
duration = bagselect.EndTime-bagselect.StartTime;

%% select gt data, process it to get velocity
bSel = select(bagselect,"Time",[start_time start_time + duration],'Topic','/isaac_a1/gt_body_pose');
orient_gt = timeseries(bSel,"Pose.Pose.Orientation.W", "Pose.Pose.Orientation.X","Pose.Pose.Orientation.Y","Pose.Pose.Orientation.Z");
orient_gt.Time = orient_gt.Time-orient_gt.Time(1);

pos_gt = timeseries(bSel,"Pose.Pose.Position.X","Pose.Pose.Position.Y","Pose.Pose.Position.Z");
pos_gt.Time = pos_gt.Time-pos_gt.Time(1);

vel_gt = timeseries(bSel,"Twist.Twist.Linear.X","Twist.Twist.Linear.Y","Twist.Twist.Linear.Z");
vel_gt.Time = vel_gt.Time-vel_gt.Time(1);

%% select IMU data 
bSel2 = select(bagselect,"Time",[start_time start_time + duration],'Topic','/isaac_a1/imu_data');
accel_IMU = timeseries(bSel2,"LinearAcceleration.X","LinearAcceleration.Y","LinearAcceleration.Z");
accel_IMU.Time = accel_IMU.Time-accel_IMU.Time(1);
gyro_IMU = timeseries(bSel2,"AngularVelocity.X","AngularVelocity.Y","AngularVelocity.Z");
gyro_IMU.Time = gyro_IMU.Time-gyro_IMU.Time(1);

%% select leg data, must use struct data 
bSel3 = select(bagselect,"Time",[start_time start_time + duration],'Topic','/isaac_a1/joint_foot');
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
        if i >=2 
            joint_vel_data(i,j) = (joint_ang_data(i,j)-joint_ang_data(i-1,j))/dt(i);
        end
    end
end
%%
joint_vel_smooth_data = movmean(joint_vel_data,15,1);
foot_force = timeseries(foot_force_data,time,'Name',"foot force");
joint_ang = timeseries(joint_ang_data,time,'Name',"joint angle");
joint_vel = timeseries(joint_vel_smooth_data,time,'Name',"joint velocity");

% TODO: smooth joint_vel?


%% plot 
figure(10)
subplot(5,1,1);
plot(pos_gt)
subplot(5,1,2);
plot(vel_gt)
subplot(5,1,3);
plot(joint_ang)
subplot(5,1,4);
plot(joint_vel)
subplot(5,1,5);
plot(foot_force)

