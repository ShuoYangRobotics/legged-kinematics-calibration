%% init configuration
run ../kinematics_init_lc;  % add libraries generate kinematics as matlab functions
run ../param_init;          % get param as global variable
warning('off')
% read rosbag, modify the path for different dataset
disp('read rosbag');
% file_path = '/home/shuoyang/rosbag/2022-01-10-14-42-35.bag';  %1
% file_path = '/home/shuoyang/rosbag/2022-01-10-14-29-58.bag';  %2
file_path = '/home/shuoyang/rosbag/2022-01-10-14-44-11.bag';    %3
% file_path = '/home/shuoyang/rosbag/2022-01-10-14-46-26.bag';    %4
% file_path = '/home/shuoyang/rosbag/2022-01-10-14-47-38.bag';    %5
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
param.with_observability = 1;
[cali_rho_param1,~] = get_rho_calibration(sensor_data, contact_flags, param);
param.with_observability = 0;
[cali_rho_param2,~] = get_rho_calibration(sensor_data, contact_flags, param);

% imu lo fusion again using calibrated leg length
lo_v_ts_cali = get_lo_velocity_ts(sensor_data.accel_IMU, sensor_data.gyro_IMU, ...
    sensor_data.pos_mocap, sensor_data.orient_mocap,...
    sensor_data.vel_mocap, sensor_data.joint_ang, sensor_data.joint_vel,...
    cali_rho_param1, param);
[pos_est_ts2, vel_est_ts2] = imu_lo_fusion(sensor_data.accel_IMU, lo_v_ts_cali, contact_flags);

%% plot
figure('Name','calibrate',...
    'Units','inches',...
'Position',[0 0 5 2.85],...
'PaperPositionMode','auto')
clf;
plot_start_time = 9;
plot_end_time = 13;
plot_dt = 0.01;
plot_dim = 1;
subplot(2,1,1)
tsout = resample(cali_rho_param1,plot_start_time:plot_dt:plot_end_time); hold on;
plot(tsout.Time, tsout.Data(:,1),'g','LineWidth',2);hold on;

tsout = resample(cali_rho_param2,plot_start_time:plot_dt:plot_end_time); hold on;
plot(tsout.Time, tsout.Data(:,1),'Color',[1 0 0],'LineWidth',2);hold on;
set(gca,...
'Units','normalized',...
'YTick',0.1:.05:0.25,...
'XTick',plot_start_time:1:plot_end_time,...
'FontUnits','points',...
'FontWeight','normal',...
'FontSize',6,...
'FontName','Times')
ylabel({'Length (m)'},...
'FontUnits','points',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')

xlabel({'(a) Time (s)'},...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')
xlim([plot_start_time plot_end_time])
% draw lines
SP=9.7; %your point goes here 
line([SP SP],get(gca,'YLim'),'LineStyle','--','Color',[0 0 0])
SP=11.2; %your point goes here 
line([SP SP],get(gca,'YLim'),'LineStyle','--','Color',[0 0 0])
txt = {'Standing'};
text(9.2,0.11,txt,'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')
txt = {'In-place trotting'};
text(10.1,0.11,txt,'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')
txt = {'Moving forward'};
text(11.7,0.11,txt,'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')
h=legend('With LUI noise', 'W/o LUI noise', ...
                       '','',...
'FontUnits','points',...
'interpreter','latex',...
'FontSize',6,...
'FontName','Times',  'Location', 'none');
pos = get(h,'Position');
posx = 0.29;
posy = 0.25;
set(h,'Position',[posx posy pos(3) pos(4)]);

subplot(2,1,2)
tsout = resample(sensor_data.vel_mocap,plot_start_time:plot_dt:plot_end_time);
tsout_Time = tsout.Time;
tsout_Data = tsout.Data(:,plot_dim);
p1 = plot(tsout_Time,tsout_Data,'LineWidth',2); hold on; 
set(gca,...
'Units','normalized',...
'YTick',-0.2:.2:0.9,...
'XTick',plot_start_time:1:plot_end_time,...
'FontUnits','points',...
'FontWeight','normal',...
'FontSize',6,...
'FontName','Times')
ylabel({'X Velocity (m/s)'},...
'FontUnits','points',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')

xlabel({'(b) Time (s)'},...
'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')
xlim([plot_start_time plot_end_time])
ylim([-0.2 0.9])
% draw lines
SP=9.7; %your point goes here 
line([SP SP],get(gca,'YLim'),'LineStyle','--','Color',[0 0 0])
SP=11.2; %your point goes here 
line([SP SP],get(gca,'YLim'),'LineStyle','--','Color',[0 0 0])
txt = {'Standing'};
text(9.2,-0.12,txt,'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')
txt = {'In-place trotting'};
text(10.1,-0.12,txt,'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')
txt = {'Moving forward'};
text(11.7,-0.12,txt,'FontUnits','points',...
'FontWeight','normal',...
'interpreter','latex',...
'FontSize',10,...
'FontName','Times')

matlab2tikz('compare_observablity_noise.tex',...
            'extraAxisOptions',{'ylabel near ticks','xlabel near ticks',...
            'scaled ticks=false',...
            'stack plots=false' })