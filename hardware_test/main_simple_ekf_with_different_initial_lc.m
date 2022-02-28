% assume at this time we have following sensor data time series:
% accel_IMU, gyro_IMU, vel_mocap, 
% foot_force, joint_ang, joint_vel

% we also have vel_mocap which can be used as ground truth

% first add functions in the parent folder
% we especially needs all the kinematicd functions 
addpath('..')
addpath('../mr')

% run kinematics function generation
% can comment the following two scripts after running them once
% run ../kinematics_init_lc
% run ../kinematics_init_cxyz
% run ../param_init

%% 1. lc = 0.18
param.lc_init = 0.18;

imm_kf_together;

rho_param1 = rho_param;
rho_variance1 = rho_variance;

%% 2. lc = 0.25
param.lc_init = 0.25;

imm_kf_together;

rho_param2 = rho_param;
rho_variance2 = rho_variance;

%% 3. lc = 0.30
param.lc_init = 0.30;

imm_kf_together;

rho_param3 = rho_param;
rho_variance3 = rho_variance;

%% put all cases together
figure(4); clf
% subplot(4,1,1)
% plot(rho_param1.Time, rho_param1.Data(:,1),'LineWidth',3); hold on;
% plot(rho_param2.Time, rho_param2.Data(:,1),'LineWidth',3);
% plot(rho_param3.Time, rho_param3.Data(:,1),'LineWidth',3);
% xlabel('Time (s)')
% ylabel('calf length (m)')
% title('Estimated calf length with different initial values')
% legend('Leg 1 length, initial lc = 0.18',...
%     'Leg 1 length, initial lc = 0.25',...
%     'Leg 1 length, initial lc = 0.30');
% subplot(4,1,2)
plot_end_time = 5;
leg_idx = 3;

plot_rho_param1 = resample(rho_param1,0:0.03:plot_end_time);
plot_rho_param2 = resample(rho_param2,0:0.03:plot_end_time);
plot_rho_param3 = resample(rho_param3,0:0.03:plot_end_time);


plot_rho_variance1 = resample(rho_variance1,0:0.03:plot_end_time);
plot_rho_variance2 = resample(rho_variance2,0:0.03:plot_end_time);
plot_rho_variance3 = resample(rho_variance3,0:0.03:plot_end_time);


plot(plot_rho_param1.Time, 0.21*ones(size(plot_rho_param1.Time))','.-'); hold on;
p1 = plot(plot_rho_param1.Time, plot_rho_param1.Data(:,leg_idx),'Color','#0072BD','LineWidth',3); hold on;
p2 = plot(plot_rho_param2.Time, plot_rho_param2.Data(:,leg_idx),'Color','#D95319','LineWidth',3);
p3 = plot(plot_rho_param3.Time, plot_rho_param3.Data(:,leg_idx),'Color','#EDB120','LineWidth',3);


plot(plot_rho_variance1.Time, plot_rho_param1.Data(:,leg_idx)+3*plot_rho_variance1.Data(:,leg_idx),'b'); hold on;
plot(plot_rho_variance1.Time, plot_rho_param1.Data(:,leg_idx)-3*plot_rho_variance1.Data(:,leg_idx),'b'); hold on;
plot(plot_rho_variance2.Time, plot_rho_param2.Data(:,leg_idx)+3*plot_rho_variance2.Data(:,leg_idx),'b'); hold on;
plot(plot_rho_variance2.Time, plot_rho_param2.Data(:,leg_idx)-3*plot_rho_variance2.Data(:,leg_idx),'b'); hold on;
plot(plot_rho_variance3.Time, plot_rho_param3.Data(:,leg_idx)+3*plot_rho_variance3.Data(:,leg_idx),'b'); hold on;
plot(plot_rho_variance3.Time, plot_rho_param3.Data(:,leg_idx)-3*plot_rho_variance3.Data(:,leg_idx),'b'); hold on;

xlabel('Time (s)')
ylabel('Calf length (m)')
title('Different calibration runs with different initial values')
legend([p1,p2,p3],{'Leg 2 calf length, initial value = 0.18m',...
    'Leg 2 calf length, initial value = 0.25m',...
    'Leg 2 calf length, initial value = 0.30m'});
% subplot(4,1,3)
% plot(rho_param1.Time, rho_param1.Data(:,3),'LineWidth',3); hold on;
% plot(rho_param2.Time, rho_param2.Data(:,3),'LineWidth',3);
% plot(rho_param3.Time, rho_param3.Data(:,3),'LineWidth',3);
% xlabel('Time (s)')
% ylabel('calf length (m)')
% title('Estimated calf length with different initial values')
% legend('Leg 3 length, initial lc = 0.18',...
%     'Leg 3 length, initial lc = 0.25',...
%     'Leg 3 length, initial lc = 0.30');
% subplot(4,1,4)
% plot(rho_param1.Time, rho_param1.Data(:,4),'LineWidth',3); hold on;
% plot(rho_param2.Time, rho_param2.Data(:,4),'LineWidth',3);
% plot(rho_param3.Time, rho_param3.Data(:,4),'LineWidth',3);
% xlabel('Time (s)')
% ylabel('calf length (m)')
% title('Estimated calf length with different initial values')
% legend('Leg 4 length, initial lc = 0.18',...
%     'Leg 4 length, initial lc = 0.25',...
%     'Leg 4 length, initial lc = 0.30');

matlab2tikz('calibration_results2.tex',...
            'extraAxisOptions',{'ylabel near ticks','xlabel near ticks',...
            'scaled ticks=false',...
            'stack plots=false',...
            'legend style={at={(0.97,0.97)}, anchor=north east, legend cell align=left, align=left, draw=white!15!black},',...
            'xtick distance=0.5',...
            'ytick distance=0.1',...
            'ylabel shift = -12pt',...
            'xlabel shift = -10pt',...
            'xtick={0,0.05,0.1}',...
            'ytick distance=0.5',...
            'minor tick num=1',...
            'ylabel style={font=\small\fontfamily{ptm}\selectfont}',...
            'xlabel style={font=\small\fontfamily{ptm}\selectfont}',...
            'ticklabel style={font=\small\fontfamily{ptm}\selectfont}',...
            })
