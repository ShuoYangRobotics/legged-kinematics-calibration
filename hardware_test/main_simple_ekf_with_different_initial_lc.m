% assume at this time we have following sensor data time series:
% accel_IMU, gyro_IMU, vel_mocap, 
% foot_force, joint_ang, joint_vel

% we also have vel_mocap which can be used as ground truth

% first add functions in the parent folder
% we especially needs all the kinematicd functions 
addpath('..')
addpath('../unitree_A1/mr')

% run kinematics function generation
% can comment the following two scripts after running them once
% run ../kinematics_init_lc
% run ../kinematics_init_cxyz
% run ../param_init

%% 1. lc = 0.18
param.lc_init = 0.18;

imm_kf_together;

rho_bias1 = rho_bias;

%% 2. lc = 0.25
param.lc_init = 0.25;

imm_kf_together;

rho_bias2 = rho_bias;

%% 3. lc = 0.30
param.lc_init = 0.30;

imm_kf_together;

rho_bias3 = rho_bias;

%% put all cases together
figure(4); clf
% subplot(4,1,1)
% plot(rho_bias1.Time, rho_bias1.Data(:,1),'LineWidth',3); hold on;
% plot(rho_bias2.Time, rho_bias2.Data(:,1),'LineWidth',3);
% plot(rho_bias3.Time, rho_bias3.Data(:,1),'LineWidth',3);
% xlabel('Time (s)')
% ylabel('calf length (m)')
% title('Estimated calf length with different initial values')
% legend('Leg 1 length, initial lc = 0.18',...
%     'Leg 1 length, initial lc = 0.25',...
%     'Leg 1 length, initial lc = 0.30');
% subplot(4,1,2)

rho_bias1 = resample(rho_bias1,0:0.03:6);
rho_bias2 = resample(rho_bias2,0:0.03:6);
rho_bias3 = resample(rho_bias3,0:0.03:6);
p1 = plot(rho_bias1.Time, rho_bias1.Data(:,2),'LineWidth',3); hold on;
p2 = plot(rho_bias2.Time, rho_bias2.Data(:,2),'LineWidth',3);
p3 = plot(rho_bias3.Time, rho_bias3.Data(:,2),'LineWidth',3);
plot(rho_bias1.Time, 0.21*ones(size(rho_bias1.Time))','.-'); hold on;
xlabel('Time (s)')
ylabel('Calf length (m)')
title('Different calibration runs with different initial values')
legend([p1,p2,p3],{'Leg 2 calf length, initial value = 0.18m',...
    'Leg 2 calf length, initial value = 0.25m',...
    'Leg 2 calf length, initial value = 0.30m'});
% subplot(4,1,3)
% plot(rho_bias1.Time, rho_bias1.Data(:,3),'LineWidth',3); hold on;
% plot(rho_bias2.Time, rho_bias2.Data(:,3),'LineWidth',3);
% plot(rho_bias3.Time, rho_bias3.Data(:,3),'LineWidth',3);
% xlabel('Time (s)')
% ylabel('calf length (m)')
% title('Estimated calf length with different initial values')
% legend('Leg 3 length, initial lc = 0.18',...
%     'Leg 3 length, initial lc = 0.25',...
%     'Leg 3 length, initial lc = 0.30');
% subplot(4,1,4)
% plot(rho_bias1.Time, rho_bias1.Data(:,4),'LineWidth',3); hold on;
% plot(rho_bias2.Time, rho_bias2.Data(:,4),'LineWidth',3);
% plot(rho_bias3.Time, rho_bias3.Data(:,4),'LineWidth',3);
% xlabel('Time (s)')
% ylabel('calf length (m)')
% title('Estimated calf length with different initial values')
% legend('Leg 4 length, initial lc = 0.18',...
%     'Leg 4 length, initial lc = 0.25',...
%     'Leg 4 length, initial lc = 0.30');

matlab2tikz('calibration_results.tex',...
            'extraAxisOptions',{'ylabel near ticks','xlabel near ticks',...
            'scaled ticks=false',...
            'stack plots=false',...
            'legend style={at={(0.97,0.97)}, anchor=north east, legend cell align=left, align=left, draw=white!15!black},',...
            'xtick distance=0.5',...
            'ytick distance=0.1',...
            'ylabel shift = -12pt',...
            'xlabel shift = -10pt',...
%             'xtick={0,0.05,0.1}',...
%             'ytick distance=0.5',...
%             'minor tick num=1',...
%             'ylabel style={font=\small\fontfamily{ptm}\selectfont}',...
%             'xlabel style={font=\small\fontfamily{ptm}\selectfont}',...
%             'ticklabel style={font=\small\fontfamily{ptm}\selectfont}',...
            })
