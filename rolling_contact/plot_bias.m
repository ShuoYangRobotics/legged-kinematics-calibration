lc1 = 0.1;
lc2 = 0.2;
lc3 = 0.3;
% load('biases_3.mat')

%% put all cases together
figure(4); clf
subplot(2,1,2)
a1 = plot(rho_bias1.Time, rho_bias1.Data(:,2)+lc1,'LineWidth',3,'Color',[0.4940, 0.1840, 0.5560]); hold on;
a2 = plot(rho_bias2.Time, rho_bias2.Data(:,2)+lc2,'LineWidth',3,'Color',[0.4660, 0.6740, 0.1880]);
a3 = plot(rho_bias3.Time, rho_bias3.Data(:,2)+lc3,'LineWidth',3,'Color',[0.3010, 0.7450, 0.9330]);
xlim([5 12.5]);
ylim([0.18 0.27]);
xlabel('Time (s)')
ylabel('calf length (m)')
title('Calf length calibration from different initial values')
% subplot(4,1,2)
% plot(rho_bias1.Time, rho_bias1.Data(:,2)+lc1,'LineWidth',3); hold on;
% plot(rho_bias2.Time, rho_bias2.Data(:,2)+lc2,'LineWidth',3);
% plot(rho_bias3.Time, rho_bias3.Data(:,2)+lc3,'LineWidth',3);
% xlabel('Time (s)')
% ylabel('calf length (m)')
% title('Estimated calf length with different initial values')
% legend('lc+\rho for initial lc = 0.18',...
%     'lc+\rho for initial lc = 0.25',...
%     'lc+\rho for initial lc = 0.30');
% subplot(4,1,3)
% plot(rho_bias1.Time, rho_bias1.Data(:,3)+lc1,'LineWidth',3); hold on;
% plot(rho_bias2.Time, rho_bias2.Data(:,3)+lc2,'LineWidth',3);
% plot(rho_bias3.Time, rho_bias3.Data(:,3)+lc3,'LineWidth',3);
% xlabel('Time (s)')
% ylabel('calf length (m)')
% title('Estimated calf length with different initial values')
% legend('lc+\rho for initial lc = 0.18',...
%     'lc+\rho for initial lc = 0.25',...
%     'lc+\rho for initial lc = 0.30');
% subplot(4,1,4)
% plot(rho_bias1.Time, rho_bias1.Data(:,4)+lc1,'LineWidth',3); hold on;
% plot(rho_bias2.Time, rho_bias2.Data(:,4)+lc2,'LineWidth',3);
% plot(rho_bias3.Time, rho_bias3.Data(:,4)+lc3,'LineWidth',3);
% xlabel('Time (s)')
% ylabel('calf length (m)')
% title('Estimated calf length with different initial values')
% legend('lc+\rho for initial lc = 0.18',...
%     'lc+\rho for initial lc = 0.25',...
%     'lc+\rho for initial lc = 0.30');



%% plot bias, velocity
% 1.1 calculate lo velocity
warning('off')
param.lc = lc3
% no bias first
rho_bias_data = zeros(size(joint_ang.Time,1),param.rho_opt_size*param.num_leg*2);
rho_bias = timeseries(rho_bias_data,joint_ang.Time,'Name',"zero_rho_bias");


lo_v_ts = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_bias, param);

% 1.2 draw lo velocity
figure(4);
subplot(2,1,1);

ll = movmean(foot_force.Data(:,1),25,1)-80';
no_contact = 1-1./(1+exp(-10*ll));
% contact = movmean(contact,3,1);
contact = 300*(1-no_contact);
% plot(foot_force.Time, tmp);
dtmp = diff(contact);
% plot(foot_force.Time(1:end-1), dtmp);
times = foot_force.Time(1:end-1);
start_time = times(dtmp>50);
end_time = times(dtmp<-50);
start_time = uniquetol(start_time,0.01,'DataScale',1);
end_time = uniquetol(end_time,0.01,'DataScale',1);
for i=1:min(size(start_time,1),size(end_time,1))
    if start_time(i)<end_time(i)
        subplot(2,1,1);
        a = area([start_time(i) end_time(i)],[-2 -2]); hold on;
        a.FaceAlpha = 0.2;
        a.FaceColor = [0.2 0.6 0.5];
        b = area([start_time(i) end_time(i)],[2 2]); 
        b.FaceAlpha = 0.2;
        b.FaceColor = [0.2 0.6 0.5];
        subplot(2,1,2);
        a = area([start_time(i) end_time(i)],[0.3 0.3]);
        a.FaceAlpha = 0.2;
        a.FaceColor = [0.2 0.6 0.5];
    end
end

subplot(2,1,1);
p1 = plot(vel_mocap.Time, vel_mocap.Data(:,1),'LineWidth',3);hold on;
ylim([-0.2 1]);
xlabel('Time (s)')
ylabel('Velocity (m/s)')
% xlim([0 20])
% title(['Ground Truth X-direction body velocity'])

% title(['Body velocity with: ' num2str(param.lc)])
title(['Inaccurate calf length leads to body velocity estimation error'])
for i=1:1
    subplot(2,1,1);
    p2 = plot(lo_v_ts.Time, movmean(lo_v_ts.Data(:,1+3*(i-1)),15,1),'LineWidth',2);hold on;
    xlim([5 12.5]);
end

% 1.6 plot on figure 1 of velocity with bias
lo_v_ts2 = get_lo_velocity_ts(accel_IMU, gyro_IMU, pos_mocap, orient_mocap,...
    vel_mocap, joint_ang, joint_vel,rho_bias3, param);

figure(4)
for i=1:1
    subplot(2,1,1);
    p4 = plot(lo_v_ts2.Time, movmean(lo_v_ts2.Data(:,1+3*(i-1)),15,1),'LineWidth',2);hold on;
    xlim([5 12.5]);
end

%%
subplot(2,1,1);
legend([p1 p2,p4],{'ground truth velocity', ...
    'estimation with calf length 0.3m',...
    'estimation with online calibration'}, 'Location','Northwest')

subplot(2,1,2);
legend([a1 a2,a3],{'initial value = 0.1',...
    'initial value = 0.2',...
    'initial value = 0.3'});