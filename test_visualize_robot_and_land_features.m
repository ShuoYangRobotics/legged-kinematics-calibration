%% init necessary parameters
% init kinematics parameter
% kinematics_init;
% init parameter
param_init;

% adjust number of active legs 
param.active_leg = [1,1,1,1];



%% generate one random stance
tgt_x = 1.8;
tgt_y = 0;
tgt_z = 0;
tgt_yaw = 10*randn;
tgt_pitch = 5*randn;
tgt_roll = 5*randn;

q = quaternion([tgt_yaw tgt_pitch tgt_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(q);
state_init = [tgt_x;tgt_y;tgt_z;w;x;y;z;zeros(3,1);zeros(3,1);zeros(3,1)];
angle_measure = 0.3*randn(3*sum(param.active_leg),1);

measurement = [zeros(3,1);zeros(3,1);angle_measure;zeros(3,1);zeros(3,1)];


%% generate random land features, save feature info in param
features_init;

%% given current state, project land features to camera frame of the robot
[num_visible_features, visible_feature_ids, feature_px_pos] = project_visible_features(state_init, param);

%% draw them 

fig_id = 1;
fig = figure(fig_id);
set(gcf, 'units','normalized','outerposition',[0.0042 0.2074 0.6031 0.6139]);

clf; hold on;
draw_robot(fig, state_init, measurement, param, 1);

% draw feature
plot3(param.feature_x, param.feature_y, param.feature_z, 'g*');
plot3(param.feature_x(visible_feature_ids), param.feature_y(visible_feature_ids), param.feature_z(visible_feature_ids), 'r*');

% adjust view point of the figure 
com_pos = state_init(1:3);
set(gca,'CameraPosition',[-47.208302215231150 23.034842416831935 11.851346583308723]);
set(gca,'CameraTarget',[1.997222389456938 -0.014617703237565 0.005325191580426]);
set(gca,'CameraUpVector',[0 0 1]);
set(gca,'CameraViewAngle',8.6765);
set(gca, 'XLim', [com_pos(1)-3.2 com_pos(1)+3.6])
set(gca, 'YLim', [com_pos(2)-3.2 com_pos(2)+3.2])
set(gca, 'ZLim', [com_pos(3)-2.1 com_pos(3)+2.1])
drawnow;