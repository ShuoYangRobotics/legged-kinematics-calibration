%% init necessary parameters
% init parameter
param_init;


%% generate two stances

tgt_x = 1.8;
tgt_y = 0;
tgt_z = 0;
tgt_yaw = 0;
tgt_pitch = 0;
tgt_roll = 0;

q = quaternion([tgt_yaw tgt_pitch tgt_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(q);
state_init = [tgt_x;tgt_y;tgt_z;w;x;y;z;zeros(3,1);zeros(3,1);zeros(3,1)];
measurement = [zeros(3,1);zeros(3,1);0.3*randn(3,1);zeros(3,1);zeros(3,1)];

tgt_x = 1.9;
tgt_y = 0;
tgt_z = 0;
tgt_yaw = 0;
tgt_pitch = 0;
tgt_roll = 0;

q = quaternion([tgt_yaw tgt_pitch tgt_roll],'eulerd','ZYX','frame');
[w,x,y,z] = parts(q);
state_final = [tgt_x;tgt_y;tgt_z;w;x;y;z;zeros(3,1);zeros(3,1);zeros(3,1)];

%% plot one stance pose
fig_id = 1;
fig = figure(fig_id);
set(gcf, 'units','normalized','outerposition',[0 0.2 0.3 0.7]);

clf; hold on;
draw_robot(fig, state_init, measurement, param, 1)



