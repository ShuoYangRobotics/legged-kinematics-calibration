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

q = quaternion(eulerAngles,'eulerd','ZYX','frame')
[w,x,y,z] = parts(q)
state_init = [tgt_x;tgt_y;tgt_z;w;x;y;z;zeros(3,1);zeros(3,1);zeros(3,1)]
measurement = [zeros(3,1);zeros(3,1);zeros(3,1);zeros(3,1);zeros(3,1)]

tgt_x = 1.9;
tgt_y = 0;
tgt_z = 0;
tgt_yaw = 0;
tgt_pitch = 0;
tgt_roll = 0;

q = quaternion(eulerAngles,'eulerd','ZYX','frame')
[w,x,y,z] = parts(q)
state_final = [tgt_x;tgt_y;tgt_z;w;x;y;z;zeros(3,1);zeros(3,1);zeros(3,1)]

%% plot one stance pose
fig_id = 1;
fig = figure(fig_id);
set(gcf, 'units','normalized','outerposition',[0 0.2 0.3 0.7]);

clf; hold on;
draw_robot(fig, state_init, measurement, param, 1)



% adjust view point of the figure 
com_pos = state_init(1:3);
set(gca,'CameraPosition',[com_pos(1)+5 com_pos(2)+5 com_pos(3)+2]);
set(gca,'CameraTarget',[com_pos(1) com_pos(2) com_pos(3)]);
set(gca,'CameraUpVector',[0 0 1]);
set(gca,'CameraViewAngle',8.6765);
set(gca, 'XLim', [com_pos(1)-1.2 com_pos(1)+1.6])
set(gca, 'YLim', [com_pos(2)-1.2 com_pos(2)+1.2])
set(gca, 'ZLim', [com_pos(3)-1.1 com_pos(3)+0.6])
drawnow;