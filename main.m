%% init necessary parameters
% init parameter
param_init;


%% generate two stances

tgt_x = 1.8;
tgt_y = 0;
tgt_yaw = 0;

tgt_x = 1.9;
tgt_y = 0;
tgt_yaw = 0;

%% plot two stance poses
fig_id = 1;
fig = figure(fig_id);
set(gcf, 'units','normalized','outerposition',[0 0.2 0.3 0.7]);

clf; hold on;
% show two robot stances
draw_robot_stance(fig, state_soln1, param, 1)
draw_robot_stance(fig, state_soln2, param, 0.2)

% adjust view point of the figure 
com_pos = state_soln1(1:3);
set(gca,'CameraPosition',[com_pos(1)+4 com_pos(2)+6 com_pos(3)+6]);
set(gca,'CameraTarget',[com_pos(1) com_pos(2) com_pos(3)]);
set(gca,'CameraUpVector',[0 0 1]);
set(gca,'CameraViewAngle',6.6765);
set(gca, 'XLim', [com_pos(1)-1.2 com_pos(1)+1.6])
set(gca, 'YLim', [com_pos(2)-1.2 com_pos(2)+1.2])
set(gca, 'ZLim', [com_pos(3)-1.1 com_pos(3)+0.6])
drawnow;