function compare_ekf_true_and_wrong(est_state_list1, est_state_list2, gt_state_list, traj_t, fig_name, param)
plot_start = 1
plot_end_idx = size(traj_t,2)-1
%
% compare est_state_list with gt_state_list;
figure('Name',fig_name,...
    'Units','inches',...
'Position',[0 0 3.5 7],...
'PaperPositionMode','auto')
line_width = 4;
axis_font_size = 19
title_font_size = 19
lgd_font_size = 10
tiledlayout(4,2, 'Padding', 'none', 'TileSpacing', 'compact'); 
nexttile
plot(traj_t(plot_start:plot_end_idx), est_state_list1(1,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(1,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Position X With $l_c = 0.21m$', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% lgd.FontSize = lgd_font_size;

nexttile
plot(traj_t(plot_start:plot_end_idx), est_state_list2(1,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(1,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Position X With $l_c = 0.20m$', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% lgd.FontSize = lgd_font_size;

nexttile
plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list1(8,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(8,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Velocity X With $l_c = 0.21m$', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'interpreter', 'latex');
% lgd.FontSize = lgd_font_size;


nexttile
plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list2(8,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(8,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Velocity X With $l_c = 0.20m$', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'interpreter', 'latex');
% lgd.FontSize = lgd_font_size;

nexttile
plot(traj_t(plot_start:plot_end_idx), est_state_list1(2,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(2,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Position Y With $l_c = 0.21m$', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% lgd.FontSize = lgd_font_size;

nexttile
plot(traj_t(plot_start:plot_end_idx), est_state_list2(2,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(2,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Position Y With $l_c = 0.20m$', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% lgd.FontSize = lgd_font_size;
nexttile
plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list1(9,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(9,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Velocity Y With $l_c = 0.21m$', 'interpreter', 'latex', 'FontSize',title_font_size)

nexttile
plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list2(9,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(9,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Velocity Y With $l_c = 0.20m$', 'interpreter', 'latex', 'FontSize',title_font_size)

end