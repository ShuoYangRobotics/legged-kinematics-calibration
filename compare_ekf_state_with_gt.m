function compare_ekf_state_with_gt(est_state_list, gt_state_list, traj_t, fig_name, param)
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
tiledlayout(7,2, 'Padding', 'none', 'TileSpacing', 'compact'); 
nexttile
plot(traj_t(plot_start:plot_end_idx), est_state_list(1,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(1,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Position X', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% lgd.FontSize = lgd_font_size;
nexttile
plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list(8,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(8,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Velocity X', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'interpreter', 'latex');
% lgd.FontSize = lgd_font_size;
nexttile
plot(traj_t(plot_start:plot_end_idx), est_state_list(2,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(2,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Position Y', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% lgd.FontSize = lgd_font_size;
nexttile
plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list(9,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(9,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Velocity Y', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'interpreter', 'latex');
% lgd.FontSize = lgd_font_size;
nexttile
plot(traj_t(plot_start:plot_end_idx), est_state_list(3,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(3,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Position Z', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'Location', 'Northwest', 'interpreter', 'latex');lgd.FontSize = 8;
% lgd.FontSize = lgd_font_size;
nexttile
plot([0 traj_t(plot_start:plot_end_idx-1)], est_state_list(10,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(10,plot_start:plot_end_idx),'-.','LineWidth',line_width)
set(gca,'FontSize',axis_font_size)
title('Velocity Z', 'interpreter', 'latex', 'FontSize',title_font_size)
% lgd = legend('Estimation', 'Ground Truth', 'interpreter', 'latex');
% lgd.FontSize = lgd_font_size;
nexttile([2 1]);
plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(10+(1-1)*param.rho_opt_size+1:10+1*param.rho_opt_size,plot_start:plot_end_idx),'-.','LineWidth',line_width)
ylim([0.1 0.3])
set(gca,'FontSize',axis_font_size)
title(strcat('Front-left Leg', {' '}, {'$\rho$ = ['}, param.rho_opt_str, {'] '}), 'interpreter', 'latex', 'FontSize',title_font_size);
% legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
%              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% lgd.FontSize = lgd_font_size;

nexttile([2 1]);
plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(10+(2-1)*param.rho_opt_size+1:10+2*param.rho_opt_size,plot_start:plot_end_idx),'-.','LineWidth',line_width)
ylim([0.1 0.3])
set(gca,'FontSize',axis_font_size)
title(strcat('Front-right Leg', {' '}, {'$\rho$ = ['}, param.rho_opt_str, {'] '}), 'interpreter', 'latex', 'FontSize',title_font_size);
% legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
%              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% lgd.FontSize = lgd_font_size;

nexttile([2 1]);
plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(10+(3-1)*param.rho_opt_size+1:10+3*param.rho_opt_size,plot_start:plot_end_idx),'-.','LineWidth',line_width)
ylim([0.1 0.3])
set(gca,'FontSize',axis_font_size)
title(strcat('Rear-left Leg', {' '}, {'$\rho$ = ['}, param.rho_opt_str, {'] '}), 'interpreter', 'latex', 'FontSize',title_font_size);
% legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
%              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% lgd.FontSize = lgd_font_size;

nexttile([2 1]);
plot(traj_t(plot_start:plot_end_idx), est_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx),...
    traj_t(plot_start:plot_end_idx), gt_state_list(10+(4-1)*param.rho_opt_size+1:10+4*param.rho_opt_size,plot_start:plot_end_idx),'-.','LineWidth',line_width)
ylim([0.1 0.3])
set(gca,'FontSize',axis_font_size)
title(strcat('Rear-right Leg', {' '}, {'$\rho$ = ['}, param.rho_opt_str, {'] '}), 'interpreter', 'latex', 'FontSize',title_font_size);
% legend([repmat({'Estimation'},1,param.rho_opt_size) repmat({'Ground Truth'},1,param.rho_opt_size)]);
% lgd = legend('$l_t$ Estimation','$l_c$ Estimation', '$o_x$ Estimation','$o_y$ Estimation',...
%              '$l_t$ Ground Truth', '$l_c$ Ground Truth','$o_x$ Ground Truth', '$o_y$ Ground Truth','NumColumns',2, 'interpreter', 'latex', 'Location', 'Southeast');
% lgd.FontSize = lgd_font_size;

end