function[]= draw_robot(fig, pose, joint_angles, param, color_factor)
% input state
%  1 2 3     4  5  6  7 
% position    orientation 

    % colors are ugly...
    set(0, 'CurrentFigure', fig)
    % draw robot  
    p_er        = pose(1:3);
    q_er        = quaternion(pose(4:7)');
    R_er = quat2rotm(q_er);
    
    % joint angle, automatically construct theta_list according to active
    % legs
    theta_list = zeros(3*param.num_leg,1);
    idx = 1;
    for i=1:param.num_leg 
        if param.active_leg(i) == 1
            theta_list((i-1)*3+1:(i-1)*3+3) = joint_angles((idx-1)*3+1:(idx-1)*3+3);
            idx = idx + 1;
        end
    end

    % draw body
    draw_cube(fig, p_er, R_er,param.vis_bl,param.vis_bw,param.vis_bh,color_factor*[1 0 1])

    % draw legs
    for i=1:param.num_leg 
        if param.active_leg(i) == 1
            theta = theta_list((i-1)*3+1:(i-1)*3+3);
            % draw hip
            p_rhip = autoFunc_fk_hip_pos(theta,param.rho_opt_true(:,i),param.rho_fix(:,i));
            R_rhip = autoFunc_fk_hip_rot(theta,param.rho_opt_true(:,i),param.rho_fix(:,i));
            p_ehip = R_er*p_rhip + p_er;
            R_ehip = R_er*R_rhip;
            draw_cube(fig, p_ehip, R_ehip, 0.05,0.16,0.05,color_factor*[0.2 0.6 1])

            % draw thigh
            p_rthigh = autoFunc_fk_thigh_pos(theta,param.rho_opt_true(:,i),param.rho_fix(:,i));
            R_rthigh = autoFunc_fk_thigh_rot(theta,param.rho_opt_true(:,i),param.rho_fix(:,i));
            p_ethigh = R_er*p_rthigh + p_er;
            R_ethigh = R_er*R_rthigh;
            draw_cube(fig, p_ethigh, R_ethigh, 0.03,0.03,param.lt,color_factor*[0.8 0.2 1])
            % draw calf
            p_rcalf = autoFunc_fk_calf_pos(theta,param.rho_opt_true(:,i),param.rho_fix(:,i));
            R_rcalf = autoFunc_fk_calf_rot(theta,param.rho_opt_true(:,i),param.rho_fix(:,i));
            p_ecalf = R_er*p_rcalf + p_er;
            R_ecalf = R_er*R_rcalf;
            draw_cube(fig, p_ecalf, R_ecalf, 0.03,0.03,param.lt,color_factor*[0.5 0.5 0.8])
        end
    end

    % draw camera FOV
    p_c = [0;0;0]; % center of camera in camera frame
    distance = 10;
    p_lu = [param.horiz_film/2/param.focus_len*distance;
            param.verti_film/2/param.focus_len*distance;
                                               distance];
    p_ll = [param.horiz_film/2/param.focus_len*distance;
            -param.verti_film/2/param.focus_len*distance;
                                               distance];
    p_ru = [-param.horiz_film/2/param.focus_len*distance;
            param.verti_film/2/param.focus_len*distance;
                                               distance];
    p_rl = [-param.horiz_film/2/param.focus_len*distance;
            -param.verti_film/2/param.focus_len*distance;
                                               distance];
    p_wc = R_er*(param.R_rc*p_c +param.p_rc)+p_er; % center of camera in world frame
    p_luc = R_er*(param.R_rc*p_lu +param.p_rc)+p_er; % p_lu in world frame
    p_llc = R_er*(param.R_rc*p_ll +param.p_rc)+p_er; % p_ll in world frame
    p_ruc = R_er*(param.R_rc*p_ru +param.p_rc)+p_er; % p_ru in world frame
    p_rlc = R_er*(param.R_rc*p_rl +param.p_rc)+p_er; % p_rl in world frame
    plot3([p_wc(1) p_luc(1)],[p_wc(2) p_luc(2)],[p_wc(3) p_luc(3)],'Color',[0 0 0],'LineWidth',3);
    plot3([p_wc(1) p_ruc(1)],[p_wc(2) p_ruc(2)],[p_wc(3) p_ruc(3)],'Color',[0 0 0],'LineWidth',3);
    plot3([p_wc(1) p_llc(1)],[p_wc(2) p_llc(2)],[p_wc(3) p_llc(3)],'Color',[0 0 0],'LineWidth',3);
    plot3([p_wc(1) p_rlc(1)],[p_wc(2) p_rlc(2)],[p_wc(3) p_rlc(3)],'Color',[0 0 0],'LineWidth',3);
end