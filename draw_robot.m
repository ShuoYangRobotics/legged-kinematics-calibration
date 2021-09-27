function[]= draw_robot(fig, state, measurement, param, color_factor)
% input state
%  1 2 3     4  5  6  7 
% position    orientation 
    set(0, 'CurrentFigure', fig)
    % draw robot  
    p_er        = state(1:3);
    q_er        = quaternion(state(4:7)');
    R_er = quat2rotm(q_er);
    
    % joint angle 
    q = zeros(3,1);
    q = measurement(7:9);



    % draw body
    draw_cube(fig, p_er, R_er,param.vis_bl,param.vis_bw,param.vis_bh,color_factor*[1 0 1])

    % draw legs and force
    % define a draw arrow function
%     drawArrow = @(x,y,z,varargin) quiver3( x(1),y(1),z(1),x(2)-x(1),y(2)-y(1),z(2)-z(1),0, varargin{:} );
%     for i=1:6
%         thetalist_link1 =[q(1,i); q(2,i)];
%         thetalist_link2 =[q(1,i); q(2,i); q(3,i)];
%         if (mod(i,2) == 0)
%             T1 = FKinSpace(param.M1, param.right_Slist_link1, thetalist_link1);
%         else
%             T1 = FKinSpace(param.M1, param.left_Slist_link1, thetalist_link1);
%         end
%         % link1 pose and orientation
%         p_s1 = T1(1:3,4);
%         R_s1 = T1(1:3,1:3);
%         p_c1 = param.R_cs(:,:,i)*p_s1 + param.t_cs(:,i);
%         R_c1 = param.R_cs(:,:,i)*R_s1;
%         p_e1 = R_ec*p_c1 + t_ec;
%         R_c1 = R_ec*R_c1;
%         draw_cube(fig, p_e1, R_c1,param.leg_l1,param.vis_lw,param.vis_lw,color_factor*[0.2 0.6 1])
% 
%         % link2 pose and orientation
%         if (mod(i,2) == 0)
%             T2 = FKinSpace(param.M2, param.right_Slist_link2, thetalist_link2);
%         else
%             T2 = FKinSpace(param.M2, param.left_Slist_link2, thetalist_link2);
%         end
%         p_s2 = T2(1:3,4);
%         R_s2 = T2(1:3,1:3);
%         p_c2 = param.R_cs(:,:,i)*p_s2 + param.t_cs(:,i);
%         R_c2 = param.R_cs(:,:,i)*R_s2;
%         p_e2 = R_ec*p_c2 + t_ec;
%         R_c2 = R_ec*R_c2;
%         draw_cube(fig, p_e2, R_c2,param.leg_l2,param.vis_lw,param.vis_lw,color_factor*[0 0 1])
% 
% %         if (mod(i,2) == 0)
% %             T3 = FKinSpace(param.M3, param.right_Slist_link2, thetalist_link2);
% %         else
% %             T3 = FKinSpace(param.M3, param.left_Slist_link2, thetalist_link2);
% %         end
% %         p_s3 = T3(1:3,4);
% %         R_s3 = T3(1:3,1:3);
% %         p_c3 = param.R_cs(:,:,i)*p_s3 + param.t_cs(:,i);
% %         p_e3 = R_ec*p_c3 + t_ec;
% %         force_scale_factor = 0.009;
% %         x_arrow = [p_e3(1) p_e3(1)+force_scale_factor*F(1,i)];
% %         y_arrow = [p_e3(2) p_e3(2)+force_scale_factor*F(2,i)];
% %         z_arrow = [p_e3(3) p_e3(3)+force_scale_factor*F(3,i)];
% %         drawArrow(x_arrow,y_arrow,z_arrow,'linewidth',2,'color',color_factor*[1 0 0])
%     end
end