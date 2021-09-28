% some A1 parameter
param.total_mass = 16;
param.g = 9.805;

% used to visualize the robot body
param.vis_bl = 0.62;
param.vis_bw = 0.30;
param.vis_bh = 0.08;

param.vis_lw = 0.03;

% number of legs, leg kinematics and fk jacobian generation 
param.leg_l1 = 0.2;
param.leg_l2 = 0.22;

param.num_leg = 4;


% camera parameters
% transformation between robot frame and camera frame
param.p_rc = [param.vis_bl/2;0;0];
param.R_rc = [ 0  0 1;
              -1  0 0;
               0 -1 0];

param.focus_len = 1;
param.horiz_film = 0.4;
param.verti_film = 0.3;